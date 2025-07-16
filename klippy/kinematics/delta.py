# Code for handling the kinematics of linear delta robots - Enhanced Version
#
# Copyright (C) 2016-2023  Kevin O'Connor <kevin@koconnor.net>
# Improvements by Assistant
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import math
import logging
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass
from enum import Enum
import stepper
import mathutil
import chelper  # For get_effector_normal

try:
    import scipy.optimize
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

# Constants
SLOW_RATIO = 3.0
DEFAULT_ANGLES = [210.0, 330.0, 90.0]
DEFAULT_EFFECTOR_RADIUS = 40.0
MIN_STEP_TOLERANCE = 1e-6
CONVERGENCE_TOLERANCE = 1e-6
MAX_BRACKET_SEARCH = 100.0
SAFETY_MARGIN = 0.5

class TowerIndex(Enum):
    """Enum for tower indices to improve code readability"""
    A = 0
    B = 1
    C = 2

@dataclass
class TowerParams:
    """Container for tower-specific parameters"""
    angle: float
    arm_length: float
    radius_offset: float
    radial_lean_rad: float
    tangential_lean_rad: float
    position_endstop: float
    
    @property
    def arm_length_squared(self) -> float:
        return self.arm_length ** 2
    
    @property
    def effective_radius(self) -> float:
        return self.radius_offset  # Will be added to global radius

@dataclass
class KinematicsLimits:
    """Container for kinematic limits and boundaries"""
    max_xy2: float
    slow_xy2: float
    very_slow_xy2: float
    limit_z: float
    max_z: float
    min_z: float

class DeltaKinematics:
    """Enhanced delta kinematics implementation with improved performance and maintainability"""
    
    def __init__(self, toolhead, config):
        self.toolhead = toolhead
        self.config = config
        
        # Initialize tower parameters
        self._setup_tower_parameters()
        
        # Setup velocity limits
        self._setup_velocity_limits()
        
        # Setup geometry
        self._setup_geometry()
        
        # Setup stepper rails
        self._setup_rails()
        
        # Calculate kinematic limits
        self._calculate_limits()
        
        # Initialize state
        self._initialize_state()
        
        # Log configuration
        self._log_configuration()

    def _setup_tower_parameters(self):
        """Initialize tower-specific parameters with validation"""
        stepper_configs = [self.config.getsection(f'stepper_{axis}') for axis in 'abc']
        
        # Read global parameters
        self.radius = self.config.getfloat('delta_radius', above=0.)
        self.effector_joint_radius = self.config.getfloat(
            'delta_effector_radius', DEFAULT_EFFECTOR_RADIUS, above=0.)
        
        if self.config.get('delta_effector_radius', None) is None:
            logging.warning(
                f"Config option 'delta_effector_radius' not specified. "
                f"Using default {self.effector_joint_radius:.2f}mm.")
        
        # Setup tower parameters
        self.towers = []
        arm_length_a = stepper_configs[0].getfloat('arm_length')
        radius_offset_a = stepper_configs[0].getfloat('delta_radius_offset', 0.0)
        
        for i, sconfig in enumerate(stepper_configs):
            tower = TowerParams(
                angle=sconfig.getfloat('angle', DEFAULT_ANGLES[i]),
                arm_length=sconfig.getfloat('arm_length', arm_length_a),
                radius_offset=sconfig.getfloat('delta_radius_offset', radius_offset_a),
                radial_lean_rad=math.radians(sconfig.getfloat('radial_lean', 0.0)),
                tangential_lean_rad=math.radians(sconfig.getfloat('tangential_lean', 0.0)),
                position_endstop=0.0  # Will be set after rail setup
            )
            self._validate_tower_parameters(tower, i)
            self.towers.append(tower)

    def _validate_tower_parameters(self, tower: TowerParams, index: int):
        """Validate tower parameters for geometric consistency"""
        effective_radius = self.radius + tower.radius_offset
        
        if tower.arm_length <= effective_radius:
            axis_name = chr(ord('a') + index)
            raise self.config.error(
                f"Arm length {tower.arm_length:.3f} for stepper {axis_name} "
                f"must be greater than effective delta radius {effective_radius:.3f} "
                f"(delta_radius {self.radius:.3f} + offset {tower.radius_offset:.3f})")
        
        # Validate lean angles don't exceed physical limits
        total_lean = math.sqrt(tower.radial_lean_rad**2 + tower.tangential_lean_rad**2)
        if total_lean > math.pi / 4:  # 45 degrees
            logging.warning(f"Tower {index} has large lean angle: {math.degrees(total_lean):.1f}°")

    def _setup_velocity_limits(self):
        """Setup velocity and acceleration limits"""
        self.max_velocity, self.max_accel = self.toolhead.get_max_velocity()
        self.max_z_velocity = self.config.getfloat(
            'max_z_velocity', self.max_velocity, above=0., maxval=self.max_velocity)
        self.max_z_accel = self.config.getfloat(
            'max_z_accel', self.max_accel, above=0., maxval=self.max_accel)

    def _setup_geometry(self):
        """Calculate tower positions and geometric parameters"""
        # Calculate tower base positions
        self.tower_positions = []
        for tower in self.towers:
            eff_radius = self.radius + tower.radius_offset
            angle_rad = math.radians(tower.angle)
            self.tower_positions.append((
                math.cos(angle_rad) * eff_radius,
                math.sin(angle_rad) * eff_radius
            ))
        
        # Precompute lean transformation factors
        self.lean_factors = []
        for tower in self.towers:
            self.lean_factors.append(self._calculate_lean_factors(tower))

    def _calculate_lean_factors(self, tower: TowerParams) -> Tuple[float, float, float]:
        """Calculate lean transformation factors for a tower"""
        alpha_rad = math.radians(tower.angle)
        theta_r = tower.radial_lean_rad
        theta_t = tower.tangential_lean_rad
        
        sin_r, cos_r = math.sin(theta_r), math.cos(theta_r)
        sin_t, cos_t = math.sin(theta_t), math.cos(theta_t)
        sin_a, cos_a = math.sin(alpha_rad), math.cos(alpha_rad)
        
        lean_dx = sin_r * cos_a - sin_t * sin_a
        lean_dy = sin_r * sin_a + sin_t * cos_a
        
        # Ensure z-component is positive (pointing up)
        z_factor_squared = max(0.0, 1.0 - sin_r**2 - sin_t**2)
        lean_dz = math.sqrt(z_factor_squared)
        
        return lean_dx, lean_dy, lean_dz

    def _setup_rails(self):
        """Setup stepper rails with proper configuration"""
        stepper_configs = [self.config.getsection(f'stepper_{axis}') for axis in 'abc']
        
        # Create rails
        self.rails = []
        rail_a = stepper.LookupMultiRail(
            stepper_configs[0], need_position_minmax=False)
        a_endstop = rail_a.get_homing_info().position_endstop
        
        self.rails.append(rail_a)
        
        for i in range(1, 3):
            rail = stepper.LookupMultiRail(
                stepper_configs[i], need_position_minmax=False,
                default_position_endstop=a_endstop)
            self.rails.append(rail)
        
        # Update tower endstop positions
        for i, (rail, tower) in enumerate(zip(self.rails, self.towers)):
            self.towers[i] = TowerParams(
                angle=tower.angle,
                arm_length=tower.arm_length,
                radius_offset=tower.radius_offset,
                radial_lean_rad=tower.radial_lean_rad,
                tangential_lean_rad=tower.tangential_lean_rad,
                position_endstop=rail.get_homing_info().position_endstop
            )
        
        # Setup itersolve for each rail
        for i, (rail, tower) in enumerate(zip(self.rails, self.towers)):
            base_x, base_y = self.tower_positions[i]
            lean_dx, lean_dy, lean_dz = self.lean_factors[i]
            
            rail.setup_itersolve(
                'delta_stepper_alloc', tower.arm_length_squared,
                base_x, base_y, lean_dx, lean_dy, lean_dz)
        
        # Register step generators
        for stepper_obj in self.get_steppers():
            stepper_obj.set_trapq(self.toolhead.get_trapq())
            self.toolhead.register_step_generator(stepper_obj.generate_steps)

    def _calculate_limits(self):
        """Calculate kinematic limits and boundaries"""
        # Calculate absolute endstops
        self.abs_endstops = []
        for tower, position in zip(self.towers, self.tower_positions):
            eff_radius = self.radius + tower.radius_offset
            val_under_sqrt = max(0.0, tower.arm_length_squared - eff_radius**2)
            self.abs_endstops.append(tower.position_endstop + math.sqrt(val_under_sqrt))
        
        # Calculate Z limits
        self.max_z = min(self.abs_endstops)
        self.min_z = self.config.getfloat('minimum_z_position', 0, maxval=self.max_z)
        
        # Calculate print radius
        print_radius = self.config.getfloat('print_radius', self.radius, above=0.)
        
        # Calculate movement limits
        min_arm_length = min(tower.arm_length for tower in self.towers)
        self.min_arm_length = min_arm_length
        
        # Approximate Z limit where radius starts tapering
        self.limit_z = min(endstop - tower.arm_length 
                          for endstop, tower in zip(self.abs_endstops, self.towers))
        
        # Calculate XY movement boundaries
        half_min_step = min(rail.get_steppers()[0].get_step_dist() 
                           for rail in self.rails) * 0.5
        
        self.slow_xy2 = self._calculate_xy_boundary(
            SLOW_RATIO, min_arm_length, half_min_step) ** 2
        self.very_slow_xy2 = self._calculate_xy_boundary(
            2.0 * SLOW_RATIO, min_arm_length, half_min_step) ** 2
        
        # Calculate maximum safe XY radius
        max_safe_xy = 0.0
        if min_arm_length > self.radius:
            max_safe_xy = self._calculate_xy_boundary(
                4.0 * SLOW_RATIO, min_arm_length, half_min_step)
        else:
            logging.warning(
                f"Minimum arm length ({min_arm_length:.2f}) is not greater than "
                f"delta radius ({self.radius:.2f}) for boundary calculation")
        
        self.max_xy2 = min(print_radius, max_safe_xy) ** 2
        
        if print_radius > max_safe_xy > 0:
            logging.warning(
                f"Print radius {print_radius:.2f}mm exceeds kinematically safe "
                f"radius {max_safe_xy:.2f}mm")
        
        # Store limits for status reporting
        self.limits = KinematicsLimits(
            max_xy2=self.max_xy2,
            slow_xy2=self.slow_xy2,
            very_slow_xy2=self.very_slow_xy2,
            limit_z=self.limit_z,
            max_z=self.max_z,
            min_z=self.min_z
        )

    def _calculate_xy_boundary(self, ratio: float, arm_length: float, 
                              half_step: float) -> float:
        """Calculate XY boundary for given ratio and parameters"""
        val_under_sqrt = max(0.0, (arm_length**2 / (ratio**2 + 1.0)) - half_step**2)
        return ratio * math.sqrt(val_under_sqrt) + half_step - self.radius

    def _initialize_state(self):
        """Initialize kinematic state variables"""
        self.need_home = True
        self.limit_xy2 = -1.0
        
        # Calculate home position
        self.home_position = tuple(self._actuator_to_cartesian(self.abs_endstops))
        
        # Calculate axes limits
        max_xy = math.sqrt(self.max_xy2) if self.max_xy2 > 0 else 0.0
        self.axes_min = self.toolhead.Coord(-max_xy, -max_xy, self.min_z, 0.)
        self.axes_max = self.toolhead.Coord(max_xy, max_xy, self.max_z, 0.)
        
        # Set initial position
        self.set_position([0., 0., 0.], "")

    def _log_configuration(self):
        """Log configuration information"""
        logging.info(
            f"Delta max build height {self.max_z:.2f}mm "
            f"(radius tapered above {self.limit_z:.2f}mm)")
        
        max_xy = math.sqrt(self.max_xy2) if self.max_xy2 > 0 else 0.0
        slow_xy = math.sqrt(self.slow_xy2) if self.slow_xy2 > 0 else 0.0
        very_slow_xy = math.sqrt(self.very_slow_xy2) if self.very_slow_xy2 > 0 else 0.0
        
        logging.info(
            f"Delta max build radius {max_xy:.2f}mm "
            f"(moves slowed past {slow_xy:.2f}mm and {very_slow_xy:.2f}mm)")

    def get_steppers(self) -> List:
        """Get all stepper objects"""
        return [stepper_obj for rail in self.rails for stepper_obj in rail.get_steppers()]

    def _get_effective_tower_position(self, tower_index: int, 
                                    carriage_height: float) -> Tuple[float, float, float]:
        """Calculate effective tower position accounting for lean"""
        tower = self.towers[tower_index]
        base_x, base_y = self.tower_positions[tower_index]
        lean_dx, lean_dy, lean_dz = self.lean_factors[tower_index]
        
        return (
            base_x + carriage_height * lean_dx,
            base_y + carriage_height * lean_dy,
            carriage_height * lean_dz
        )

    def _actuator_to_cartesian(self, stepper_positions: List[float]) -> List[float]:
        """Convert stepper positions to cartesian coordinates (forward kinematics)"""
        sphere_coords = []
        for i, pos in enumerate(stepper_positions):
            eff_pos = self._get_effective_tower_position(i, pos)
            sphere_coords.append(eff_pos)
        
        arm_lengths_squared = [tower.arm_length_squared for tower in self.towers]
        return mathutil.trilateration(sphere_coords, arm_lengths_squared)

    def calc_position(self, stepper_positions: Dict[str, float]) -> List[float]:
        """Calculate cartesian position from stepper positions"""
        positions = [stepper_positions[rail.get_name()] for rail in self.rails]
        return self._actuator_to_cartesian(positions)

    def set_position(self, newpos: List[float], homing_axes: str):
        """Set current position"""
        for rail in self.rails:
            rail.set_position(newpos)
        self.limit_xy2 = -1.0
        
        if homing_axes == "xyz":
            self.need_home = False

    def clear_homing_state(self, clear_axes: str):
        """Clear homing state"""
        if clear_axes:
            self.limit_xy2 = -1.0
            self.need_home = True

    def home(self, homing_state):
        """Perform homing sequence"""
        homing_state.set_axes([0, 1, 2])
        
        # Calculate force position for homing
        forcepos = list(self.home_position)
        max_arm_squared = max(tower.arm_length_squared for tower in self.towers)
        val_under_sqrt = max(0.0, max_arm_squared - self.max_xy2)
        forcepos[2] = -1.5 * math.sqrt(val_under_sqrt)
        
        homing_state.home_rails(self.rails, forcepos, self.home_position)

    def check_move(self, move):
        """Check if move is valid and apply speed limits"""
        end_pos = move.end_pos
        end_xy2 = end_pos[0]**2 + end_pos[1]**2
        
        # Quick check for moves within cached limits
        if end_xy2 <= self.limit_xy2 and not move.axes_d[2]:
            return
        
        # Check if homing is required
        if self.need_home:
            raise move.move_error("Must home first")
        
        # Calculate dynamic XY limits based on Z position
        end_z = end_pos[2]
        limit_xy2 = self.max_xy2
        
        if end_z > self.limit_z:
            limit_xy2 = self._calculate_z_dependent_xy_limit(end_z)
        
        # Check position limits
        if (end_xy2 > limit_xy2 or end_z > self.max_z or end_z < self.min_z):
            if self._is_homing_move(end_pos):
                limit_xy2 = -1.0
            else:
                raise move.move_error()
        
        # Apply Z-axis speed limits
        if move.axes_d[2]:
            self._apply_z_speed_limits(move)
            limit_xy2 = -1.0
        
        # Apply XY speed limits for extreme positions
        extreme_xy2 = max(end_xy2, move.start_pos[0]**2 + move.start_pos[1]**2)
        if extreme_xy2 > self.slow_xy2:
            self._apply_xy_speed_limits(move, extreme_xy2)
            limit_xy2 = -1.0
        
        # Cache the limit for future moves
        self.limit_xy2 = min(limit_xy2, self.slow_xy2 if self.slow_xy2 > 0 else -1.0)

    def _calculate_z_dependent_xy_limit(self, z_pos: float) -> float:
        """Calculate XY limit based on Z position (cone constraint)"""
        above_limit = z_pos - self.limit_z
        remaining_arm = self.min_arm_length - above_limit
        
        val_under_sqrt = max(0.0, self.min_arm_length**2 - remaining_arm**2)
        allowed_radius = max(0.0, self.radius - math.sqrt(val_under_sqrt))
        
        return allowed_radius**2

    def _is_homing_move(self, end_pos: List[float]) -> bool:
        """Check if this is a homing move"""
        return (end_pos[:2] == self.home_position[:2] and
                self.min_z <= end_pos[2] <= self.home_position[2])

    def _apply_z_speed_limits(self, move):
        """Apply speed limits for Z-axis movement"""
        z_move = abs(move.axes_d[2])
        if z_move > 1e-9:
            z_ratio = move.move_d / z_move
            move.limit_speed(
                self.max_z_velocity * z_ratio,
                self.max_z_accel * z_ratio
            )

    def _apply_xy_speed_limits(self, move, extreme_xy2: float):
        """Apply speed limits for extreme XY positions"""
        speed_ratio = 0.25 if extreme_xy2 > self.very_slow_xy2 else 0.5
        move.limit_speed(
            self.max_velocity * speed_ratio,
            self.max_accel * speed_ratio
        )

    def get_status(self, eventtime: float) -> Dict[str, Any]:
        """Get current status"""
        return {
            'homed_axes': '' if self.need_home else 'xyz',
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
            'cone_start_z': self.limit_z,
        }

    def get_calibration(self):
        """Get calibration object for delta calibration"""
        endstops = [tower.position_endstop for tower in self.towers]
        stepdists = [rail.get_steppers()[0].get_step_dist() for rail in self.rails]
        angles = [tower.angle for tower in self.towers]
        arm_lengths = [tower.arm_length for tower in self.towers]
        radius_offsets = [tower.radius_offset for tower in self.towers]
        radial_leans = [tower.radial_lean_rad for tower in self.towers]
        tangential_leans = [tower.tangential_lean_rad for tower in self.towers]
        
        return DeltaCalibration(
            self, self.radius, angles, arm_lengths, endstops, stepdists,
            radius_offsets, self.effector_joint_radius,
            radial_leans, tangential_leans
        )

    def get_effector_normal(self, nozzle_x: float, nozzle_y: float, 
                           nozzle_z: float) -> Tuple[float, float, float]:
        """Calculate effector normal vector for tilt compensation"""
        # Calculate carriage heights
        carriage_heights = []
        for i, rail in enumerate(self.rails):
            stepper_obj = rail.get_steppers()[0]
            stepper_kinematics = stepper_obj._stepper_kinematics
            
            ffi_main, ffi_lib = chelper.get_ffi()
            height = ffi_lib.itersolve_calc_position_from_coord(
                stepper_kinematics, nozzle_x, nozzle_y, nozzle_z)
            carriage_heights.append(height)
        
        # Calculate effective carriage positions
        carriage_coords = []
        for i, height in enumerate(carriage_heights):
            eff_pos = self._get_effective_tower_position(i, height)
            carriage_coords.append(eff_pos)
        
        # Calculate effector joint positions
        effector_joints = []
        for i, tower in enumerate(self.towers):
            angle_rad = math.radians(tower.angle)
            joint_x = nozzle_x + self.effector_joint_radius * math.cos(angle_rad)
            joint_y = nozzle_y + self.effector_joint_radius * math.sin(angle_rad)
            
            # Calculate joint Z position
            carriage_pos = carriage_coords[i]
            dx = joint_x - carriage_pos[0]
            dy = joint_y - carriage_pos[1]
            distance_squared = dx**2 + dy**2
            
            if distance_squared >= tower.arm_length_squared:
                logging.warning(
                    f"Probe tilt: Unreachable effector joint {i} for nozzle "
                    f"({nozzle_x:.3f}, {nozzle_y:.3f}, {nozzle_z:.3f})")
                return (0.0, 0.0, 1.0)
            
            val_under_sqrt = tower.arm_length_squared - distance_squared
            joint_z = carriage_pos[2] - math.sqrt(val_under_sqrt)
            effector_joints.append((joint_x, joint_y, joint_z))
        
        # Calculate normal vector from cross product
        return self._calculate_normal_from_points(effector_joints)

    def _calculate_normal_from_points(self, points: List[Tuple[float, float, float]]) -> Tuple[float, float, float]:
        """Calculate normal vector from three points using cross product"""
        # Calculate vectors from first point to other two
        v1 = (points[1][0] - points[0][0], 
              points[1][1] - points[0][1], 
              points[1][2] - points[0][2])
        
        v2 = (points[2][0] - points[0][0], 
              points[2][1] - points[0][1], 
              points[2][2] - points[0][2])
        
        # Calculate cross product
        nx = v1[1] * v2[2] - v1[2] * v2[1]
        ny = v1[2] * v2[0] - v1[0] * v2[2]
        nz = v1[0] * v2[1] - v1[1] * v2[0]
        
        # Normalize
        norm_length = math.sqrt(nx**2 + ny**2 + nz**2)
        if norm_length < 1e-9:
            logging.warning(
                f"Probe tilt: Zero length normal vector for points {points}")
            return (0.0, 0.0, 1.0)
        
        nx /= norm_length
        ny /= norm_length
        nz /= norm_length
        
        # Ensure normal points upward
        return (-nx, -ny, -nz) if nz < 0.0 else (nx, ny, nz)


# Delta parameter calibration for DELTA_CALIBRATE tool
class DeltaCalibration:
    def __init__(self, kinematics_parent, radius, angles, arms, endstops, stepdists,
                 radius_offsets=None, effector_joint_radius=40.0,
                 radial_leans_rad=None, tangential_leans_rad=None):
        self.kinematics_parent = kinematics_parent
        self.radius = radius
        self.angles = angles
        self.arms = arms
        self.endstops = endstops
        self.stepdists = stepdists
        self.radius_offsets = radius_offsets if radius_offsets is not None else [0.0, 0.0, 0.0]
        self.effector_joint_radius = effector_joint_radius
        self.radial_leans_rad = radial_leans_rad if radial_leans_rad is not None else [0.0, 0.0, 0.0]
        self.tangential_leans_rad = tangential_leans_rad if tangential_leans_rad is not None else [0.0, 0.0, 0.0]

        # Validate input parameters
        self._validate_parameters()
        
        # Pre-compute commonly used values
        self._precompute_constants()
        
        # Cache for expensive computations
        self._tower_cache = {}
        self._endstop_cache = {}

    def _validate_parameters(self):
        """Validate calibration parameters and raise errors for invalid configurations."""
        if self.radius <= 0:
            raise ValueError(f"Delta radius must be positive, got: {self.radius}")
        
        if len(self.angles) != 3 or len(self.arms) != 3 or len(self.endstops) != 3:
            raise ValueError("Must have exactly 3 angles, arms, and endstops")
        
        if any(arm <= 0 for arm in self.arms):
            raise ValueError(f"All arm lengths must be positive, got: {self.arms}")
        
        if any(step <= 0 for step in self.stepdists):
            raise ValueError(f"All step distances must be positive, got: {self.stepdists}")
        
        # Check that arms are long enough for effective radius
        for i in range(3):
            eff_radius = self.radius + self.radius_offsets[i]
            if self.arms[i] <= eff_radius:
                raise ValueError(
                    f"Arm length {self.arms[i]:.3f} for tower {i} must be greater than "
                    f"effective radius {eff_radius:.3f}")

    def _precompute_constants(self):
        """Pre-compute commonly used values for better performance."""
        # Pre-compute squared arm lengths
        self.arm_squares = [arm**2 for arm in self.arms]
        
        # Pre-compute angle radians
        self.angle_rads = [math.radians(angle) for angle in self.angles]
        
        # Pre-compute trigonometric values
        self.cos_angles = [math.cos(rad) for rad in self.angle_rads]
        self.sin_angles = [math.sin(rad) for rad in self.angle_rads]
        
        # Pre-compute lean trigonometric values
        self.sin_radial_leans = [math.sin(lean) for lean in self.radial_leans_rad]
        self.cos_radial_leans = [math.cos(lean) for lean in self.radial_leans_rad]
        self.sin_tangential_leans = [math.sin(lean) for lean in self.tangential_leans_rad]
        self.cos_tangential_leans = [math.cos(lean) for lean in self.tangential_leans_rad]
        
        # Update towers and endstops
        self._update_towers()
        self._update_endstops()

    def _update_towers(self):
        """Update tower positions using pre-computed values."""
        self.towers = []
        for i in range(3):
            eff_radius = self.radius + self.radius_offsets[i]
            self.towers.append((
                self.cos_angles[i] * eff_radius,
                self.sin_angles[i] * eff_radius
            ))

    def _update_endstops(self):
        """Update absolute endstop positions with better error handling."""
        self.abs_endstops = []
        for i in range(3):
            eff_radius_i = self.radius + self.radius_offsets[i]
            val_inside_sqrt = self.arm_squares[i] - eff_radius_i**2
            if val_inside_sqrt < 0:
                logging.warning(
                    f"DeltaCalibration init: Arm {self.arms[i]:.2f} for tower {i} "
                    f"is too short for effective radius {eff_radius_i:.2f}. "
                    f"Using minimum safe value.")
                val_inside_sqrt = 0
            self.abs_endstops.append(self.endstops[i] + math.sqrt(val_inside_sqrt))

    def coordinate_descent_params(self, is_extended=False):
        """Get parameters for coordinate descent optimization."""
        base_params = [
            'radius', 'radius_offset_a', 'radius_offset_b', 'radius_offset_c',
            'angle_a', 'angle_b',
            'arm_a', 'arm_b', 'arm_c',
            'endstop_a', 'endstop_b', 'endstop_c'
        ]
        
        extended_params = [
            'radial_lean_a', 'radial_lean_b', 'radial_lean_c',
            'tangential_lean_a', 'tangential_lean_b', 'tangential_lean_c'
        ]
        
        adj_params = base_params + (extended_params if is_extended else [])
        
        params = {'radius': self.radius}
        for i, axis in enumerate('abc'):
            params[f'radius_offset_{axis}'] = self.radius_offsets[i]
            params[f'angle_{axis}'] = self.angles[i]
            params[f'arm_{axis}'] = self.arms[i]
            params[f'endstop_{axis}'] = self.endstops[i]
            params[f'stepdist_{axis}'] = self.stepdists[i]
            params[f'radial_lean_{axis}'] = math.degrees(self.radial_leans_rad[i])
            params[f'tangential_lean_{axis}'] = math.degrees(self.tangential_leans_rad[i])
        
        return adj_params, params

    def new_calibration(self, params):
        """Create a new calibration instance with updated parameters."""
        try:
            radius = params['radius']
            current_angles = list(self.angles)
            current_angles[0] = params.get('angle_a', self.angles[0])
            current_angles[1] = params.get('angle_b', self.angles[1])
            
            new_radius_offsets = [
                params.get(f'radius_offset_{axis}', self.radius_offsets[i]) 
                for i, axis in enumerate('abc')
            ]
            new_arms = [
                params.get(f'arm_{axis}', self.arms[i]) 
                for i, axis in enumerate('abc')
            ]
            new_endstops = [
                params.get(f'endstop_{axis}', self.endstops[i]) 
                for i, axis in enumerate('abc')
            ]
            new_radial_leans_rad = [
                math.radians(params.get(f'radial_lean_{axis}', math.degrees(self.radial_leans_rad[i]))) 
                for i, axis in enumerate('abc')
            ]
            new_tangential_leans_rad = [
                math.radians(params.get(f'tangential_lean_{axis}', math.degrees(self.tangential_leans_rad[i]))) 
                for i, axis in enumerate('abc')
            ]

            return DeltaCalibration(
                self.kinematics_parent, radius, current_angles, new_arms, new_endstops,
                self.stepdists, new_radius_offsets, self.effector_joint_radius,
                new_radial_leans_rad, new_tangential_leans_rad
            )
        except Exception as e:
            logging.error(f"Failed to create new calibration: {e}")
            raise

    def _get_effective_tower_xy_and_z(self, tower_index, carriage_height_on_rail):
        """Calculate effective tower position with lean compensation using pre-computed values."""
        h_i = carriage_height_on_rail
        B_ix, B_iy = self.towers[tower_index]
        
        # Use pre-computed trigonometric values
        cos_alpha = self.cos_angles[tower_index]
        sin_alpha = self.sin_angles[tower_index]
        sin_theta_r = self.sin_radial_leans[tower_index]
        sin_theta_t = self.sin_tangential_leans[tower_index]
        
        eff_carriage_x = B_ix + h_i * (sin_theta_r * cos_alpha - sin_theta_t * sin_alpha)
        eff_carriage_y = B_iy + h_i * (sin_theta_r * sin_alpha + sin_theta_t * cos_alpha)
        
        # Calculate Z component with better numerical stability
        val_under_sqrt_z = max(0.0, 1.0 - sin_theta_r**2 - sin_theta_t**2)
        eff_carriage_z = h_i * math.sqrt(val_under_sqrt_z)
        
        return eff_carriage_x, eff_carriage_y, eff_carriage_z

    def get_position_from_stable(self, stable_position):
        """Forward kinematics: Convert stable position to cartesian coordinates."""
        try:
            sphere_coords = []
            for i in range(3):
                h_i = self.endstops[i] - (stable_position[i] * self.stepdists[i])
                eff_x, eff_y, eff_z = self._get_effective_tower_xy_and_z(i, h_i)
                sphere_coords.append((eff_x, eff_y, eff_z))
            
            return mathutil.trilateration(sphere_coords, self.arm_squares)
        except Exception as e:
            logging.error(f"Forward kinematics failed for position {stable_position}: {e}")
            raise

    def calc_stable_position(self, coord):
        """Inverse kinematics: Convert cartesian coordinates to stable position."""
        nx, ny, nz = coord
        steppos_h_on_rail = []

        for i in range(3):
            try:
                h_i_solution = self._solve_tower_height(i, nx, ny, nz)
                steppos_h_on_rail.append(h_i_solution)
            except Exception as e:
                logging.error(f"IK failed for tower {i}, coord {coord}: {e}")
                raise ValueError(f"Delta kinematics: IK failed for tower {i}")

        return [(self.endstops[i] - h_i) / self.stepdists[i]
                for i, h_i in enumerate(steppos_h_on_rail)]

    def _solve_tower_height(self, tower_index, nx, ny, nz):
        """Solve for tower height using multiple fallback methods."""
        def f_h_i(h_i_trial):
            ctx_i, cty_i, ctz_i = self._get_effective_tower_xy_and_z(tower_index, h_i_trial)
            dist_sq = (nx - ctx_i)**2 + (ny - cty_i)**2 + (nz - ctz_i)**2
            return dist_sq - self.arm_squares[tower_index]

        # Calculate initial guess with better numerical stability
        dx_simple = self.towers[tower_index][0] - nx
        dy_simple = self.towers[tower_index][1] - ny
        val_under_sqrt_simple = self.arm_squares[tower_index] - dx_simple**2 - dy_simple**2
        
        if val_under_sqrt_simple < 0:
            logging.error(f"DeltaCalibration IK: Unreachable point for tower {tower_index}: {(nx, ny, nz)}")
            raise ValueError(f"Delta kinematics: Unreachable point for tower {tower_index}")
        
        h_i_approx = math.sqrt(val_under_sqrt_simple) + nz

        # Improved bracket calculation
        bracket_range = min(100.0, max(50.0, abs(h_i_approx) * 0.5))
        h_min_bracket = max(h_i_approx - bracket_range, -200.0)
        h_max_bracket = min(h_i_approx + bracket_range, self.endstops[tower_index] + 200.0)

        # Try SciPy first if available
        if SCIPY_AVAILABLE:
            try:
                return scipy.optimize.brentq(f_h_i, h_min_bracket, h_max_bracket, 
                                           xtol=1e-8, rtol=1e-8, maxiter=100)
            except Exception as e:
                logging.warning(f"SciPy brentq failed for tower {tower_index}: {e}")

        # Fallback to C-level IK
        if self.kinematics_parent is not None:
            try:
                ffi_main, ffi_lib = chelper.get_ffi()
                sk = self.kinematics_parent.rails[tower_index].get_steppers()[0]._stepper_kinematics
                return ffi_lib.itersolve_calc_position_from_coord(sk, nx, ny, nz)
            except Exception as e:
                logging.warning(f"C-level IK failed for tower {tower_index}: {e}")

        # Final fallback: simple iterative solver
        logging.warning(f"Using simple iterative solver for tower {tower_index}")
        return self._simple_iterative_solver(f_h_i, h_i_approx, bracket_range)

    def _simple_iterative_solver(self, func, initial_guess, bracket_range, max_iter=50, tolerance=1e-6):
        """Simple iterative solver as final fallback."""
        x = initial_guess
        for i in range(max_iter):
            f_x = func(x)
            if abs(f_x) < tolerance:
                return x
            
            # Simple derivative approximation
            h = bracket_range * 0.001
            f_x_h = func(x + h)
            df_dx = (f_x_h - f_x) / h
            
            if abs(df_dx) < 1e-12:
                break
            
            x_new = x - f_x / df_dx
            
            # Ensure we stay within reasonable bounds
            if abs(x_new - x) > bracket_range:
                x_new = x - math.copysign(bracket_range * 0.1, f_x / df_dx)
            
            x = x_new
        
        logging.warning(f"Simple iterative solver did not converge, returning best guess: {x}")
        return x

    def save_state(self, configfile):
        """Save calibration state to config file with improved formatting."""
        try:
            # Set main printer parameters
            configfile.set('printer', 'delta_radius', f"{self.radius:.6f}")
            configfile.set('printer', 'delta_effector_radius', f"{self.effector_joint_radius:.6f}")
            
            # Prepare gcode response lines
            gcode_lines = [
                f"Delta Calibration Results:",
                f"  delta_radius: {self.radius:.6f}",
                f"  delta_effector_radius: {self.effector_joint_radius:.6f}",
                ""
            ]
            
            # Set stepper parameters
            for i, axis in enumerate('abc'):
                section = f'stepper_{axis}'
                configfile.set(section, 'angle', f"{self.angles[i]:.6f}")
                configfile.set(section, 'arm_length', f"{self.arms[i]:.6f}")
                configfile.set(section, 'position_endstop', f"{self.endstops[i]:.6f}")
                configfile.set(section, 'delta_radius_offset', f"{self.radius_offsets[i]:.6f}")
                configfile.set(section, 'radial_lean', f"{math.degrees(self.radial_leans_rad[i]):.4f}")
                configfile.set(section, 'tangential_lean', f"{math.degrees(self.tangential_leans_rad[i]):.4f}")
                
                gcode_lines.append(
                    f"  {section}: endstop={self.endstops[i]:.4f} angle={self.angles[i]:.4f} "
                    f"arm={self.arms[i]:.4f} r_offset={self.radius_offsets[i]:.4f} "
                    f"r_lean={math.degrees(self.radial_leans_rad[i]):.4f} "
                    f"t_lean={math.degrees(self.tangential_leans_rad[i]):.4f}"
                )
            
            # Send response via gcode
            gcode = configfile.get_printer().lookup_object("gcode")
            gcode.respond_info("\n".join(gcode_lines))
            
        except Exception as e:
            logging.error(f"Failed to save calibration state: {e}")
            raise

    def get_calibration_info(self):
        """Get comprehensive calibration information for debugging."""
        info = {
            'radius': self.radius,
            'effector_joint_radius': self.effector_joint_radius,
            'towers': self.towers,
            'abs_endstops': self.abs_endstops,
            'parameters': {}
        }
        
        for i, axis in enumerate('abc'):
            info['parameters'][f'stepper_{axis}'] = {
                'angle': self.angles[i],
                'arm_length': self.arms[i],
                'endstop': self.endstops[i],
                'radius_offset': self.radius_offsets[i],
                'radial_lean_deg': math.degrees(self.radial_leans_rad[i]),
                'tangential_lean_deg': math.degrees(self.tangential_leans_rad[i])
            }
        
        return info
        
        
    def load_kinematics(toolhead, config):
        return DeltaKinematics(toolhead, config)
