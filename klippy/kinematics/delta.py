# Code for handling the kinematics of linear delta robots
#
# Copyright (C) 2016-2023  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
import stepper, mathutil
import chelper # For get_effector_normal

try:
    import scipy.optimize
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    # This module might be imported even if scipy isn't used by delta_calibrate,
    # so a warning here might be too noisy if user isn't using advanced calibration.
    # logging.info("SciPy library not found, brentq for lean-aware IK in DeltaCalibration will be disabled.")

# Slow moves once the ratio of tower to XY movement exceeds SLOW_RATIO
SLOW_RATIO = 3.

# Numerical constants for improved stability
EPSILON = 1e-12
MIN_STEP_DIST_FACTOR = 0.5
MAX_ITERATION_COUNT = 100

class DeltaKinematics:
    def __init__(self, toolhead, config):
        # Setup tower rails
        stepper_configs = [config.getsection('stepper_' + a) for a in 'abc']
        rail_a = stepper.LookupMultiRail(
            stepper_configs[0], need_position_minmax = False)
        a_endstop = rail_a.get_homing_info().position_endstop
        rail_b = stepper.LookupMultiRail(
            stepper_configs[1], need_position_minmax = False,
            default_position_endstop=a_endstop)
        rail_c = stepper.LookupMultiRail(
            stepper_configs[2], need_position_minmax = False,
            default_position_endstop=a_endstop)
        self.rails = [rail_a, rail_b, rail_c]
        
        # Setup max velocity with validation
        self.max_velocity, self.max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', self.max_velocity,
            above=0., maxval=self.max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', self.max_accel,
                                          above=0., maxval=self.max_accel)
        
        # Read global delta radius with validation
        self.radius = config.getfloat('delta_radius', above=0.)
        self.effector_joint_radius = config.getfloat('delta_effector_radius', 40.0, above=0.)
        if config.get('delta_effector_radius', None) is None:
            logging.warning(
                "Config option 'delta_effector_radius' not specified in [printer] section."
                " Using default %.2fmm. For accurate probe tilt compensation, this value should be set."
                % (self.effector_joint_radius,))

        # Read per-stepper parameters with improved validation
        self.angles = []
        self.arm_lengths = []
        self.radius_offsets = []
        self.radial_leans_rad = []
        self.tangential_leans_rad = []

        default_angles = [210., 330., 90.]
        arm_length_a_cfg = stepper_configs[0].getfloat('arm_length')
        radius_offset_a_cfg = stepper_configs[0].getfloat('delta_radius_offset', 0.0)

        for i, sconfig in enumerate(stepper_configs):
            self.angles.append(sconfig.getfloat('angle', default_angles[i]))
            current_arm_length = sconfig.getfloat('arm_length', arm_length_a_cfg)
            self.arm_lengths.append(current_arm_length)
            self.radius_offsets.append(sconfig.getfloat('delta_radius_offset',
                                                        radius_offset_a_cfg))
            radial_lean_deg = sconfig.getfloat('radial_lean', 0.0)
            tangential_lean_deg = sconfig.getfloat('tangential_lean', 0.0)
            
            # Validate lean angles to prevent numerical issues
            if abs(radial_lean_deg) > 45.0:
                logging.warning("Radial lean angle %.2f degrees for stepper %s is large and may cause numerical issues"
                               % (radial_lean_deg, chr(ord('a') + i)))
            if abs(tangential_lean_deg) > 45.0:
                logging.warning("Tangential lean angle %.2f degrees for stepper %s is large and may cause numerical issues"
                               % (tangential_lean_deg, chr(ord('a') + i)))
            
            self.radial_leans_rad.append(math.radians(radial_lean_deg))
            self.tangential_leans_rad.append(math.radians(tangential_lean_deg))

        self.arm2 = [arm**2 for arm in self.arm_lengths]

        # Determine tower base locations (at Z=0) in cartesian space with improved validation
        self.towers = [] # Stores (Base_X, Base_Y) for each tower
        for i in range(3):
            eff_radius = self.radius + self.radius_offsets[i]
            if self.arm_lengths[i] <= eff_radius:
                raise config.error(
                    "Arm length %.3f for stepper %s must be greater than its"
                    " effective delta radius %.3f (delta_radius %.3f + offset %.3f)" % (
                        self.arm_lengths[i], chr(ord('a') + i), eff_radius,
                        self.radius, self.radius_offsets[i]))
            
            # Check for reasonable margin between arm length and effective radius
            margin = self.arm_lengths[i] - eff_radius
            if margin < self.arm_lengths[i] * 0.1:  # Less than 10% margin
                logging.warning("Small margin (%.3f) between arm length %.3f and effective radius %.3f for stepper %s"
                               % (margin, self.arm_lengths[i], eff_radius, chr(ord('a') + i)))
            
            angle_rad = math.radians(self.angles[i])
            self.towers.append((math.cos(angle_rad) * eff_radius,
                                math.sin(angle_rad) * eff_radius))

        # Calculate absolute endstops with improved numerical stability
        self.abs_endstops = []
        for i in range(3):
            rail_homing_pos = self.rails[i].get_homing_info().position_endstop
            eff_radius_i = self.radius + self.radius_offsets[i]
            val_under_sqrt = self.arm_lengths[i]**2 - eff_radius_i**2
            if val_under_sqrt < EPSILON:
                logging.warning("Near-zero or negative value under sqrt for endstop calculation (tower %d): %.6f"
                               % (i, val_under_sqrt))
                val_under_sqrt = max(val_under_sqrt, 0.0)
            self.abs_endstops.append(rail_homing_pos + math.sqrt(val_under_sqrt))

        print_radius = config.getfloat('print_radius', self.radius, above=0.)

        # Precompute lean factors for C helper with improved numerical stability
        self.lean_factors = []
        for i in range(3):
            alpha_i_rad = math.radians(self.angles[i])
            theta_r_rad = self.radial_leans_rad[i]
            theta_t_rad = self.tangential_leans_rad[i]

            s_tr = math.sin(theta_r_rad)
            s_tt = math.sin(theta_t_rad)
            c_a = math.cos(alpha_i_rad)
            s_a = math.sin(alpha_i_rad)

            lean_dx_dh_i = s_tr * c_a - s_tt * s_a
            lean_dy_dh_i = s_tr * s_a + s_tt * c_a

            # Improved numerical stability for lean_dz_dh calculation
            sin_sum_sq = s_tr**2 + s_tt**2
            if sin_sum_sq > 1.0:
                if sin_sum_sq > 1.0 + EPSILON:
                    logging.warning("Invalid lean angle combination for tower %d: sin²(θr) + sin²(θt) = %.6f > 1.0"
                                   % (i, sin_sum_sq))
                sin_sum_sq = 1.0
            
            val_for_dz_sqrt = 1.0 - sin_sum_sq
            lean_dz_dh_i = math.sqrt(max(val_for_dz_sqrt, 0.0))
            
            # Warn if lean angles result in very small dz component
            if lean_dz_dh_i < 0.1:
                logging.warning("Very small Z component (%.6f) for tower %d lean angles. This may cause numerical issues."
                               % (lean_dz_dh_i, i))

            self.lean_factors.append( (lean_dx_dh_i, lean_dy_dh_i, lean_dz_dh_i) )

        # Setup itersolve for each rail
        for i in range(3):
            base_tower_x = self.towers[i][0]
            base_tower_y = self.towers[i][1]
            lean_dx_dh, lean_dy_dh, lean_dz_dh = self.lean_factors[i]
            self.rails[i].setup_itersolve('delta_stepper_alloc', self.arm2[i],
                                          base_tower_x, base_tower_y,
                                          lean_dx_dh, lean_dy_dh, lean_dz_dh)
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)

        self.need_home = True
        self.limit_xy2 = -1.
        self.home_position = tuple(
            self._actuator_to_cartesian(self.abs_endstops))
        self.max_z = min([rail.get_homing_info().position_endstop
                          for rail in self.rails])
        self.min_z = config.getfloat('minimum_z_position', 0, maxval=self.max_z)

        # Improved boundary calculations with better numerical stability
        min_arm_val = min(self.arm_lengths)
        self.limit_z = min([ep - arm for ep, arm in zip(self.abs_endstops, self.arm_lengths)]) # Approx
        self.min_arm_length = min_arm_val
        self.min_arm2 = self.min_arm_length**2
        logging.info(
            "Delta max build height %.2fmm (radius tapered above %.2fmm)"
            % (self.max_z, self.limit_z))

        half_min_step_dist = min([r.get_steppers()[0].get_step_dist()
                                  for r in self.rails]) * MIN_STEP_DIST_FACTOR
        
        def ratio_to_xy(ratio, arm_len, r_boundary):
            """Calculate XY boundary with improved numerical stability"""
            if ratio <= 0:
                return 0.0
            val_inside_sqrt = (arm_len**2 / (ratio**2 + 1.)) - half_min_step_dist**2
            val_inside_sqrt = max(val_inside_sqrt, 0.0)  # Ensure non-negative
            return max(0.0, ratio * math.sqrt(val_inside_sqrt) + half_min_step_dist - r_boundary)

        self.slow_xy2 = ratio_to_xy(SLOW_RATIO, self.min_arm_length, self.radius)**2
        self.very_slow_xy2 = ratio_to_xy(2. * SLOW_RATIO, self.min_arm_length, self.radius)**2
        
        max_safe_xy_for_min_arm = 0.0
        if self.min_arm_length > self.radius + EPSILON:
             max_safe_xy_for_min_arm = ratio_to_xy(4. * SLOW_RATIO, self.min_arm_length, self.radius)
        else:
             logging.warning("min_arm_length (%.2f) is not greater than delta_radius (%.2f) "
                             "for max_xy2 boundary calculation." % (self.min_arm_length, self.radius))
        
        self.max_xy2 = min(print_radius, max_safe_xy_for_min_arm)**2
        if print_radius > max_safe_xy_for_min_arm and max_safe_xy_for_min_arm > 0:
            logging.warning("print_radius %.2fmm is larger than kinematically safe radius %.2fmm"
                            % (print_radius, max_safe_xy_for_min_arm))

        max_xy = math.sqrt(self.max_xy2) if self.max_xy2 > 0 else 0.0
        logging.info("Delta max build radius %.2fmm (moves slowed past %.2fmm"
                     " and %.2fmm)"
                     % (max_xy, math.sqrt(self.slow_xy2) if self.slow_xy2 > 0 else 0.0,
                        math.sqrt(self.very_slow_xy2) if self.very_slow_xy2 > 0 else 0.0))
        self.axes_min = toolhead.Coord(-max_xy, -max_xy, self.min_z, 0.)
        self.axes_max = toolhead.Coord(max_xy, max_xy, self.max_z, 0.)
        self.set_position([0., 0., 0.], "")

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def _get_effective_tower_xy_and_z(self, tower_index, carriage_height_on_rail):
        """Calculate effective tower position with improved numerical stability"""
        h_i = carriage_height_on_rail
        B_ix, B_iy = self.towers[tower_index]
        alpha_i_rad = math.radians(self.angles[tower_index])
        theta_r_rad = self.radial_leans_rad[tower_index]
        theta_t_rad = self.tangential_leans_rad[tower_index]

        # Pre-compute trigonometric values for efficiency
        sin_theta_r = math.sin(theta_r_rad)
        sin_theta_t = math.sin(theta_t_rad)
        cos_alpha = math.cos(alpha_i_rad)
        sin_alpha = math.sin(alpha_i_rad)

        eff_carriage_x = B_ix + h_i * (sin_theta_r * cos_alpha - sin_theta_t * sin_alpha)
        eff_carriage_y = B_iy + h_i * (sin_theta_r * sin_alpha + sin_theta_t * cos_alpha)
        
        # Improved numerical stability for Z calculation
        sin_sum_sq = sin_theta_r**2 + sin_theta_t**2
        if sin_sum_sq > 1.0:
            if sin_sum_sq > 1.0 + EPSILON:
                logging.warning("Invalid lean angle combination for tower %d in effective position calculation: %.6f"
                               % (tower_index, sin_sum_sq))
            sin_sum_sq = 1.0
        
        val_under_sqrt_z = 1.0 - sin_sum_sq
        eff_carriage_z = h_i * math.sqrt(max(val_under_sqrt_z, 0.0))
        
        return eff_carriage_x, eff_carriage_y, eff_carriage_z

    def _actuator_to_cartesian(self, spos): # spos are carriage heights on rail
        """Forward kinematics with improved error handling"""
        sphere_coords = []
        for i in range(3):
            try:
                eff_x, eff_y, eff_z = self._get_effective_tower_xy_and_z(i, spos[i])
                sphere_coords.append( (eff_x, eff_y, eff_z) )
            except Exception as e:
                logging.error("Error calculating effective position for tower %d: %s" % (i, str(e)))
                raise
        
        try:
            return mathutil.trilateration(sphere_coords, self.arm2)
        except Exception as e:
            logging.error("Trilateration failed with sphere coords: %s, arm2: %s" % (sphere_coords, self.arm2))
            raise

    def calc_position(self, stepper_positions):
        spos = [stepper_positions[rail.get_name()] for rail in self.rails]
        return self._actuator_to_cartesian(spos)

    def set_position(self, newpos, homing_axes):
        for rail in self.rails:
            rail.set_position(newpos)
        self.limit_xy2 = -1.
        if homing_axes == "xyz":
            self.need_home = False

    def clear_homing_state(self, clear_axes):
        if clear_axes:
            self.limit_xy2 = -1
            self.need_home = True

    def home(self, homing_state):
        homing_state.set_axes([0, 1, 2])
        forcepos = list(self.home_position)
        val_under_sqrt = max(self.arm2) - self.max_xy2
        val_under_sqrt = max(val_under_sqrt, 0.0)  # Ensure non-negative
        forcepos[2] = -1.5 * math.sqrt(val_under_sqrt)
        homing_state.home_rails(self.rails, forcepos, self.home_position)

    def check_move(self, move):
        """Move validation with improved boundary checking"""
        end_pos = move.end_pos
        end_xy2 = end_pos[0]**2 + end_pos[1]**2
        if end_xy2 <= self.limit_xy2 and not move.axes_d[2]:
            return
        if self.need_home:
            raise move.move_error("Must home first")
        
        end_z = end_pos[2]
        limit_xy2 = self.max_xy2
        
        # Improved Z-dependent radius calculation
        if end_z > self.limit_z:
            above_z_limit = end_z - self.limit_z
            if above_z_limit > self.min_arm_length:
                # Completely unreachable
                allowed_radius = 0.0
            else:
                val_under_sqrt = self.min_arm2 - (self.min_arm_length - above_z_limit)**2
                val_under_sqrt = max(val_under_sqrt, 0.0)  # Ensure non-negative
                allowed_radius = max(0.0, self.radius - math.sqrt(val_under_sqrt))
            limit_xy2 = min(limit_xy2, allowed_radius**2)
        
        # Boundary check with improved error handling
        if end_xy2 > limit_xy2 or end_z > self.max_z or end_z < self.min_z:
            if (end_pos[:2] != self.home_position[:2]
                or end_z < self.min_z or end_z > self.home_position[2]):
                raise move.move_error("Move out of bounds: pos=(%.3f,%.3f,%.3f), limits: xy2=%.3f, z=[%.3f,%.3f]"
                                     % (end_pos[0], end_pos[1], end_z, math.sqrt(limit_xy2), self.min_z, self.max_z))
            limit_xy2 = -1.
        
        # Z-axis move speed limiting
        if move.axes_d[2]:
            z_abs_delta = abs(move.axes_d[2])
            if z_abs_delta > EPSILON:
                z_ratio = move.move_d / z_abs_delta
            else:
                z_ratio = SLOW_RATIO * 10
            move.limit_speed(self.max_z_velocity * z_ratio,
                             self.max_z_accel * z_ratio)
            limit_xy2 = -1.
        
        # XY speed limiting for extreme positions
        extreme_xy2 = max(end_xy2, move.start_pos[0]**2 + move.start_pos[1]**2)
        if extreme_xy2 > self.slow_xy2:
            r = 0.5
            if extreme_xy2 > self.very_slow_xy2:
                r = 0.25
            move.limit_speed(self.max_velocity * r, self.max_accel * r)
            limit_xy2 = -1.
        
        self.limit_xy2 = min(limit_xy2, self.slow_xy2 if self.slow_xy2 > 0 else -1.)

    def get_status(self, eventtime):
        return {
            'homed_axes': '' if self.need_home else 'xyz',
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
            'cone_start_z': self.limit_z,
        }

    def get_calibration(self):
        endstops = [rail.get_homing_info().position_endstop
                    for rail in self.rails]
        stepdists = [rail.get_steppers()[0].get_step_dist()
                     for rail in self.rails]
        return DeltaCalibration(self, self.radius, self.angles, self.arm_lengths,
                                endstops, stepdists, self.radius_offsets,
                                self.effector_joint_radius,
                                self.radial_leans_rad, self.tangential_leans_rad)

    def get_effector_normal(self, nozzle_x, nozzle_y, nozzle_z):
        """Calculate effector normal with improved numerical stability"""
        ffi_main, ffi_lib = chelper.get_ffi()
        carriage_heights = []
        
        # Get carriage heights for each tower
        for i in range(3):
            stepper_obj = self.rails[i].get_steppers()[0]
            sk = stepper_obj._stepper_kinematics
            try:
                h = ffi_lib.itersolve_calc_position_from_coord(
                    sk, nozzle_x, nozzle_y, nozzle_z
                )
                carriage_heights.append(h)
            except Exception as e:
                logging.error("Failed to calculate carriage height for tower %d: %s" % (i, str(e)))
                return (0.0, 0.0, 1.0)

        # Calculate effective carriage positions
        C_coords = []
        for i in range(3):
            try:
                eff_cx, eff_cy, eff_cz = self._get_effective_tower_xy_and_z(i, carriage_heights[i])
                C_coords.append( (eff_cx, eff_cy, eff_cz) )
            except Exception as e:
                logging.error("Failed to calculate effective carriage position for tower %d: %s" % (i, str(e)))
                return (0.0, 0.0, 1.0)

        # Calculate effector joint positions
        E_coords_xy = []
        for i in range(3):
            angle_rad = math.radians(self.angles[i])
            ex = nozzle_x + self.effector_joint_radius * math.cos(angle_rad)
            ey = nozzle_y + self.effector_joint_radius * math.sin(angle_rad)
            E_coords_xy.append( (ex, ey) )

        # Calculate effector joint Z positions with improved error handling
        E_coords_z = []
        for i in range(3):
            dx_sq = (E_coords_xy[i][0] - C_coords[i][0])**2
            dy_sq = (E_coords_xy[i][1] - C_coords[i][1])**2
            arm_len_sq = self.arm2[i]
            val_under_sqrt = arm_len_sq - dx_sq - dy_sq
            
            if val_under_sqrt < EPSILON:
                logging.warning(
                    "Probe tilt: Unreachable effector joint %d for nozzle (%.3f,%.3f,%.3f). "
                    "ArmLenSq=%.3f, dXsq+dYsq=%.3f, diff=%.6f"
                    % (i, nozzle_x, nozzle_y, nozzle_z, arm_len_sq, dx_sq + dy_sq, val_under_sqrt))
                return (0.0, 0.0, 1.0)
            
            e_z = C_coords[i][2] - math.sqrt(val_under_sqrt)
            E_coords_z.append(e_z)

        # Calculate normal vector with improved numerical stability
        E = [ (E_coords_xy[i][0], E_coords_xy[i][1], E_coords_z[i]) for i in range(3) ]
        
        # Calculate cross product vectors
        v1_x, v1_y, v1_z = E[1][0]-E[0][0], E[1][1]-E[0][1], E[1][2]-E[0][2]
        v2_x, v2_y, v2_z = E[2][0]-E[0][0], E[2][1]-E[0][1], E[2][2]-E[0][2]
        
        # Cross product
        nx = v1_y*v2_z - v1_z*v2_y
        ny = v1_z*v2_x - v1_x*v2_z
        nz = v1_x*v2_y - v1_y*v2_x
        
        # Normalize with improved error handling
        norm_len = math.sqrt(nx**2 + ny**2 + nz**2)
        if norm_len < EPSILON:
            logging.warning("Probe tilt: Zero or very small normal vector length (%.9f) for nozzle (%.3f,%.3f,%.3f)."
                            % (norm_len, nozzle_x, nozzle_y, nozzle_z))
            return (0.0, 0.0, 1.0)
        
        nx /= norm_len
        ny /= norm_len
        nz /= norm_len
        
        # Ensure normal points up (positive Z component)
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

        self.towers = []
        for i in range(3):
            eff_radius = self.radius + self.radius_offsets[i]
            angle_rad = math.radians(self.angles[i])
            self.towers.append((math.cos(angle_rad) * eff_radius,
                                math.sin(angle_rad) * eff_radius))

        self.abs_endstops = []
        for i in range(3):
            eff_radius_i = self.radius + self.radius_offsets[i]
            val_inside_sqrt = self.arms[i]**2 - eff_radius_i**2
            if val_inside_sqrt < 0:
                logging.warning("DeltaCalibration init: Arm %.2f for tower %d is too short for effective radius %.2f."
                                % (self.arms[i], i, eff_radius_i))
                val_inside_sqrt = 0
            self.abs_endstops.append(self.endstops[i] + math.sqrt(val_inside_sqrt))

    def coordinate_descent_params(self, is_extended=False):
        adj_params = [
            'radius', 'radius_offset_a', 'radius_offset_b', 'radius_offset_c',
            'angle_a', 'angle_b',
            'arm_a', 'arm_b', 'arm_c',
            'endstop_a', 'endstop_b', 'endstop_c',
            'radial_lean_a', 'radial_lean_b', 'radial_lean_c',
            'tangential_lean_a', 'tangential_lean_b', 'tangential_lean_c'
        ]
        params = {'radius': self.radius}
        for i, axis in enumerate('abc'):
            params['radius_offset_'+axis] = self.radius_offsets[i]
            params['angle_'+axis] = self.angles[i]
            params['arm_'+axis] = self.arms[i]
            params['endstop_'+axis] = self.endstops[i]
            params['stepdist_'+axis] = self.stepdists[i]
            params['radial_lean_'+axis] = math.degrees(self.radial_leans_rad[i])
            params['tangential_lean_'+axis] = math.degrees(self.tangential_leans_rad[i])
        return adj_params, params

    def new_calibration(self, params):
        radius = params['radius']
        current_angles = list(self.angles)
        current_angles[0] = params.get('angle_a', self.angles[0])
        current_angles[1] = params.get('angle_b', self.angles[1])
        new_radius_offsets = [params.get('radius_offset_'+axis, self.radius_offsets[i]) for i, axis in enumerate('abc')]
        new_arms = [params.get('arm_'+axis, self.arms[i]) for i, axis in enumerate('abc')]
        new_endstops = [params.get('endstop_'+axis, self.endstops[i]) for i, axis in enumerate('abc')]
        new_radial_leans_rad = [math.radians(params.get('radial_lean_'+axis, math.degrees(self.radial_leans_rad[i]))) for i, axis in enumerate('abc')]
        new_tangential_leans_rad = [math.radians(params.get('tangential_lean_'+axis, math.degrees(self.tangential_leans_rad[i]))) for i, axis in enumerate('abc')]

        return DeltaCalibration(self.kinematics_parent, radius, current_angles, new_arms, new_endstops,
                                self.stepdists, new_radius_offsets,
                                self.effector_joint_radius,
                                new_radial_leans_rad, new_tangential_leans_rad)

    def _get_effective_tower_xy_and_z(self, tower_index, carriage_height_on_rail):
        h_i = carriage_height_on_rail
        B_ix, B_iy = self.towers[tower_index]
        alpha_i_rad = math.radians(self.angles[tower_index])
        theta_r_rad = self.radial_leans_rad[tower_index]
        theta_t_rad = self.tangential_leans_rad[tower_index]
        eff_carriage_x = B_ix + h_i * (math.sin(theta_r_rad) * math.cos(alpha_i_rad)
                                     - math.sin(theta_t_rad) * math.sin(alpha_i_rad))
        eff_carriage_y = B_iy + h_i * (math.sin(theta_r_rad) * math.sin(alpha_i_rad)
                                     + math.sin(theta_t_rad) * math.cos(alpha_i_rad))
        val_under_sqrt_z = 1.0 - math.sin(theta_r_rad)**2 - math.sin(theta_t_rad)**2
        eff_carriage_z = h_i * math.sqrt(max(0.0, val_under_sqrt_z))
        return eff_carriage_x, eff_carriage_y, eff_carriage_z

    def get_position_from_stable(self, stable_position): # FK
        sphere_coords = []
        for i in range(3):
            h_i = self.endstops[i] - (stable_position[i] * self.stepdists[i])
            eff_x, eff_y, eff_z = self._get_effective_tower_xy_and_z(i, h_i)
            sphere_coords.append( (eff_x, eff_y, eff_z) )
        return mathutil.trilateration(sphere_coords, [a**2 for a in self.arms])

    def calc_stable_position(self, coord): # IK (coord is nozzle_xyz = (nx, ny, nz))
        nx, ny, nz = coord
        steppos_h_on_rail = []

        for i in range(3):
            def f_h_i(h_i_trial):
                ctx_i, cty_i, ctz_i = self._get_effective_tower_xy_and_z(i, h_i_trial)
                dist_sq = (nx - ctx_i)**2 + (ny - cty_i)**2 + (nz - ctz_i)**2
                return dist_sq - self.arms[i]**2

            dx_simple = self.towers[i][0] - nx
            dy_simple = self.towers[i][1] - ny
            val_under_sqrt_simple = self.arms[i]**2 - dx_simple**2 - dy_simple**2
            if val_under_sqrt_simple < 0:
                logging.error("DeltaCalibration IK: Unreachable point for tower %d (initial guess): %s" % (i, coord,))
                raise ValueError("Delta kinematics: Unreachable point for tower %d in IK initial guess" % i)
            h_i_approx = math.sqrt(val_under_sqrt_simple) + nz

            h_min_bracket = h_i_approx - 50
            h_max_bracket = h_i_approx + 50
            h_min_bracket = max(h_min_bracket, -100.0)
            h_max_bracket = min(h_max_bracket, self.endstops[i] + 100.0)

            h_i_solution = None
            try:
                if not SCIPY_AVAILABLE:
                    raise ImportError("SciPy not available for brentq in DeltaCalibration")
                h_i_solution = scipy.optimize.brentq(f_h_i, h_min_bracket, h_max_bracket, xtol=1e-6, rtol=1e-6)
            except Exception as e:
                logging.warning("SciPy brentq for lean-aware IK in DeltaCalibration failed (tower %d, coord %s): %s. "
                                "Falling back to C-level lean-aware IK." % (i, coord, str(e)))
                if self.kinematics_parent is not None:
                    ffi_main, ffi_lib = chelper.get_ffi()
                    sk = self.kinematics_parent.rails[i].get_steppers()[0]._stepper_kinematics
                    h_i_solution = ffi_lib.itersolve_calc_position_from_coord(sk, nx, ny, nz)
                else:
                    logging.error("DeltaCalibration: kinematics_parent not available for C-IK fallback.")
                    raise ValueError("Delta kinematics: kinematics_parent unavailable for fallback IK")

            steppos_h_on_rail.append(h_i_solution)

        return [(self.endstops[i] - h_i) / self.stepdists[i]
                for i, h_i in enumerate(steppos_h_on_rail)]

    def save_state(self, configfile):
        configfile.set('printer', 'delta_radius', "%.6f" % (self.radius,))
        configfile.set('printer', 'delta_effector_radius', "%.6f" % (self.effector_joint_radius,))
        gcode_lines = ["delta_radius: %.6f" % self.radius,
                       "delta_effector_radius: %.6f" % self.effector_joint_radius]
        for i, axis in enumerate('abc'):
            section = 'stepper_'+axis
            configfile.set(section, 'angle', "%.6f" % (self.angles[i],))
            configfile.set(section, 'arm_length', "%.6f" % (self.arms[i],))
            configfile.set(section, 'position_endstop', "%.6f" % (self.endstops[i],))
            configfile.set(section, 'delta_radius_offset', "%.6f" % (self.radius_offsets[i],))
            configfile.set(section, 'radial_lean', "%.4f" % (math.degrees(self.radial_leans_rad[i]),))
            configfile.set(section, 'tangential_lean', "%.4f" % (math.degrees(self.tangential_leans_rad[i]),))
            gcode_lines.append(
                "%s: pos_endstop: %.4f angle: %.4f arm: %.4f roff: %.4f rlean: %.4f tlean: %.4f" %
                (section, self.endstops[i], self.angles[i], self.arms[i],
                 self.radius_offsets[i], math.degrees(self.radial_leans_rad[i]),
                 math.degrees(self.tangential_leans_rad[i])))
        gcode = configfile.get_printer().lookup_object("gcode")
        gcode.respond_info("\n".join(gcode_lines))

def load_kinematics(toolhead, config):
    return DeltaKinematics(toolhead, config)
