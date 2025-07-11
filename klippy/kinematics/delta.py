# Code for handling the kinematics of linear delta robots
#
# Copyright (C) 2016-2023  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
import stepper, mathutil

# Slow moves once the ratio of tower to XY movement exceeds SLOW_RATIO
SLOW_RATIO = 3.

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
        # Setup max velocity
        self.max_velocity, self.max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', self.max_velocity,
            above=0., maxval=self.max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', self.max_accel,
                                          above=0., maxval=self.max_accel)
        # Read global delta radius
        self.radius = config.getfloat('delta_radius', above=0.)
        # Read per-stepper parameters
        self.angles = []
        self.arm_lengths = []
        self.radius_offsets = [] # Store individual radius offsets

        default_angles = [210., 330., 90.]
        # Stepper_a's arm_length is mandatory and default for others
        # Stepper_a's delta_radius_offset defaults to 0.0 and is default for others
        # Arm lengths must be greater than effective radius for that tower.
        # Check this after effective radius is known.
        arm_length_a_cfg = stepper_configs[0].getfloat('arm_length') # No default, must be present
        radius_offset_a_cfg = stepper_configs[0].getfloat('delta_radius_offset', 0.0)

        for i, sconfig in enumerate(stepper_configs):
            self.angles.append(sconfig.getfloat('angle', default_angles[i]))
            # Arm length: use own, else stepper_a's
            current_arm_length = sconfig.getfloat('arm_length', arm_length_a_cfg)
            self.arm_lengths.append(current_arm_length)
            # Radius offset: use own, else stepper_a's (which defaults to 0)
            self.radius_offsets.append(sconfig.getfloat('delta_radius_offset',
                                                        radius_offset_a_cfg))

        self.arm2 = [arm**2 for arm in self.arm_lengths]

        # Determine tower locations in cartesian space using effective radii
        self.towers = []
        for i in range(3):
            eff_radius = self.radius + self.radius_offsets[i]
            # Check if arm length is sufficient for this effective radius
            if self.arm_lengths[i] <= eff_radius:
                raise config.error(
                    "Arm length %.3f for stepper %s must be greater than its"
                    " effective delta radius %.3f (delta_radius %.3f + offset %.3f)" % (
                        self.arm_lengths[i], chr(ord('a') + i), eff_radius,
                        self.radius, self.radius_offsets[i]))
            angle_rad = math.radians(self.angles[i])
            self.towers.append((math.cos(angle_rad) * eff_radius,
                                math.sin(angle_rad) * eff_radius))

        # Calculate absolute endstops (used for homing_position and limit_z)
        self.abs_endstops = []
        for i in range(3):
            rail_homing_pos = self.rails[i].get_homing_info().position_endstop
            eff_radius_i = self.radius + self.radius_offsets[i]
            # arm_lengths[i] > eff_radius_i check is done above
            self.abs_endstops.append(rail_homing_pos
                                     + math.sqrt(self.arm_lengths[i]**2 - eff_radius_i**2))

        print_radius = config.getfloat('print_radius', self.radius, above=0.)

        for i in range(3): # Use index i to access per-tower lists
            self.rails[i].setup_itersolve('delta_stepper_alloc', self.arm2[i],
                                          self.towers[i][0], self.towers[i][1])
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)

        # Setup boundary checks
        self.need_home = True
        self.limit_xy2 = -1.
        self.home_position = tuple(
            self._actuator_to_cartesian(self.abs_endstops)) # uses new towers/arms
        self.max_z = min([rail.get_homing_info().position_endstop
                          for rail in self.rails])
        self.min_z = config.getfloat('minimum_z_position', 0, maxval=self.max_z)

        min_arm_val = min(self.arm_lengths)
        self.limit_z = min([ep - arm for ep, arm in zip(self.abs_endstops, self.arm_lengths)])
        self.min_arm_length = min_arm_val
        self.min_arm2 = self.min_arm_length**2
        logging.info(
            "Delta max build height %.2fmm (radius tapered above %.2fmm)"
            % (self.max_z, self.limit_z))

        # Find the point where an XY move could result in excessive tower movement
        # These estimations use min_arm_length and global radius for conservatism.
        half_min_step_dist = min([r.get_steppers()[0].get_step_dist()
                                  for r in self.rails]) * .5

        # Define ratio_to_xy using the actual global radius for boundary calculations
        # as a simplification. A more precise boundary would be complex.
        def ratio_to_xy(ratio, arm_len, r_boundary):
            val_inside_sqrt = (arm_len**2 / (ratio**2 + 1.)) - half_min_step_dist**2
            if val_inside_sqrt < 0.: val_inside_sqrt = 0. # avoid domain error
            return (ratio * math.sqrt(val_inside_sqrt)
                    + half_min_step_dist - r_boundary)

        self.slow_xy2 = ratio_to_xy(SLOW_RATIO, self.min_arm_length, self.radius)**2
        self.very_slow_xy2 = ratio_to_xy(2. * SLOW_RATIO, self.min_arm_length, self.radius)**2
        # max_xy2 uses print_radius, ensure arm > radius for the min_arm_length case
        max_safe_xy_for_min_arm = 0.0
        if self.min_arm_length > self.radius: # Check to prevent math error
             max_safe_xy_for_min_arm = ratio_to_xy(4. * SLOW_RATIO, self.min_arm_length, self.radius)
        else: # if min_arm_length <= self.radius, this indicates a config issue for this check
             logging.warning("min_arm_length (%.2f) is not greater than delta_radius (%.2f) "
                             "for max_xy2 boundary calculation." % (self.min_arm_length, self.radius))

        self.max_xy2 = min(print_radius, max_safe_xy_for_min_arm)**2
        if print_radius > max_safe_xy_for_min_arm and max_safe_xy_for_min_arm > 0:
            logging.warning("print_radius %.2fmm is larger than kinematically safe radius %.2fmm"
                            % (print_radius, max_safe_xy_for_min_arm))

        max_xy = math.sqrt(self.max_xy2)
        logging.info("Delta max build radius %.2fmm (moves slowed past %.2fmm"
                     " and %.2fmm)"
                     % (max_xy, math.sqrt(self.slow_xy2),
                        math.sqrt(self.very_slow_xy2)))
        self.axes_min = toolhead.Coord(-max_xy, -max_xy, self.min_z, 0.)
        self.axes_max = toolhead.Coord(max_xy, max_xy, self.max_z, 0.)
        self.set_position([0., 0., 0.], "")
    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]
    def _actuator_to_cartesian(self, spos):
        sphere_coords = [(t[0], t[1], sp) for t, sp in zip(self.towers, spos)]
        return mathutil.trilateration(sphere_coords, self.arm2)
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
        # Clearing homing state for each axis individually is not implemented
        if clear_axes:
            self.limit_xy2 = -1
            self.need_home = True
    def home(self, homing_state):
        # All axes are homed simultaneously
        homing_state.set_axes([0, 1, 2])
        forcepos = list(self.home_position)
        forcepos[2] = -1.5 * math.sqrt(max(self.arm2)-self.max_xy2)
        homing_state.home_rails(self.rails, forcepos, self.home_position)
    def check_move(self, move):
        end_pos = move.end_pos
        end_xy2 = end_pos[0]**2 + end_pos[1]**2
        if end_xy2 <= self.limit_xy2 and not move.axes_d[2]:
            # Normal XY move
            return
        if self.need_home:
            raise move.move_error("Must home first")
        end_z = end_pos[2]
        limit_xy2 = self.max_xy2
        if end_z > self.limit_z:
            above_z_limit = end_z - self.limit_z
            allowed_radius = self.radius - math.sqrt(
                self.min_arm2 - (self.min_arm_length - above_z_limit)**2
            )
            limit_xy2 = min(limit_xy2, allowed_radius**2)
        if end_xy2 > limit_xy2 or end_z > self.max_z or end_z < self.min_z:
            # Move out of range - verify not a homing move
            if (end_pos[:2] != self.home_position[:2]
                or end_z < self.min_z or end_z > self.home_position[2]):
                raise move.move_error()
            limit_xy2 = -1.
        if move.axes_d[2]:
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(self.max_z_velocity * z_ratio,
                             self.max_z_accel * z_ratio)
            limit_xy2 = -1.
        # Limit the speed/accel of this move if is is at the extreme
        # end of the build envelope
        extreme_xy2 = max(end_xy2, move.start_pos[0]**2 + move.start_pos[1]**2)
        if extreme_xy2 > self.slow_xy2:
            r = 0.5
            if extreme_xy2 > self.very_slow_xy2:
                r = 0.25
            move.limit_speed(self.max_velocity * r, self.max_accel * r)
            limit_xy2 = -1.
        self.limit_xy2 = min(limit_xy2, self.slow_xy2)
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
        # Pass radius_offsets to DeltaCalibration constructor
        return DeltaCalibration(self.radius, self.angles, self.arm_lengths,
                                endstops, stepdists, self.radius_offsets)

# Delta parameter calibration for DELTA_CALIBRATE tool
class DeltaCalibration:
    def __init__(self, radius, angles, arms, endstops, stepdists,
                 radius_offsets=None): # Added radius_offsets
        self.radius = radius # Global delta_radius
        self.angles = angles # List of 3 tower angles [angle_a, angle_b, angle_c]
        self.arms = arms # List of 3 arm lengths [arm_a, arm_b, arm_c]
        self.endstops = endstops # List of 3 endstop positions
        self.stepdists = stepdists # List of 3 step distances (not calibrated)
        if radius_offsets is None:
            # For backward compatibility or if not provided during direct instantiation
            self.radius_offsets = [0.0, 0.0, 0.0]
        else:
            self.radius_offsets = radius_offsets # List of 3 radius offsets

        # Calculate the XY cartesian coordinates of the delta towers
        self.towers = []
        for i in range(3):
            eff_radius = self.radius + self.radius_offsets[i]
            angle_rad = math.radians(self.angles[i])
            self.towers.append((math.cos(angle_rad) * eff_radius,
                                math.sin(angle_rad) * eff_radius))

        # Calculate the absolute Z height of each tower endstop
        # This calculation uses the effective radius and arm length for each tower.
        self.abs_endstops = []
        for i in range(3):
            eff_radius_i = self.radius + self.radius_offsets[i]
            # Ensure arm_length is greater than effective_radius for this tower
            if self.arms[i]**2 <= eff_radius_i**2:
                # This condition should ideally be caught by config validation
                # or handled gracefully if it occurs mid-calibration due to extreme values.
                # For calculation, sqrt of negative is problematic.
                # Defaulting to a large value or erroring might be options.
                # For now, assume valid parameters from config or previous step.
                # A robust solver should prevent params from reaching here in an invalid state.
                # If arms[i] == eff_radius_i, sqrt is 0, which is valid.
                val_inside_sqrt = self.arms[i]**2 - eff_radius_i**2
                if val_inside_sqrt < 0: # Should not happen with sane params
                    logging.warning("Arm length %.3f for tower %d is less than or equal to"
                                    " effective radius %.3f during DeltaCalibration init."
                                    % (self.arms[i], i, eff_radius_i))
                    # Fallback to prevent math error, though this indicates a deeper issue
                    val_inside_sqrt = 0
                self.abs_endstops.append(self.endstops[i] + math.sqrt(val_inside_sqrt))
            else:
                self.abs_endstops.append(self.endstops[i]
                                         + math.sqrt(self.arms[i]**2 - eff_radius_i**2))


    def coordinate_descent_params(self, is_extended):
        # Determine adjustment parameters for coordinate_descent
        # Now includes individual arm lengths and radius offsets by default.
        # angle_c is often kept fixed or derived to prevent rotation of the frame,
        # so only angle_a and angle_b are typically adjusted.
        adj_params = [
            'radius', # Global delta radius
            'radius_offset_a', 'radius_offset_b', 'radius_offset_c',
            'angle_a', 'angle_b', # Calibrate two angles, third is fixed/derived
            'arm_a', 'arm_b', 'arm_c',
            'endstop_a', 'endstop_b', 'endstop_c'
        ]
        # The 'is_extended' flag is no longer used to add arm_a,b,c as they are default.
        # It could be repurposed if DELTA_ANALYZE takes on a new role for even more params.

        params = {'radius': self.radius}
        for i, axis in enumerate('abc'):
            params['radius_offset_'+axis] = self.radius_offsets[i]
            params['angle_'+axis] = self.angles[i] # All angles into params dict
            params['arm_'+axis] = self.arms[i]
            params['endstop_'+axis] = self.endstops[i]
            # stepdist is not calibrated but required by other logic if new_calibration were to use it
            params['stepdist_'+axis] = self.stepdists[i]
        return adj_params, params

    def new_calibration(self, params):
        # Create a new calibration object from coordinate_descent params
        radius = params['radius']
        # If angle_c was fixed, it needs to be restored here.
        # The 'params' dict from coordinate_descent will only contain optimized params.
        # We need to ensure all necessary state (like fixed angle_c or stepdists) is preserved.

        current_angles = list(self.angles) # Start with original angles
        current_angles[0] = params.get('angle_a', self.angles[0])
        current_angles[1] = params.get('angle_b', self.angles[1])
        # angle_c (self.angles[2]) remains as it was if not in adj_params
        # Or, if it IS in adj_params (e.g. 'angle_c'), it would be params['angle_c']

        new_radius_offsets = [params.get('radius_offset_a', self.radius_offsets[0]),
                              params.get('radius_offset_b', self.radius_offsets[1]),
                              params.get('radius_offset_c', self.radius_offsets[2])]
        new_arms = [params.get('arm_a', self.arms[0]),
                    params.get('arm_b', self.arms[1]),
                    params.get('arm_c', self.arms[2])]
        new_endstops = [params.get('endstop_a', self.endstops[0]),
                        params.get('endstop_b', self.endstops[1]),
                        params.get('endstop_c', self.endstops[2])]

        # stepdists are fixed and must be carried over from the original object (self.stepdists)
        return DeltaCalibration(radius, current_angles, new_arms, new_endstops,
                                self.stepdists, new_radius_offsets)
    def get_position_from_stable(self, stable_position):
        # Return cartesian coordinates for the given stable_position
        sphere_coords = [
            (t[0], t[1], es - sp * sd)
            for sd, t, es, sp in zip(self.stepdists, self.towers,
                                     self.abs_endstops, stable_position) ]
        return mathutil.trilateration(sphere_coords, [a**2 for a in self.arms])
    def calc_stable_position(self, coord):
        # Return a stable_position from a cartesian coordinate
        steppos = [
            math.sqrt(a**2 - (t[0]-coord[0])**2 - (t[1]-coord[1])**2) + coord[2]
            for t, a in zip(self.towers, self.arms) ]
        return [(ep - sp) / sd
                for sd, ep, sp in zip(self.stepdists,
                                      self.abs_endstops, steppos)]
    def save_state(self, configfile):
        # Save the current parameters (for use with SAVE_CONFIG)
        configfile.set('printer', 'delta_radius', "%.6f" % (self.radius,))
        for i, axis in enumerate('abc'):
            configfile.set('stepper_'+axis, 'angle', "%.6f" % (self.angles[i],))
            configfile.set('stepper_'+axis, 'arm_length',
                           "%.6f" % (self.arms[i],))
            configfile.set('stepper_'+axis, 'position_endstop',
                           "%.6f" % (self.endstops[i],))
        gcode = configfile.get_printer().lookup_object("gcode")
        gcode.respond_info(
            "stepper_a: position_endstop: %.6f angle: %.6f arm_length: %.6f\n"
            "stepper_b: position_endstop: %.6f angle: %.6f arm_length: %.6f\n"
            "stepper_c: position_endstop: %.6f angle: %.6f arm_length: %.6f\n"
            "delta_radius: %.6f"
            % (self.endstops[0], self.angles[0], self.arms[0],
               self.endstops[1], self.angles[1], self.arms[1],
               self.endstops[2], self.angles[2], self.arms[2],
               self.radius))

def load_kinematics(toolhead, config):
    return DeltaKinematics(toolhead, config)
