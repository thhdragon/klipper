# Delta calibration support
#
# Copyright (C) 2017-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging, collections
import mathutil
from . import probe

try:
    import scipy.optimize
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    logging.info("SciPy library not found, advanced delta calibration optimizer will be disabled.")

# A "stable position" is a 3-tuple containing the number of steps
# taken since hitting the endstop on each delta tower.  Delta
# calibration uses this coordinate system because it allows a position
# to be described independent of the software parameters.

# Load a stable position from a config entry
def load_config_stable(config, option):
    return config.getfloatlist(option, count=3)


######################################################################
# Delta calibration object
######################################################################

# The angles and distances of the calibration object found in
# docs/prints/calibrate_size.stl
MeasureAngles = [210., 270., 330., 30., 90., 150.]
MeasureOuterRadius = 65
MeasureRidgeRadius = 5. - .5

# How much to prefer a distance measurement over a height measurement
MEASURE_WEIGHT = 0.5

# Convert distance measurements made on the calibration object to
# 3-tuples of (actual_distance, stable_position1, stable_position2)
def measurements_to_distances(measured_params, delta_params):
    # Extract params
    mp = measured_params
    dp = delta_params
    scale = mp['SCALE'][0]
    cpw = mp['CENTER_PILLAR_WIDTHS']
    center_widths = [cpw[0], cpw[2], cpw[1], cpw[0], cpw[2], cpw[1]]
    center_dists = [od - cw
                    for od, cw in zip(mp['CENTER_DISTS'], center_widths)]
    outer_dists = [
        od - opw
        for od, opw in zip(mp['OUTER_DISTS'], mp['OUTER_PILLAR_WIDTHS']) ]
    # Convert angles in degrees to an XY multiplier
    obj_angles = list(map(math.radians, MeasureAngles))
    xy_angles = list(zip(map(math.cos, obj_angles), map(math.sin, obj_angles)))
    # Calculate stable positions for center measurements
    inner_ridge = MeasureRidgeRadius * scale
    inner_pos = [(ax * inner_ridge, ay * inner_ridge, 0.)
                 for ax, ay in xy_angles]
    outer_ridge = (MeasureOuterRadius + MeasureRidgeRadius) * scale
    outer_pos = [(ax * outer_ridge, ay * outer_ridge, 0.)
                 for ax, ay in xy_angles]
    center_positions = [
        (cd, dp.calc_stable_position(ip), dp.calc_stable_position(op))
        for cd, ip, op in zip(center_dists, inner_pos, outer_pos)]
    # Calculate positions of outer measurements
    outer_center = MeasureOuterRadius * scale
    start_pos = [(ax * outer_center, ay * outer_center) for ax, ay in xy_angles]
    shifted_angles = xy_angles[2:] + xy_angles[:2]
    first_pos = [(ax * inner_ridge + spx, ay * inner_ridge + spy, 0.)
                 for (ax, ay), (spx, spy) in zip(shifted_angles, start_pos)]
    second_pos = [(ax * outer_ridge + spx, ay * outer_ridge + spy, 0.)
                  for (ax, ay), (spx, spy) in zip(shifted_angles, start_pos)]
    outer_positions = [
        (od, dp.calc_stable_position(fp), dp.calc_stable_position(sp))
        for od, fp, sp in zip(outer_dists, first_pos, second_pos)]
    return center_positions + outer_positions


######################################################################
# Delta Calibrate class
######################################################################

class DeltaCalibrate:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
        # Calculate default probing points
        radius = config.getfloat('radius', above=0.)
        points = [(0., 0.)]
        scatter = [.95, .90, .85, .70, .75, .80]
        for i in range(6):
            r = math.radians(90. + 60. * i)
            dist = radius * scatter[i]
            points.append((math.cos(r) * dist, math.sin(r) * dist))
        self.probe_helper = probe.ProbePointsHelper(
            config, self.probe_finalize, default_points=points)
        self.probe_helper.minimum_points(3)

        # Multi-Z probing heights - simplified implementation
        self.probe_z_heights = config.getfloatlist('probe_z_heights', None)
        if self.probe_z_heights is not None and len(self.probe_z_heights) > 1:
            logging.info("Multi-Z height probing configured for DELTA_CALIBRATE. "
                         "Using first height: %.3f" % self.probe_z_heights[0])

        # Restore probe stable positions
        self.last_probe_positions = []
        for i in range(999):
            height = config.getfloat("height%d" % (i,), None)
            if height is None:
                break
            height_pos = load_config_stable(config, "height%d_pos" % (i,))
            self.last_probe_positions.append((height, height_pos))
        
        # Restore manually entered heights
        self.manual_heights = []
        for i in range(999):
            height = config.getfloat("manual_height%d" % (i,), None)
            if height is None:
                break
            height_pos = load_config_stable(config, "manual_height%d_pos" % (i,))
            self.manual_heights.append((height, height_pos))
        
        # Restore distance measurements
        self.delta_analyze_entry = {'SCALE': [1.0]}  # Fixed: should be list, not tuple
        self.last_distances = []
        for i in range(999):
            dist = config.getfloat("distance%d" % (i,), None)
            if dist is None:
                break
            distance_pos1 = load_config_stable(config, "distance%d_pos1" % (i,))
            distance_pos2 = load_config_stable(config, "distance%d_pos2" % (i,))
            self.last_distances.append((dist, distance_pos1, distance_pos2))
        
        # Register gcode commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('DELTA_CALIBRATE', self.cmd_DELTA_CALIBRATE,
                                    desc=self.cmd_DELTA_CALIBRATE_help)
        self.gcode.register_command('DELTA_ANALYZE', self.cmd_DELTA_ANALYZE,
                                    desc=self.cmd_DELTA_ANALYZE_help)

    def handle_connect(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        if not hasattr(kin, "get_calibration"):
            raise self.printer.config_error(
                "Delta calibrate is only for delta printers")

    def save_state(self, probe_positions, distances, delta_params):
        # Save main delta parameters
        configfile = self.printer.lookup_object('configfile')
        delta_params.save_state(configfile)
        # Save probe stable positions
        section = 'delta_calibrate'
        configfile.remove_section(section)
        for i, (z_offset, spos) in enumerate(probe_positions):
            configfile.set(section, "height%d" % (i,), z_offset)
            configfile.set(section, "height%d_pos" % (i,),
                           "%.3f,%.3f,%.3f" % tuple(spos))
        # Save manually entered heights
        for i, (z_offset, spos) in enumerate(self.manual_heights):
            configfile.set(section, "manual_height%d" % (i,), z_offset)
            configfile.set(section, "manual_height%d_pos" % (i,),
                           "%.3f,%.3f,%.3f" % tuple(spos))
        # Save distance measurements
        for i, (dist, spos1, spos2) in enumerate(distances):
            configfile.set(section, "distance%d" % (i,), dist)
            configfile.set(section, "distance%d_pos1" % (i,),
                           "%.3f,%.3f,%.3f" % tuple(spos1))
            configfile.set(section, "distance%d_pos2" % (i,),
                           "%.3f,%.3f,%.3f" % tuple(spos2))

    def probe_finalize(self, offsets, positions):
        # Convert positions into (z_offset, stable_position) pairs
        probe_x_offset, probe_y_offset, probe_z_offset_config = offsets
        toolhead = self.printer.lookup_object('toolhead')
        kin = toolhead.get_kinematics()
        delta_params = kin.get_calibration()

        corrected_probe_positions = []
        logging.info("Probe tilt compensation: Applying corrections...")

        for i, (px_uncorrected, py_uncorrected, pz_bed) in enumerate(positions):
            # Estimate nozzle XYZ at the moment of probe trigger
            noz_x_at_trigger = px_uncorrected - probe_x_offset
            noz_y_at_trigger = py_uncorrected - probe_y_offset
            noz_z_at_trigger = pz_bed + probe_z_offset_config

            # Get effector normal for tilt compensation
            try:
                nx, ny, nz = kin.get_effector_normal(noz_x_at_trigger, noz_y_at_trigger, noz_z_at_trigger)
            except AttributeError:
                # Fallback if get_effector_normal doesn't exist
                nx, ny, nz = 0.0, 0.0, 1.0
                logging.debug("get_effector_normal not available, using vertical normal")

            # Apply tilt correction
            apply_tilt_correction = True
            if abs(probe_z_offset_config) < 0.5:  # Small Z offset threshold
                apply_tilt_correction = False
                logging.debug("Point %d: Small probe Z offset, skipping tilt correction" % i)

            if apply_tilt_correction and abs(nz) > 1e-6:
                physical_probe_extension = -probe_z_offset_config
                x_displacement = physical_probe_extension * (nx / nz)
                y_displacement = physical_probe_extension * (ny / nz)
                corrected_px = px_uncorrected + x_displacement
                corrected_py = py_uncorrected + y_displacement
            else:
                corrected_px = px_uncorrected
                corrected_py = py_uncorrected

            logging.debug(
                "Point %d: Uncorrected(%.3f,%.3f,%.3f) -> Corrected(%.3f,%.3f,%.3f)" %
                (i, px_uncorrected, py_uncorrected, pz_bed, corrected_px, corrected_py, pz_bed))

            corrected_probe_positions.append((corrected_px, corrected_py, pz_bed))

        # Convert to stable positions for calibration
        final_probe_data = []
        for cpx, cpy, cpz_bed in corrected_probe_positions:
            stable_pos = delta_params.calc_stable_position((cpx, cpy, cpz_bed))
            final_probe_data.append((cpz_bed, stable_pos))

        # Perform analysis
        self.calculate_params(final_probe_data, self.last_distances)

    def calculate_params(self, probe_positions, distances):
        height_positions = self.manual_heights + probe_positions
        if not height_positions:
            self.gcode.respond_info("No probe positions available for calibration.")
            return

        # Setup for coordinate descent analysis
        toolhead = self.printer.lookup_object('toolhead')
        kin = toolhead.get_kinematics()
        orig_delta_params = kin.get_calibration()
        
        # Get calibration parameters
        try:
            adj_params, params = orig_delta_params.coordinate_descent_params(is_extended=True)
        except TypeError:
            # Fallback for older API
            adj_params, params = orig_delta_params.coordinate_descent_params()

        logging.info("Calculating delta calibration with %d height points and %d distance points." %
                     (len(height_positions), len(distances)))
        logging.info("Initial parameters: %s" % params)
        logging.info("Adjustable parameters: %s" % adj_params)

        # Weight calculation
        z_weight = 1.0
        if distances and probe_positions:
            z_weight = len(distances) / (MEASURE_WEIGHT * len(probe_positions))
        elif distances and not probe_positions:
            z_weight = 0.0

        def delta_errorfunc(current_params_dict):
            try:
                temp_delta_params = orig_delta_params.new_calibration(current_params_dict)
                getpos = temp_delta_params.get_position_from_stable
                total_error = 0.0

                # Height error calculation
                if height_positions:
                    height_error = 0.0
                    for z_offset, stable_pos in height_positions:
                        x, y, z = getpos(stable_pos)
                        height_error += (z - z_offset) ** 2
                    total_error += height_error * z_weight

                # Distance error calculation
                if distances:
                    distance_error = 0.0
                    for dist, stable_pos1, stable_pos2 in distances:
                        x1, y1, z1 = getpos(stable_pos1)
                        x2, y2, z2 = getpos(stable_pos2)
                        calculated_dist = math.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)
                        distance_error += (calculated_dist - dist) ** 2
                    total_error += distance_error

                return total_error
            except Exception as e:
                logging.debug("Error in delta_errorfunc: %s" % str(e))
                return float('inf')

        # Optimization
        if SCIPY_AVAILABLE:
            initial_values = [params[key] for key in adj_params]
            
            # Set bounds for parameters
            bounds = []
            for key in adj_params:
                if 'arm_' in key:
                    bounds.append((50.0, None))
                elif key == 'radius' and 'offset' not in key:
                    bounds.append((50.0, None))
                elif 'lean' in key:
                    bounds.append((-5.0, 5.0))
                else:
                    bounds.append((None, None))

            def objective_func(x_values):
                current_params = dict(zip(adj_params, x_values))
                full_params = dict(params)
                full_params.update(current_params)
                return delta_errorfunc(full_params)

            # Calculate initial error
            initial_error = objective_func(initial_values)
            logging.info("Initial error: %.6e" % initial_error)

            # Try L-BFGS-B first
            try:
                result = scipy.optimize.minimize(
                    objective_func, initial_values, 
                    method='L-BFGS-B', bounds=bounds,
                    options={'maxiter': 3000, 'ftol': 1e-10, 'gtol': 1e-8}
                )
                
                if result.success:
                    logging.info("L-BFGS-B optimization successful. Final error: %.6e" % result.fun)
                    final_values = result.x
                else:
                    logging.warning("L-BFGS-B failed: %s. Trying Nelder-Mead..." % result.message)
                    result = scipy.optimize.minimize(
                        objective_func, initial_values,
                        method='Nelder-Mead',
                        options={'maxiter': 20000, 'xatol': 1e-6, 'fatol': 1e-8}
                    )
                    if result.success:
                        logging.info("Nelder-Mead optimization successful. Final error: %.6e" % result.fun)
                        final_values = result.x
                    else:
                        logging.error("Both optimization methods failed. Using initial values.")
                        final_values = initial_values
            except Exception as e:
                logging.error("SciPy optimization failed: %s" % str(e))
                final_values = initial_values

            # Update parameters
            new_params_dict = dict(zip(adj_params, final_values))
            new_params = dict(params)
            new_params.update(new_params_dict)
        else:
            # Fallback to coordinate descent
            logging.info("Using coordinate descent optimization")
            new_params = mathutil.background_coordinate_descent(
                self.printer, adj_params, params, delta_errorfunc)

        # Apply new parameters and log results
        logging.info("Final parameters: %s" % new_params)
        new_delta_params = orig_delta_params.new_calibration(new_params)
        
        # Log height improvements
        for z_offset, spos in height_positions:
            orig_z = orig_delta_params.get_position_from_stable(spos)[2]
            new_z = new_delta_params.get_position_from_stable(spos)[2]
            logging.info("Height: original=%.6f new=%.6f target=%.6f" % (orig_z, new_z, z_offset))

        # Log distance improvements
        for dist, spos1, spos2 in distances:
            # Original distance
            x1, y1, z1 = orig_delta_params.get_position_from_stable(spos1)
            x2, y2, z2 = orig_delta_params.get_position_from_stable(spos2)
            orig_dist = math.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)
            # New distance
            x1, y1, z1 = new_delta_params.get_position_from_stable(spos1)
            x2, y2, z2 = new_delta_params.get_position_from_stable(spos2)
            new_dist = math.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)
            logging.info("Distance: original=%.6f new=%.6f target=%.6f" % (orig_dist, new_dist, dist))

        # Save results
        self.save_state(probe_positions, distances, new_delta_params)
        self.gcode.respond_info(
            "Delta calibration complete. Use SAVE_CONFIG to store results.")

    cmd_DELTA_CALIBRATE_help = "Delta calibration script"
    def cmd_DELTA_CALIBRATE(self, gcmd):
        self.probe_helper.start_probe(gcmd)

    def add_manual_height(self, height):
        # Get current toolhead position
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        kin = toolhead.get_kinematics()
        
        # Get stepper positions
        kin_spos = {s.get_name(): s.get_commanded_position()
                    for s in kin.get_steppers()}
        kin_pos = kin.calc_position(kin_spos)
        
        # Convert to stable position
        delta_params = kin.get_calibration()
        stable_pos = tuple(delta_params.calc_stable_position(kin_pos))
        
        # Add to manual heights
        self.manual_heights.append((height, stable_pos))
        self.gcode.respond_info(
            "Added manual height: position(%.3f,%.3f,%.3f) = z%.3f" %
            (kin_pos[0], kin_pos[1], kin_pos[2], height))

    def do_extended_calibration(self):
        # Get distance measurements
        if len(self.delta_analyze_entry) <= 1:
            distances = self.last_distances
        elif len(self.delta_analyze_entry) < 5:
            raise self.gcode.error("Not all measurements provided")
        else:
            kin = self.printer.lookup_object('toolhead').get_kinematics()
            delta_params = kin.get_calibration()
            distances = measurements_to_distances(self.delta_analyze_entry, delta_params)
        
        if not self.last_probe_positions:
            raise self.gcode.error("Must run DELTA_CALIBRATE first")
        
        # Perform extended calibration
        self.calculate_params(self.last_probe_positions, distances)

    cmd_DELTA_ANALYZE_help = "Extended delta calibration tool"
    def cmd_DELTA_ANALYZE(self, gcmd):
        # Handle manual height entry
        mheight = gcmd.get_float('MANUAL_HEIGHT', None)
        if mheight is not None:
            self.add_manual_height(mheight)
            return

        # Parse measurement parameters
        measurement_params = {
            'CENTER_DISTS': 6, 'CENTER_PILLAR_WIDTHS': 3,
            'OUTER_DISTS': 6, 'OUTER_PILLAR_WIDTHS': 6, 'SCALE': 1
        }
        
        for name, expected_count in measurement_params.items():
            data = gcmd.get(name, None)
            if data is None:
                continue
            try:
                values = [float(x.strip()) for x in data.split(',')]
            except (ValueError, AttributeError):
                raise gcmd.error("Unable to parse parameter '%s'" % name)
            
            if len(values) != expected_count:
                raise gcmd.error("Parameter '%s' must have %d values, got %d" %
                                 (name, expected_count, len(values)))
            
            self.delta_analyze_entry[name] = values
            logging.info("DELTA_ANALYZE %s = %s" % (name, values))

        # Perform calibration if requested
        action = gcmd.get('CALIBRATE', None)
        if action == 'extended':
            self.do_extended_calibration()
        elif action is not None:
            raise gcmd.error("Unknown calibrate action: %s" % action)

def load_config(config):
    return DeltaCalibrate(config)
