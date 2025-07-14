# Delta calibration support
#
# Copyright (C) 2017-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging, collections
import numpy as np
import mathutil
from . import probe

try:
    import scipy.optimize
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    logging.info("SciPy library not found, advanced delta calibration optimizer will be disabled if not installed.")

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
        self.config = config
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
        # Register calibration parameters
        config.getfloat('radius', None)
        config.getint('probe_count', None)
        self.probe_helper = probe.ProbePointsHelper(config, self.probe_finalize,
                                                    default_points=[])

        # Multi-Z probing heights
        self.probe_z_heights = config.getfloatlist('probe_z_heights', None)
        if self.probe_z_heights is None:
            # Default to using the horizontal_move_z from the [probe] section,
            # or a sensible default if that's not available.
            # This will be resolved during actual probing cmd.
            pass
        elif len(self.probe_z_heights) > 1:
            logging.info("Multi-Z height probing requested for DELTA_CALIBRATE. "
                         "Full implementation of probing at multiple Z levels is pending.")
            # For now, we'll just use the first Z height if multiple are given,
            # until the probing loop is refactored.

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
            height_pos = load_config_stable(config, "manual_height%d_pos"
                                            % (i,))
            self.manual_heights.append((height, height_pos))
        # Restore distance measurements
        self.delta_analyze_entry = {'SCALE': (1.,)}
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

        # Calculate default probing points
        print_radius = kin.get_print_radius()
        # Get probing radius from [delta_calibrate] or use 80% of print_radius
        radius = self.config.getfloat('radius', default=print_radius * 0.8,
                                 above=0.)

        probe_count = self.config.getint('probe_count', 7)
        # For now, we generate points in a pattern of 6 points per ring
        # plus a center point. This logic can be expanded later.
        # This implementation uses a fixed 7-point pattern for simplicity,
        # but is structured for future expansion.
        if probe_count < 4:
            raise self.config.error("delta_calibrate probe_count must be at least 4")

        points = [(0., 0.)]
        if probe_count > 1:
            # Generate 6 points on a circle of 'radius'
            # This is a common pattern for delta calibration
            num_outer_points = min(probe_count - 1, 6)
            for i in range(num_outer_points):
                r = math.radians(90. + (360. / num_outer_points) * i)
                dist = radius
                points.append((math.cos(r) * dist, math.sin(r) * dist))

        logging.info("Generated %d probe points for DELTA_CALIBRATE: %s",
                     len(points),
                     ", ".join(["(%.2f,%.2f)" % (x,y) for x,y in points]))
        self.probe_helper.update_default_points(points)
        self.probe_helper.minimum_points(4)
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
        # offsets = (probe_x_offset, probe_y_offset, probe_z_offset_from_config)
        # positions = list of (uncorrected_probe_tip_x, uncorrected_probe_tip_y, bed_z_at_that_xy)

        probe_x_offset, probe_y_offset, probe_z_offset_config = offsets
        toolhead = self.printer.lookup_object('toolhead')
        kin = toolhead.get_kinematics()
        delta_params = kin.get_calibration() # Current delta parameters

        corrected_probe_positions_for_stable_calc = []
        logging.info("Probe tilt compensation: Applying corrections...")

        for i, (px_uncorrected, py_uncorrected, pz_bed) in enumerate(positions):
            # Estimate nozzle XYZ at the moment of probe trigger
            noz_x_at_trigger = px_uncorrected - probe_x_offset
            noz_y_at_trigger = py_uncorrected - probe_y_offset
            noz_z_at_trigger = pz_bed + probe_z_offset_config

            nx, ny, nz = kin.get_effector_normal(noz_x_at_trigger, noz_y_at_trigger, noz_z_at_trigger)

            x_displacement, y_displacement = 0.0, 0.0 # Initialize here
            apply_tilt_xy_correction = True
            # If probe_z_offset_config is very small (e.g. manual probing), skip XY correction part of tilt.
            if abs(probe_z_offset_config) < 0.5: # Threshold for "negligible" Z offset, e.g., 0.5mm
                apply_tilt_xy_correction = False
                logging.debug(
                    "Point %d: Probe Z offset (%.3f) is small, XY tilt correction heuristically skipped."
                    % (i, probe_z_offset_config))

            if apply_tilt_xy_correction and abs(nz) < 1e-6:
                logging.warning("Probe tilt: Effector normal Z component is near zero (nz=%.3e) at point %d. Skipping XY tilt correction for this point." % (nz, i))
                apply_tilt_xy_correction = False

            if apply_tilt_xy_correction:
                physical_probe_extension = -probe_z_offset_config
                x_displacement = physical_probe_extension * (nx / nz)
                y_displacement = physical_probe_extension * (ny / nz)
                corrected_px = (noz_x_at_trigger + probe_x_offset) + x_displacement
                corrected_py = (noz_y_at_trigger + probe_y_offset) + y_displacement
            else:
                # No XY correction applied (either due to small z_offset or vertical effector normal)
                corrected_px = px_uncorrected
                corrected_py = py_uncorrected

            logging.debug(
                "Point %d: Nozzle(%.3f,%.3f,%.3f) UncorrectedProbe(%.3f,%.3f) Normal(%.3f,%.3f,%.3f) Disp(%.4f,%.4f) CorrectedProbe(%.3f,%.3f)" %
                (i, noz_x_at_trigger, noz_y_at_trigger, noz_z_at_trigger,
                    px_uncorrected, py_uncorrected, nx, ny, nz,
                    x_displacement, y_displacement, corrected_px, corrected_py))

            corrected_probe_positions_for_stable_calc.append( (corrected_px, corrected_py, pz_bed) )

        final_probe_data_for_calc = []
        for (cpx, cpy, cpz_bed) in corrected_probe_positions_for_stable_calc:
            stable_pos = delta_params.calc_stable_position( (cpx, cpy, cpz_bed) )
            final_probe_data_for_calc.append( (cpz_bed, stable_pos) )

        # Perform analysis
        self.calculate_params(final_probe_data_for_calc, self.last_distances)

    def calculate_params(self, probe_positions, distances):
        height_positions = self.manual_heights + probe_positions
        if not height_positions:
            self.gcode.respond_info("No probe positions available for calibration.")
            return

        # Setup for coordinate descent analysis
        toolhead = self.printer.lookup_object('toolhead')
        kin = toolhead.get_kinematics()
        orig_delta_params = odp = kin.get_calibration()
        adj_params, params = odp.coordinate_descent_params(is_extended=True)

        logging.info("Calculating delta_calibrate with %d height points and %d distance points."
                     % (len(height_positions), len(distances)))
        logging.info("Initial delta_calibrate parameters: %s" % (params,))
        logging.info("Parameters to be adjusted: %s" % (adj_params,))

        z_weight = 1.0
        if distances and probe_positions:
            z_weight = len(distances) / (MEASURE_WEIGHT * len(probe_positions))
        elif distances and not probe_positions:
            z_weight = 0.0

        def delta_errorfunc(current_iter_params_dict):
            total_error = 0.
            try:
                temp_delta_params = orig_delta_params.new_calibration(current_iter_params_dict)
                getpos = temp_delta_params.get_position_from_stable

                if height_positions:
                    height_error_sum = 0.
                    for z_offset, stable_pos in height_positions:
                        x, y, z = getpos(stable_pos)
                        height_error_sum += (z - z_offset)**2
                    total_error += height_error_sum * z_weight

                if distances:
                    distance_error_sum = 0.
                    for dist, stable_pos1, stable_pos2 in distances:
                        x1, y1, z1 = getpos(stable_pos1)
                        x2, y2, z2 = getpos(stable_pos2)
                        d = math.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)
                        distance_error_sum += (d - dist)**2
                    total_error += distance_error_sum

                if not np.isfinite(total_error):
                    logging.debug("Infinite error in delta_errorfunc with params %s", current_iter_params_dict)
                    return float('inf')
                return total_error
            except ValueError as e:
                logging.debug("Math error in delta_errorfunc: %s with params %s", str(e), current_iter_params_dict)
                return float('inf')
            except Exception as e:
                logging.exception("Unexpected error in delta_errorfunc with params %s", current_iter_params_dict)
                return float('inf')

        if SCIPY_AVAILABLE:
            initial_values = [params[key] for key in adj_params]
            bounds = []
            for key in adj_params:
                if 'arm_' in key: bounds.append((50.0, None))
                elif key == 'radius' and 'offset' not in key: bounds.append((50.0, None))
                elif 'lean' in key: bounds.append((-5.0, 5.0)) # Degrees
                else: bounds.append((None, None))

            def objective_func_for_scipy(x_values_array):
                current_iter_params_dict = dict(zip(adj_params, x_values_array))
                full_params_for_iter = dict(params)
                full_params_for_iter.update(current_iter_params_dict)
                return delta_errorfunc(full_params_for_iter)

            # Store initial error for comparison if SciPy fails badly
            # Note: delta_errorfunc expects a full dict.
            # Create a temporary full dict with initial values for this call.
            initial_full_params_dict = dict(params)
            for idx, key in enumerate(adj_params): # Ensure all adj_params are in initial_full_params_dict
                initial_full_params_dict[key] = initial_values[idx]
            initial_error_val = delta_errorfunc(initial_full_params_dict)
            logging.info("Initial error before SciPy: %.3e" % initial_error_val)


            logging.info("Attempting optimization with scipy.optimize.minimize (L-BFGS-B)...")
            try:
                opt_result = scipy.optimize.minimize(
                    objective_func_for_scipy, initial_values, method='L-BFGS-B', bounds=bounds,
                    options={'maxiter': 3000, 'disp': False, 'ftol': 1e-10, 'gtol': 1e-8, 'eps': 1e-9} )
            except (ValueError, np.linalg.LinAlgError) as e:
                logging.error("SciPy L-BFGS-B optimization threw a numerical exception: %s", str(e))
                opt_result = scipy.optimize.OptimizeResult({'success': False, 'message': str(e), 'fun': initial_error_val, 'x': initial_values, 'nit':0, 'nfev':0})
            except Exception as e:
                logging.error("SciPy L-BFGS-B optimization threw an unexpected exception: %s", str(e))
                opt_result = scipy.optimize.OptimizeResult({'success': False, 'message': str(e), 'fun': initial_error_val, 'x': initial_values, 'nit':0, 'nfev':0})

            if not opt_result.success:
                logging.warning("SciPy L-BFGS-B optimization failed or did not converge: %s. Trying Nelder-Mead...", opt_result.message)
                try:
                    opt_result = scipy.optimize.minimize(
                        objective_func_for_scipy, initial_values, method='Nelder-Mead',
                        options={'maxiter': 20000, 'disp': False, 'xatol': 1e-6, 'fatol': 1e-8} )
                except (ValueError, np.linalg.LinAlgError) as e:
                    logging.error("SciPy Nelder-Mead optimization threw a numerical exception: %s", str(e))
                    opt_result = scipy.optimize.OptimizeResult({'success': False, 'message': str(e), 'fun': initial_error_val, 'x': initial_values, 'nit':0, 'nfev':0})
                except Exception as e:
                    logging.error("SciPy Nelder-Mead optimization threw an unexpected exception: %s", str(e))
                    opt_result = scipy.optimize.OptimizeResult({'success': False, 'message': str(e), 'fun': initial_error_val, 'x': initial_values, 'nit':0, 'nfev':0})

            if opt_result.success and np.isfinite(opt_result.fun) and opt_result.fun < initial_error_val:
                logging.info("SciPy optimization successful (%s). Final error: %.3e, Iterations: %d, FuncEvals: %d",
                             opt_result.message, opt_result.fun, getattr(opt_result, 'nit', -1), getattr(opt_result, 'nfev', -1))
                final_param_values = opt_result.x
                new_params_dict = dict(zip(adj_params, final_param_values))
                final_params_for_klipper = dict(params)
                final_params_for_klipper.update(new_params_dict)
                new_params = final_params_for_klipper
            else:
                logging.error("All SciPy optimization attempts failed or did not improve upon initial error. Reverting to initial parameters for safety. Final result: %s", opt_result)
                new_params = dict(params)
        else:
            logging.warning("SciPy library not available. Using existing coordinate descent optimizer."
                            " This may be less effective for the full parameter set including tower leans.")
            new_params = mathutil.background_coordinate_descent(
                self.printer, adj_params, params, delta_errorfunc)

        # Log and report results
        logging.info("Final Calculated delta_calibrate parameters: %s", new_params)
        new_delta_params = orig_delta_params.new_calibration(new_params)
        for z_offset, spos in height_positions:
            logging.info("height orig: %.6f new: %.6f goal: %.6f",
                         orig_delta_params.get_position_from_stable(spos)[2],
                         new_delta_params.get_position_from_stable(spos)[2],
                         z_offset)
        for dist, spos1, spos2 in distances:
            x1, y1, z1 = orig_delta_params.get_position_from_stable(spos1)
            x2, y2, z2 = orig_delta_params.get_position_from_stable(spos2)
            orig_dist = math.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)
            x1, y1, z1 = new_delta_params.get_position_from_stable(spos1)
            x2, y2, z2 = new_delta_params.get_position_from_stable(spos2)
            new_dist = math.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)
            logging.info("distance orig: %.6f new: %.6f goal: %.6f",
                         orig_dist, new_dist, dist)
        # Store results for SAVE_CONFIG
        self.save_state(probe_positions, distances, new_delta_params)
        self.gcode.respond_info(
            "The SAVE_CONFIG command will update the printer config file\n"
            "with these parameters and restart the printer.")
    cmd_DELTA_CALIBRATE_help = "Delta calibration script"
    def cmd_DELTA_CALIBRATE(self, gcmd):
        self.probe_helper.start_probe(gcmd)
    def add_manual_height(self, height):
        # Determine current location of toolhead
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        kin = toolhead.get_kinematics()
        kin_spos = {s.get_name(): s.get_commanded_position()
                    for s in kin.get_steppers()}
        kin_pos = kin.calc_position(kin_spos)
        # Convert location to a stable position
        delta_params = kin.get_calibration()
        stable_pos = tuple(delta_params.calc_stable_position(kin_pos))
        # Add to list of manual heights
        self.manual_heights.append((height, stable_pos))
        self.gcode.respond_info(
            "Adding manual height: %.3f,%.3f,%.3f is actually z=%.3f"
            % (kin_pos[0], kin_pos[1], kin_pos[2], height))
    def do_extended_calibration(self):
        # Extract distance positions
        if len(self.delta_analyze_entry) <= 1:
            distances = self.last_distances
        elif len(self.delta_analyze_entry) < 5:
            raise self.gcode.error("Not all measurements provided")
        else:
            kin = self.printer.lookup_object('toolhead').get_kinematics()
            delta_params = kin.get_calibration()
            distances = measurements_to_distances(
                self.delta_analyze_entry, delta_params)
        if not self.last_probe_positions:
            raise self.gcode.error(
                "Must run basic calibration with DELTA_CALIBRATE first")
        # Perform analysis
        self.calculate_params(self.last_probe_positions, distances)
    cmd_DELTA_ANALYZE_help = "Extended delta calibration tool"
    def cmd_DELTA_ANALYZE(self, gcmd):
        # Check for manual height entry
        mheight = gcmd.get_float('MANUAL_HEIGHT', None)
        if mheight is not None:
            self.add_manual_height(mheight)
            return
        # Parse distance measurements
        args = {'CENTER_DISTS': 6, 'CENTER_PILLAR_WIDTHS': 3,
                'OUTER_DISTS': 6, 'OUTER_PILLAR_WIDTHS': 6, 'SCALE': 1}
        for name, count in args.items():
            data = gcmd.get(name, None)
            if data is None:
                continue
            try:
                parts = list(map(float, data.split(',')))
            except:
                raise gcmd.error("Unable to parse parameter '%s'" % (name,))
            if len(parts) != count:
                raise gcmd.error("Parameter '%s' must have %d values"
                                 % (name, count))
            self.delta_analyze_entry[name] = parts
            logging.info("DELTA_ANALYZE %s = %s", name, parts)
        # Perform analysis if requested
        action = gcmd.get('CALIBRATE', None)
        if action is not None:
            if action != 'extended':
                raise gcmd.error("Unknown calibrate action")
            self.do_extended_calibration()

def load_config(config):
    return DeltaCalibrate(config)
