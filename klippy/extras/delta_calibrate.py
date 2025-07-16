# Delta calibration support - Enhanced version
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

# Constants
MEASURE_WEIGHT = 0.5
SMALL_Z_OFFSET_THRESHOLD = 0.5
MAX_CONFIG_ENTRIES = 999
DEFAULT_SCATTER = [.95, .90, .85, .70, .75, .80]

# Calibration object measurements from docs/prints/calibrate_size.stl
MEASURE_ANGLES = [210., 270., 330., 30., 90., 150.]
MEASURE_OUTER_RADIUS = 65
MEASURE_RIDGE_RADIUS = 5. - .5

# Optimization parameters
OPTIMIZATION_CONFIG = {
    'lbfgs': {
        'method': 'L-BFGS-B',
        'options': {'maxiter': 3000, 'ftol': 1e-10, 'gtol': 1e-8}
    },
    'nelder_mead': {
        'method': 'Nelder-Mead',
        'options': {'maxiter': 20000, 'xatol': 1e-6, 'fatol': 1e-8}
    }
}

# Parameter bounds for optimization
PARAMETER_BOUNDS = {
    'arm_': (50.0, None),
    'radius': (50.0, None),
    'lean': (-5.0, 5.0),
    'default': (None, None)
}

def load_config_stable(config, option):
    """Load a stable position from a config entry"""
    return config.getfloatlist(option, count=3)

def get_parameter_bounds(param_name):
    """Get optimization bounds for a parameter"""
    for key, bounds in PARAMETER_BOUNDS.items():
        if key in param_name:
            return bounds
    return PARAMETER_BOUNDS['default']

def measurements_to_distances(measured_params, delta_params):
    """Convert distance measurements to calibration format"""
    mp = measured_params
    dp = delta_params
    scale = mp['SCALE'][0]
    
    # Process center measurements
    cpw = mp['CENTER_PILLAR_WIDTHS']
    center_widths = [cpw[0], cpw[2], cpw[1], cpw[0], cpw[2], cpw[1]]
    center_dists = [od - cw for od, cw in zip(mp['CENTER_DISTS'], center_widths)]
    
    # Process outer measurements
    outer_dists = [od - opw for od, opw in zip(mp['OUTER_DISTS'], mp['OUTER_PILLAR_WIDTHS'])]
    
    # Convert angles to XY multipliers
    obj_angles = list(map(math.radians, MEASURE_ANGLES))
    xy_angles = list(zip(map(math.cos, obj_angles), map(math.sin, obj_angles)))
    
    # Calculate positions
    inner_ridge = MEASURE_RIDGE_RADIUS * scale
    outer_ridge = (MEASURE_OUTER_RADIUS + MEASURE_RIDGE_RADIUS) * scale
    
    inner_pos = [(ax * inner_ridge, ay * inner_ridge, 0.) for ax, ay in xy_angles]
    outer_pos = [(ax * outer_ridge, ay * outer_ridge, 0.) for ax, ay in xy_angles]
    
    # Center positions
    center_positions = [
        (cd, dp.calc_stable_position(ip), dp.calc_stable_position(op))
        for cd, ip, op in zip(center_dists, inner_pos, outer_pos)
    ]
    
    # Outer positions
    outer_center = MEASURE_OUTER_RADIUS * scale
    start_pos = [(ax * outer_center, ay * outer_center) for ax, ay in xy_angles]
    shifted_angles = xy_angles[2:] + xy_angles[:2]
    
    first_pos = [(ax * inner_ridge + spx, ay * inner_ridge + spy, 0.)
                 for (ax, ay), (spx, spy) in zip(shifted_angles, start_pos)]
    second_pos = [(ax * outer_ridge + spx, ay * outer_ridge + spy, 0.)
                  for (ax, ay), (spx, spy) in zip(shifted_angles, start_pos)]
    
    outer_positions = [
        (od, dp.calc_stable_position(fp), dp.calc_stable_position(sp))
        for od, fp, sp in zip(outer_dists, first_pos, second_pos)
    ]
    
    return center_positions + outer_positions

class OptimizationResult:
    """Container for optimization results"""
    def __init__(self, success=False, params=None, error=None, method=None, message=None):
        self.success = success
        self.params = params
        self.error = error
        self.method = method
        self.message = message

class DeltaCalibrate:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.gcode = self.printer.lookup_object('gcode')
        
        # Initialize probe helper
        self._setup_probe_helper(config)
        
        # Initialize data storage
        self._initialize_data_storage(config)
        
        # Register commands
        self._register_commands()
        
        # Configuration options
        self.tilt_correction_enabled = config.getboolean('tilt_correction', True)
        self.verbose_logging = config.getboolean('verbose_logging', False)
        self.max_optimization_iterations = config.getint('max_optimization_iterations', 3000)
        
    def _setup_probe_helper(self, config):
        """Setup probe helper with default points"""
        radius = config.getfloat('radius', above=0.)
        points = [(0., 0.)]
        scatter = config.getfloatlist('scatter', DEFAULT_SCATTER, count=6)
        
        for i in range(6):
            r = math.radians(90. + 60. * i)
            dist = radius * scatter[i]
            points.append((math.cos(r) * dist, math.sin(r) * dist))
        
        self.probe_helper = probe.ProbePointsHelper(
            config, self.probe_finalize, default_points=points)
        self.probe_helper.minimum_points(3)
        
        # Multi-Z probing configuration
        self.probe_z_heights = config.getfloatlist('probe_z_heights', None)
        if self.probe_z_heights and len(self.probe_z_heights) > 1:
            logging.info("Multi-Z height probing configured. Using first height: %.3f" % 
                        self.probe_z_heights[0])
    
    def _initialize_data_storage(self, config):
        """Initialize data storage from config"""
        self.last_probe_positions = self._load_positions(config, "height")
        self.manual_heights = self._load_positions(config, "manual_height")
        self.last_distances = self._load_distances(config)
        self.delta_analyze_entry = {'SCALE': [1.0]}
        
    def _load_positions(self, config, prefix):
        """Load height positions from config"""
        positions = []
        for i in range(MAX_CONFIG_ENTRIES):
            height = config.getfloat(f"{prefix}{i}", None)
            if height is None:
                break
            pos = load_config_stable(config, f"{prefix}{i}_pos")
            positions.append((height, pos))
        return positions
    
    def _load_distances(self, config):
        """Load distance measurements from config"""
        distances = []
        for i in range(MAX_CONFIG_ENTRIES):
            dist = config.getfloat(f"distance{i}", None)
            if dist is None:
                break
            pos1 = load_config_stable(config, f"distance{i}_pos1")
            pos2 = load_config_stable(config, f"distance{i}_pos2")
            distances.append((dist, pos1, pos2))
        return distances
    
    def _register_commands(self):
        """Register G-code commands"""
        self.gcode.register_command('DELTA_CALIBRATE', self.cmd_DELTA_CALIBRATE,
                                   desc=self.cmd_DELTA_CALIBRATE_help)
        self.gcode.register_command('DELTA_ANALYZE', self.cmd_DELTA_ANALYZE,
                                   desc=self.cmd_DELTA_ANALYZE_help)
        self.gcode.register_command('DELTA_CALIBRATE_STATUS', self.cmd_DELTA_CALIBRATE_STATUS,
                                   desc="Show delta calibration status")
        
    def handle_connect(self):
        """Verify this is a delta printer"""
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        if not hasattr(kin, "get_calibration"):
            raise self.printer.config_error("Delta calibrate is only for delta printers")

    def save_state(self, probe_positions, distances, delta_params):
        """Save calibration state to config"""
        configfile = self.printer.lookup_object('configfile')
        delta_params.save_state(configfile)
        
        section = 'delta_calibrate'
        configfile.remove_section(section)
        
        # Save probe positions
        self._save_positions(configfile, section, probe_positions, "height")
        
        # Save manual heights
        self._save_positions(configfile, section, self.manual_heights, "manual_height")
        
        # Save distances
        for i, (dist, pos1, pos2) in enumerate(distances):
            configfile.set(section, f"distance{i}", dist)
            configfile.set(section, f"distance{i}_pos1", "%.3f,%.3f,%.3f" % tuple(pos1))
            configfile.set(section, f"distance{i}_pos2", "%.3f,%.3f,%.3f" % tuple(pos2))
    
    def _save_positions(self, configfile, section, positions, prefix):
        """Save position data to config"""
        for i, (z_offset, spos) in enumerate(positions):
            configfile.set(section, f"{prefix}{i}", z_offset)
            configfile.set(section, f"{prefix}{i}_pos", "%.3f,%.3f,%.3f" % tuple(spos))

    def _apply_tilt_correction(self, positions, offsets):
        """Apply tilt correction to probe positions"""
        if not self.tilt_correction_enabled:
            return [(px, py, pz) for px, py, pz in positions]
        
        probe_x_offset, probe_y_offset, probe_z_offset = offsets
        toolhead = self.printer.lookup_object('toolhead')
        kin = toolhead.get_kinematics()
        
        corrected_positions = []
        logging.info("Applying tilt correction to %d probe points" % len(positions))
        
        for i, (px_raw, py_raw, pz_bed) in enumerate(positions):
            # Calculate nozzle position at probe trigger
            noz_x = px_raw - probe_x_offset
            noz_y = py_raw - probe_y_offset
            noz_z = pz_bed + probe_z_offset
            
            # Get effector normal for tilt compensation
            try:
                nx, ny, nz = kin.get_effector_normal(noz_x, noz_y, noz_z)
            except AttributeError:
                nx, ny, nz = 0.0, 0.0, 1.0
                if self.verbose_logging:
                    logging.debug("get_effector_normal not available, using vertical normal")
            
            # Apply correction if conditions are met
            if abs(probe_z_offset) >= SMALL_Z_OFFSET_THRESHOLD and abs(nz) > 1e-6:
                physical_extension = -probe_z_offset
                x_displacement = physical_extension * (nx / nz)
                y_displacement = physical_extension * (ny / nz)
                corrected_px = px_raw + x_displacement
                corrected_py = py_raw + y_displacement
                
                if self.verbose_logging:
                    logging.debug("Point %d: Tilt correction applied (%.3f,%.3f) -> (%.3f,%.3f)" %
                                (i, px_raw, py_raw, corrected_px, corrected_py))
            else:
                corrected_px = px_raw
                corrected_py = py_raw
                if self.verbose_logging:
                    logging.debug("Point %d: No tilt correction applied" % i)
            
            corrected_positions.append((corrected_px, corrected_py, pz_bed))
        
        return corrected_positions

    def probe_finalize(self, offsets, positions):
        """Finalize probe measurements and start calibration"""
        if not positions:
            self.gcode.respond_info("No valid probe positions found")
            return
        
        # Apply tilt correction
        corrected_positions = self._apply_tilt_correction(positions, offsets)
        
        # Convert to stable positions
        toolhead = self.printer.lookup_object('toolhead')
        kin = toolhead.get_kinematics()
        delta_params = kin.get_calibration()
        
        probe_data = []
        for cpx, cpy, cpz in corrected_positions:
            stable_pos = delta_params.calc_stable_position((cpx, cpy, cpz))
            probe_data.append((cpz, stable_pos))
        
        # Store probe positions for later use
        self.last_probe_positions = probe_data
        
        # Perform calibration
        self.calculate_params(probe_data, self.last_distances)

    def _create_error_function(self, orig_delta_params, height_positions, distances, z_weight):
        """Create error function for optimization"""
        def delta_errorfunc(params_dict):
            try:
                temp_params = orig_delta_params.new_calibration(params_dict)
                getpos = temp_params.get_position_from_stable
                total_error = 0.0
                
                # Height error
                if height_positions:
                    height_error = sum((getpos(spos)[2] - z_offset) ** 2 
                                     for z_offset, spos in height_positions)
                    total_error += height_error * z_weight
                
                # Distance error
                if distances:
                    distance_error = 0.0
                    for dist, spos1, spos2 in distances:
                        x1, y1, z1 = getpos(spos1)
                        x2, y2, z2 = getpos(spos2)
                        calc_dist = math.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)
                        distance_error += (calc_dist - dist) ** 2
                    total_error += distance_error
                
                return total_error
            except Exception as e:
                if self.verbose_logging:
                    logging.debug("Error in optimization function: %s" % str(e))
                return float('inf')
        
        return delta_errorfunc

    def _optimize_scipy(self, error_func, adj_params, initial_params):
        """Perform SciPy optimization"""
        initial_values = [initial_params[key] for key in adj_params]
        bounds = [get_parameter_bounds(key) for key in adj_params]
        
        def objective(x_values):
            params = dict(zip(adj_params, x_values))
            full_params = dict(initial_params)
            full_params.update(params)
            return error_func(full_params)
        
        initial_error = objective(initial_values)
        logging.info("Initial optimization error: %.6e" % initial_error)
        
        # Try L-BFGS-B first
        try:
            config = OPTIMIZATION_CONFIG['lbfgs'].copy()
            config['options']['maxiter'] = self.max_optimization_iterations
            
            result = scipy.optimize.minimize(objective, initial_values, bounds=bounds, **config)
            
            if result.success:
                return OptimizationResult(True, result.x, result.fun, 'L-BFGS-B', result.message)
            
            # Fallback to Nelder-Mead
            logging.warning("L-BFGS-B failed: %s. Trying Nelder-Mead..." % result.message)
            config = OPTIMIZATION_CONFIG['nelder_mead'].copy()
            config['options']['maxiter'] = self.max_optimization_iterations
            
            result = scipy.optimize.minimize(objective, initial_values, **config)
            
            if result.success:
                return OptimizationResult(True, result.x, result.fun, 'Nelder-Mead', result.message)
            
        except Exception as e:
            logging.error("SciPy optimization failed: %s" % str(e))
        
        return OptimizationResult(False, initial_values, initial_error, 'Failed', "All methods failed")

    def calculate_params(self, probe_positions, distances):
        """Calculate optimized delta parameters"""
        height_positions = self.manual_heights + probe_positions
        
        if not height_positions:
            self.gcode.respond_info("No probe positions available for calibration")
            return
        
        # Get calibration setup
        toolhead = self.printer.lookup_object('toolhead')
        kin = toolhead.get_kinematics()
        orig_delta_params = kin.get_calibration()
        
        try:
            adj_params, params = orig_delta_params.coordinate_descent_params(is_extended=True)
        except TypeError:
            adj_params, params = orig_delta_params.coordinate_descent_params()
        
        # Calculate weights
        z_weight = 1.0
        if distances and probe_positions:
            z_weight = len(distances) / (MEASURE_WEIGHT * len(probe_positions))
        elif distances and not probe_positions:
            z_weight = 0.0
        
        logging.info("Calibrating with %d height points, %d distance points (z_weight=%.3f)" %
                    (len(height_positions), len(distances), z_weight))
        
        # Create error function
        error_func = self._create_error_function(orig_delta_params, height_positions, distances, z_weight)
        
        # Optimize parameters
        if SCIPY_AVAILABLE:
            result = self._optimize_scipy(error_func, adj_params, params)
            if result.success:
                logging.info("%s optimization successful. Final error: %.6e" % 
                           (result.method, result.error))
                new_params_dict = dict(zip(adj_params, result.params))
            else:
                logging.error("SciPy optimization failed. Using coordinate descent fallback.")
                new_params_dict = mathutil.background_coordinate_descent(
                    self.printer, adj_params, params, error_func)
        else:
            logging.info("Using coordinate descent optimization")
            new_params_dict = mathutil.background_coordinate_descent(
                self.printer, adj_params, params, error_func)
        
        # Apply new parameters
        new_params = dict(params)
        new_params.update(new_params_dict)
        new_delta_params = orig_delta_params.new_calibration(new_params)
        
        # Log improvements
        self._log_calibration_results(height_positions, distances, orig_delta_params, new_delta_params)
        
        # Save results
        self.save_state(probe_positions, distances, new_delta_params)
        self.gcode.respond_info("Delta calibration complete. Use SAVE_CONFIG to store results.")

    def _log_calibration_results(self, height_positions, distances, orig_params, new_params):
        """Log calibration improvements"""
        logging.info("=== Calibration Results ===")
        
        # Height improvements
        if height_positions:
            logging.info("Height improvements:")
            for i, (z_offset, spos) in enumerate(height_positions):
                orig_z = orig_params.get_position_from_stable(spos)[2]
                new_z = new_params.get_position_from_stable(spos)[2]
                orig_error = abs(orig_z - z_offset)
                new_error = abs(new_z - z_offset)
                logging.info("  Point %d: %.6f -> %.6f (target: %.6f, error: %.6f -> %.6f)" %
                           (i, orig_z, new_z, z_offset, orig_error, new_error))
        
        # Distance improvements
        if distances:
            logging.info("Distance improvements:")
            for i, (target_dist, spos1, spos2) in enumerate(distances):
                # Original distance
                x1, y1, z1 = orig_params.get_position_from_stable(spos1)
                x2, y2, z2 = orig_params.get_position_from_stable(spos2)
                orig_dist = math.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)
                
                # New distance
                x1, y1, z1 = new_params.get_position_from_stable(spos1)
                x2, y2, z2 = new_params.get_position_from_stable(spos2)
                new_dist = math.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)
                
                orig_error = abs(orig_dist - target_dist)
                new_error = abs(new_dist - target_dist)
                logging.info("  Distance %d: %.6f -> %.6f (target: %.6f, error: %.6f -> %.6f)" %
                           (i, orig_dist, new_dist, target_dist, orig_error, new_error))

    def add_manual_height(self, height):
        """Add a manual height measurement at current position"""
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        kin = toolhead.get_kinematics()
        
        # Get current position
        kin_spos = {s.get_name(): s.get_commanded_position() for s in kin.get_steppers()}
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
        """Perform extended calibration with distance measurements"""
        # Get distance measurements
        if len(self.delta_analyze_entry) <= 1:
            distances = self.last_distances
        elif len(self.delta_analyze_entry) < 5:
            raise self.gcode.error("Not all measurements provided for extended calibration")
        else:
            kin = self.printer.lookup_object('toolhead').get_kinematics()
            delta_params = kin.get_calibration()
            distances = measurements_to_distances(self.delta_analyze_entry, delta_params)
        
        if not self.last_probe_positions:
            raise self.gcode.error("Must run DELTA_CALIBRATE first")
        
        self.calculate_params(self.last_probe_positions, distances)

    # Command handlers
    cmd_DELTA_CALIBRATE_help = "Delta calibration script"
    def cmd_DELTA_CALIBRATE(self, gcmd):
        """Handle DELTA_CALIBRATE command"""
        self.probe_helper.start_probe(gcmd)

    cmd_DELTA_ANALYZE_help = "Extended delta calibration tool"
    def cmd_DELTA_ANALYZE(self, gcmd):
        """Handle DELTA_ANALYZE command"""
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
        
        # Perform calibration
        action = gcmd.get('CALIBRATE', None)
        if action == 'extended':
            self.do_extended_calibration()
        elif action is not None:
            raise gcmd.error("Unknown calibrate action: %s" % action)

    def cmd_DELTA_CALIBRATE_STATUS(self, gcmd):
        """Show calibration status"""
        status = []
        status.append("=== Delta Calibration Status ===")
        status.append("Probe positions: %d" % len(self.last_probe_positions))
        status.append("Manual heights: %d" % len(self.manual_heights))
        status.append("Distance measurements: %d" % len(self.last_distances))
        status.append("Tilt correction: %s" % ("enabled" if self.tilt_correction_enabled else "disabled"))
        status.append("SciPy available: %s" % ("yes" if SCIPY_AVAILABLE else "no"))
        
        if self.delta_analyze_entry and len(self.delta_analyze_entry) > 1:
            status.append("Analyze measurements ready: %s" % list(self.delta_analyze_entry.keys()))
        
        self.gcode.respond_info("\n".join(status))

def load_config(config):
    return DeltaCalibrate(config)
