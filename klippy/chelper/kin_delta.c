// Delta kinematics stepper pulse time generation - Improved Version
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord
#include "pyhelper.h" // errorf

// Constants for numerical robustness
#define SQRT_EPSILON 1e-9        // Small epsilon for sqrt robustness
#define POSITION_EPSILON 1e-6    // Epsilon for position calculations
#define MIN_ARM_LENGTH 1e-3      // Minimum valid arm length
#define MAX_ARM_LENGTH 1e3       // Maximum valid arm length

// Error codes for better error handling
typedef enum {
    DELTA_OK = 0,
    DELTA_ERROR_INVALID_PARAMS,
    DELTA_ERROR_UNREACHABLE,
    DELTA_ERROR_NUMERICAL
} delta_error_t;

struct delta_stepper {
    struct stepper_kinematics sk;
    double arm2;                 // Square of arm length
    double arm_length;           // Arm length (for validation)
    double base_tower_x, base_tower_y; // Tower base XY at Z=0
    double lean_dx_dh;          // X offset factor due to lean = dx/dh_rail
    double lean_dy_dh;          // Y offset factor due to lean = dy/dh_rail
    double lean_dz_dh;          // Z projection factor = dz_vertical/dh_rail
    uint8_t has_lean;           // Flag to optimize no-lean case
    
    // Cached values for performance
    double lean_magnitude_sq;   // lean_dx_dh^2 + lean_dy_dh^2 + lean_dz_dh^2
};

// Validate coordinate inputs
static inline int
validate_coord(struct coord c)
{
    return !(isnan(c.x) || isnan(c.y) || isnan(c.z) || 
             isinf(c.x) || isinf(c.y) || isinf(c.z));
}

// Clamp small negative values to zero for numerical stability
static inline double
clamp_sqrt_arg(double val)
{
    return (val < 0. && val > -SQRT_EPSILON) ? 0. : val;
}

// Fast path for no-lean case (most common scenario)
static double
delta_calc_position_no_lean(struct delta_stepper *ds, struct coord c)
{
    double dx = ds->base_tower_x - c.x;
    double dy = ds->base_tower_y - c.y;
    double radial_dist_sq = dx*dx + dy*dy;
    
    // Check if point is reachable
    if (radial_dist_sq > ds->arm2) {
        return NAN; // Point is unreachable
    }
    
    double arm_projection_sq = ds->arm2 - radial_dist_sq;
    arm_projection_sq = clamp_sqrt_arg(arm_projection_sq);
    
    return sqrt(arm_projection_sq) + c.z;
}

// Solve quadratic equation with numerical stability
static int
solve_quadratic(double a, double b, double c, double *x1, double *x2)
{
    if (fabs(a) < POSITION_EPSILON) {
        // Linear equation: bx + c = 0
        if (fabs(b) < POSITION_EPSILON) {
            return 0; // No solution or infinite solutions
        }
        *x1 = *x2 = -c / b;
        return 1;
    }
    
    double discriminant = b*b - 4.*a*c;
    
    if (discriminant < -SQRT_EPSILON) {
        return 0; // No real solutions
    }
    
    discriminant = clamp_sqrt_arg(discriminant);
    double sqrt_d = sqrt(discriminant);
    
    // Use numerically stable formula to avoid loss of precision
    double q = -0.5 * (b + (b >= 0 ? sqrt_d : -sqrt_d));
    *x1 = q / a;
    *x2 = c / q;
    
    // Sort solutions so x1 >= x2
    if (*x1 < *x2) {
        double temp = *x1;
        *x1 = *x2;
        *x2 = temp;
    }
    
    return 2;
}

// Lean-aware calculation with improved numerics
static double
delta_calc_position_with_lean(struct delta_stepper *ds, struct coord c)
{
    // Transform to local coordinate system
    double dx0 = c.x - ds->base_tower_x;
    double dy0 = c.y - ds->base_tower_y;
    
    // Quadratic equation coefficients: A*h^2 + B*h + C = 0
    // A = lean_magnitude_sq (should be 1 for unit vector, but we calculate it)
    double A = ds->lean_magnitude_sq;
    double B = -2. * (dx0 * ds->lean_dx_dh + dy0 * ds->lean_dy_dh + c.z * ds->lean_dz_dh);
    double C = dx0*dx0 + dy0*dy0 + c.z*c.z - ds->arm2;
    
    double h1, h2;
    int num_solutions = solve_quadratic(A, B, C, &h1, &h2);
    
    if (num_solutions == 0) {
        return NAN; // No valid solution
    }
    
    // Choose the physically correct solution
    // We want h > 0 (positive distance along rail)
    // and the solution that places the carriage above the nozzle
    if (h1 >= -POSITION_EPSILON) {
        return (h1 < 0.) ? 0. : h1;
    }
    
    if (num_solutions == 2 && h2 >= -POSITION_EPSILON) {
        return (h2 < 0.) ? 0. : h2;
    }
    
    return NAN; // No valid positive solution
}

// Main position calculation function
static double
delta_stepper_calc_position(struct stepper_kinematics *sk, struct move *m,
                            double move_time)
{
    struct delta_stepper *ds = container_of(sk, struct delta_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    
    // Validate inputs
    if (!validate_coord(c)) {
        return NAN;
    }
    
    // Use optimized path for no-lean case
    if (!ds->has_lean) {
        return delta_calc_position_no_lean(ds, c);
    }
    
    return delta_calc_position_with_lean(ds, c);
}

// Validate delta stepper parameters
static delta_error_t
validate_delta_params(double arm2, double base_tower_x, double base_tower_y,
                      double lean_dx_dh, double lean_dy_dh, double lean_dz_dh)
{
    // Check for NaN values
    if (isnan(arm2) || isnan(base_tower_x) || isnan(base_tower_y) || 
        isnan(lean_dx_dh) || isnan(lean_dy_dh) || isnan(lean_dz_dh)) {
        return DELTA_ERROR_INVALID_PARAMS;
    }
    
    // Check for infinite values
    if (isinf(arm2) || isinf(base_tower_x) || isinf(base_tower_y) || 
        isinf(lean_dx_dh) || isinf(lean_dy_dh) || isinf(lean_dz_dh)) {
        return DELTA_ERROR_INVALID_PARAMS;
    }
    
    // Validate arm length
    double arm_length = sqrt(arm2);
    if (arm_length < MIN_ARM_LENGTH || arm_length > MAX_ARM_LENGTH) {
        return DELTA_ERROR_INVALID_PARAMS;
    }
    
    // Validate lean parameters form a reasonable unit vector
    double lean_magnitude = sqrt(lean_dx_dh*lean_dx_dh + lean_dy_dh*lean_dy_dh + lean_dz_dh*lean_dz_dh);
    if (lean_magnitude < 0.9 || lean_magnitude > 1.1) {
        return DELTA_ERROR_INVALID_PARAMS;
    }
    
    return DELTA_OK;
}

// Public API function with improved error handling
struct stepper_kinematics * __visible
delta_stepper_alloc(double arm2, double base_tower_x, double base_tower_y,
                    double lean_dx_dh, double lean_dy_dh, double lean_dz_dh)
{
    // Validate parameters first
    delta_error_t error = validate_delta_params(arm2, base_tower_x, base_tower_y,
                                               lean_dx_dh, lean_dy_dh, lean_dz_dh);
    if (error != DELTA_OK) {
        errorf("Delta kinematics: Invalid parameters (error %d)", error);
        return NULL;
    }
    
    struct delta_stepper *ds = malloc(sizeof(*ds));
    if (!ds) {
        errorf("Delta kinematics: Memory allocation failed");
        return NULL;
    }
    
    memset(ds, 0, sizeof(*ds));
    
    // Initialize parameters
    ds->arm2 = arm2;
    ds->arm_length = sqrt(arm2);
    ds->base_tower_x = base_tower_x;
    ds->base_tower_y = base_tower_y;
    ds->lean_dx_dh = lean_dx_dh;
    ds->lean_dy_dh = lean_dy_dh;
    ds->lean_dz_dh = lean_dz_dh;
    
    // Optimize for no-lean case
    ds->has_lean = (fabs(lean_dx_dh) > POSITION_EPSILON || 
                    fabs(lean_dy_dh) > POSITION_EPSILON || 
                    fabs(lean_dz_dh - 1.0) > POSITION_EPSILON);
    
    // Cache lean magnitude for performance
    ds->lean_magnitude_sq = lean_dx_dh*lean_dx_dh + lean_dy_dh*lean_dy_dh + lean_dz_dh*lean_dz_dh;
    
    // Initialize stepper kinematics structure
    ds->sk.calc_position_cb = delta_stepper_calc_position;
    ds->sk.active_flags = AF_X | AF_Y | AF_Z;
    
    return &ds->sk;
}

// Additional utility function for debugging/validation
void __visible
delta_stepper_get_info(struct stepper_kinematics *sk, double *arm_length,
                       double *tower_x, double *tower_y, int *has_lean)
{
    if (!sk) return;
    
    struct delta_stepper *ds = container_of(sk, struct delta_stepper, sk);
    
    if (arm_length) *arm_length = ds->arm_length;
    if (tower_x) *tower_x = ds->base_tower_x;
    if (tower_y) *tower_y = ds->base_tower_y;
    if (has_lean) *has_lean = ds->has_lean;
}
