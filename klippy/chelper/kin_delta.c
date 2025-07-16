// Delta kinematics stepper pulse time generation - Enhanced Version
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
// Enhanced version with improved performance and maintainability
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "compiler.h"
#include "itersolve.h"
#include "trapq.h"
#include "pyhelper.h"

// Enhanced constants for numerical robustness
#define SQRT_EPSILON 1e-12       // Tighter epsilon for sqrt robustness
#define POSITION_EPSILON 1e-9    // Tighter epsilon for position calculations
#define MIN_ARM_LENGTH 1e-3      // Minimum valid arm length
#define MAX_ARM_LENGTH 1e3       // Maximum valid arm length
#define LEAN_TOLERANCE 0.05      // Tolerance for lean vector magnitude (5%)

// Improved error codes
typedef enum {
    DELTA_OK = 0,
    DELTA_ERROR_INVALID_PARAMS,
    DELTA_ERROR_UNREACHABLE,
    DELTA_ERROR_NUMERICAL,
    DELTA_ERROR_MEMORY
} delta_error_t;

// Enhanced delta stepper structure with better organization
struct delta_stepper {
    struct stepper_kinematics sk;
    
    // Core kinematic parameters
    double arm2;                    // Square of arm length (cached for performance)
    double arm_length;              // Arm length (for validation and debugging)
    double base_tower_x, base_tower_y; // Tower base XY at Z=0
    
    // Lean parameters
    double lean_dx_dh;              // X offset factor due to lean = dx/dh_rail
    double lean_dy_dh;              // Y offset factor due to lean = dy/dh_rail  
    double lean_dz_dh;              // Z projection factor = dz_vertical/dh_rail
    
    // Optimization flags and cached values
    uint8_t has_lean;               // Flag to optimize no-lean case
    double lean_magnitude_sq;       // Cached: lean_dx_dh^2 + lean_dy_dh^2 + lean_dz_dh^2
    double inv_lean_magnitude_sq;   // Cached: 1/lean_magnitude_sq for division optimization
    
    // Reachability optimization
    double max_reach_xy_sq;         // Maximum reachable XY distance squared
    
    // Statistics for performance monitoring (optional, can be compiled out)
    #ifdef DELTA_STATS
    uint32_t calc_count;
    uint32_t unreachable_count;
    uint32_t lean_calc_count;
    #endif
};

// Enhanced coordinate validation with specific error detection
static inline int
validate_coord(struct coord c)
{
    // Check for NaN values
    if (isnan(c.x) || isnan(c.y) || isnan(c.z)) {
        return 0;
    }
    
    // Check for infinite values
    if (isinf(c.x) || isinf(c.y) || isinf(c.z)) {
        return 0;
    }
    
    // Check for reasonable bounds (prevents numerical issues)
    if (fabs(c.x) > MAX_ARM_LENGTH || fabs(c.y) > MAX_ARM_LENGTH || 
        fabs(c.z) > MAX_ARM_LENGTH) {
        return 0;
    }
    
    return 1;
}

// Enhanced sqrt argument clamping with better precision
static inline double
clamp_sqrt_arg(double val)
{
    return (val < 0.0 && val > -SQRT_EPSILON) ? 0.0 : val;
}

// Fast path for no-lean case with early reachability check
static double
delta_calc_position_no_lean(struct delta_stepper *ds, struct coord c)
{
    // Calculate radial distance to tower base
    double dx = ds->base_tower_x - c.x;
    double dy = ds->base_tower_y - c.y;
    double radial_dist_sq = dx*dx + dy*dy;
    
    // Early reachability check - if point is too far horizontally, it's unreachable
    if (radial_dist_sq > ds->max_reach_xy_sq) {
        return NAN;
    }
    
    // Calculate vertical projection of arm
    double arm_projection_sq = ds->arm2 - radial_dist_sq;
    
    // Handle near-zero case for numerical stability
    if (arm_projection_sq < 0.0) {
        if (arm_projection_sq > -SQRT_EPSILON) {
            return c.z; // Point is at exact reach limit
        }
        return NAN; // Point is unreachable
    }
    
    return sqrt(arm_projection_sq) + c.z;
}

// Enhanced quadratic solver with better numerical stability
static int
solve_quadratic_stable(double a, double b, double c, double *x1, double *x2)
{
    // Handle degenerate case where a ≈ 0
    if (fabs(a) < POSITION_EPSILON) {
        if (fabs(b) < POSITION_EPSILON) {
            return 0; // No solution or infinite solutions
        }
        *x1 = *x2 = -c / b;
        return 1;
    }
    
    double discriminant = b*b - 4.0*a*c;
    
    // No real solutions
    if (discriminant < -SQRT_EPSILON) {
        return 0;
    }
    
    // Handle near-zero discriminant
    if (discriminant < SQRT_EPSILON) {
        *x1 = *x2 = -b / (2.0*a);
        return 1;
    }
    
    // Use numerically stable formula to avoid catastrophic cancellation
    double sqrt_d = sqrt(discriminant);
    double q = -0.5 * (b + copysign(sqrt_d, b));
    
    *x1 = q / a;
    *x2 = c / q;
    
    // Ensure x1 >= x2 for consistent ordering
    if (*x1 < *x2) {
        double temp = *x1;
        *x1 = *x2;
        *x2 = temp;
    }
    
    return 2;
}

// Enhanced lean-aware calculation with optimized math
static double
delta_calc_position_with_lean(struct delta_stepper *ds, struct coord c)
{
    // Transform to tower-relative coordinates
    double dx0 = c.x - ds->base_tower_x;
    double dy0 = c.y - ds->base_tower_y;
    
    // Quadratic equation coefficients: A*h^2 + B*h + C = 0
    // This represents the intersection of the arm sphere with the lean direction
    double A = ds->lean_magnitude_sq;
    double B = -2.0 * (dx0 * ds->lean_dx_dh + dy0 * ds->lean_dy_dh + c.z * ds->lean_dz_dh);
    double C = dx0*dx0 + dy0*dy0 + c.z*c.z - ds->arm2;
    
    double h1, h2;
    int num_solutions = solve_quadratic_stable(A, B, C, &h1, &h2);
    
    if (num_solutions == 0) {
        return NAN; // No intersection - point unreachable
    }
    
    // Choose the physically meaningful solution
    // We prefer h > 0 (positive distance along rail)
    // If both solutions are valid, choose the one that makes physical sense
    if (num_solutions == 1) {
        return (h1 >= -POSITION_EPSILON) ? fmax(h1, 0.0) : NAN;
    }
    
    // Two solutions - choose the appropriate one
    // Typically we want the positive solution closest to zero
    if (h1 >= -POSITION_EPSILON && h2 >= -POSITION_EPSILON) {
        // Both positive - choose the smaller one (closer to nozzle)
        double result = fmin(h1, h2);
        return fmax(result, 0.0);
    } else if (h1 >= -POSITION_EPSILON) {
        return fmax(h1, 0.0);
    } else if (h2 >= -POSITION_EPSILON) {
        return fmax(h2, 0.0);
    }
    
    return NAN; // No valid positive solution
}

// Main position calculation with enhanced error handling
static double
delta_stepper_calc_position(struct stepper_kinematics *sk, struct move *m,
                            double move_time)
{
    struct delta_stepper *ds = container_of(sk, struct delta_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    
    #ifdef DELTA_STATS
    ds->calc_count++;
    #endif
    
    // Validate coordinate inputs
    if (!validate_coord(c)) {
        return NAN;
    }
    
    double result;
    
    // Use optimized path for no-lean case
    if (!ds->has_lean) {
        result = delta_calc_position_no_lean(ds, c);
    } else {
        #ifdef DELTA_STATS
        ds->lean_calc_count++;
        #endif
        result = delta_calc_position_with_lean(ds, c);
    }
    
    #ifdef DELTA_STATS
    if (isnan(result)) {
        ds->unreachable_count++;
    }
    #endif
    
    return result;
}

// Enhanced parameter validation with detailed error reporting
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
    if (arm2 <= 0.0) {
        return DELTA_ERROR_INVALID_PARAMS;
    }
    
    double arm_length = sqrt(arm2);
    if (arm_length < MIN_ARM_LENGTH || arm_length > MAX_ARM_LENGTH) {
        return DELTA_ERROR_INVALID_PARAMS;
    }
    
    // Validate lean parameters form a reasonable unit vector
    double lean_magnitude_sq = lean_dx_dh*lean_dx_dh + lean_dy_dh*lean_dy_dh + lean_dz_dh*lean_dz_dh;
    if (lean_magnitude_sq <= 0.0) {
        return DELTA_ERROR_INVALID_PARAMS;
    }
    
    double lean_magnitude = sqrt(lean_magnitude_sq);
    if (fabs(lean_magnitude - 1.0) > LEAN_TOLERANCE) {
        return DELTA_ERROR_INVALID_PARAMS;
    }
    
    return DELTA_OK;
}

// Enhanced public API with comprehensive initialization
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
    
    // Initialize all fields to zero
    memset(ds, 0, sizeof(*ds));
    
    // Initialize core parameters
    ds->arm2 = arm2;
    ds->arm_length = sqrt(arm2);
    ds->base_tower_x = base_tower_x;
    ds->base_tower_y = base_tower_y;
    ds->lean_dx_dh = lean_dx_dh;
    ds->lean_dy_dh = lean_dy_dh;
    ds->lean_dz_dh = lean_dz_dh;
    
    // Optimize for no-lean case (more precise detection)
    ds->has_lean = (fabs(lean_dx_dh) > POSITION_EPSILON || 
                    fabs(lean_dy_dh) > POSITION_EPSILON || 
                    fabs(lean_dz_dh - 1.0) > POSITION_EPSILON);
    
    // Cache lean magnitude calculations
    ds->lean_magnitude_sq = lean_dx_dh*lean_dx_dh + lean_dy_dh*lean_dy_dh + lean_dz_dh*lean_dz_dh;
    ds->inv_lean_magnitude_sq = 1.0 / ds->lean_magnitude_sq;
    
    // Cache maximum reachable XY distance (optimization for early rejection)
    ds->max_reach_xy_sq = ds->arm2;
    
    // Initialize stepper kinematics structure
    ds->sk.calc_position_cb = delta_stepper_calc_position;
    ds->sk.active_flags = AF_X | AF_Y | AF_Z;
    
    return &ds->sk;
}

// Enhanced utility function for debugging and monitoring
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

// New function for performance statistics (optional)
#ifdef DELTA_STATS
void __visible
delta_stepper_get_stats(struct stepper_kinematics *sk, uint32_t *calc_count,
                        uint32_t *unreachable_count, uint32_t *lean_calc_count)
{
    if (!sk) return;
    
    struct delta_stepper *ds = container_of(sk, struct delta_stepper, sk);
    
    if (calc_count) *calc_count = ds->calc_count;
    if (unreachable_count) *unreachable_count = ds->unreachable_count;
    if (lean_calc_count) *lean_calc_count = ds->lean_calc_count;
}

void __visible
delta_stepper_reset_stats(struct stepper_kinematics *sk)
{
    if (!sk) return;
    
    struct delta_stepper *ds = container_of(sk, struct delta_stepper, sk);
    
    ds->calc_count = 0;
    ds->unreachable_count = 0;
    ds->lean_calc_count = 0;
}
#endif

// New function for runtime parameter validation
int __visible
delta_stepper_validate_coord(struct stepper_kinematics *sk, double x, double y, double z)
{
    if (!sk) return 0;
    
    struct delta_stepper *ds = container_of(sk, struct delta_stepper, sk);
    struct coord c = {x, y, z};
    
    if (!validate_coord(c)) {
        return 0;
    }
    
    // Quick reachability check
    if (!ds->has_lean) {
        double dx = ds->base_tower_x - c.x;
        double dy = ds->base_tower_y - c.y;
        double radial_dist_sq = dx*dx + dy*dy;
        return radial_dist_sq <= ds->max_reach_xy_sq;
    }
    
    // For lean case, we need to do the full calculation
    double result = delta_calc_position_with_lean(ds, c);
    return !isnan(result);
}
