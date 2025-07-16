// Delta kinematics stepper pulse time generation
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

#define MAX_SQRT_ARG 1e-9 // Small epsilon for sqrt robustness

struct delta_stepper {
    struct stepper_kinematics sk;
    double arm2;
    double base_tower_x, base_tower_y; // Tower base XY at Z=0
    double lean_dx_dh; // Precomputed factor for X offset due to lean = dx/dh_rail
    double lean_dy_dh; // Precomputed factor for Y offset due to lean = dy/dh_rail
    double lean_dz_dh; // Precomputed factor for Z projection = dz_vertical/dh_rail (cos of total lean)
};

// Calculate carriage height (h) along the rail for a given nozzle position (c.x, c.y, c.z)
// This solves the quadratic equation A*h^2 + B*h + C_prime = 0, where A=1 due to unit vector def.
static double
delta_stepper_calc_position(struct stepper_kinematics *sk, struct move *m,
                            double move_time)
{
    struct delta_stepper *ds = container_of(sk, struct delta_stepper, sk);
    struct coord c = move_get_coord(m, move_time); // Target nozzle coordinates (nx, ny, nz)

    // Validate inputs
    if (isnan(c.x) || isnan(c.y) || isnan(c.z)) {
        return NAN;
    }
    if (ds->arm2 <= 0.) {
        return NAN;
    }

    // If no lean, use the simpler, faster original formula
    if (ds->lean_dx_dh == 0. && ds->lean_dy_dh == 0. && ds->lean_dz_dh == 1.) {
        double dx_simple = ds->base_tower_x - c.x;
        double dy_simple = ds->base_tower_y - c.y;
        double val_under_sqrt = ds->arm2 - dx_simple*dx_simple - dy_simple*dy_simple;
        if (val_under_sqrt < -MAX_SQRT_ARG) {
            // Point is significantly unreachable. Propagate NAN like original sqrt would.
            // errorf("Delta kinematics: Unreachable point (no lean) at (%.3f,%.3f,%.3f)", c.x,c.y,c.z);
            return NAN;
        }
        if (val_under_sqrt < 0.) val_under_sqrt = 0.; // Clamp if slightly negative due to precision
        return sqrt(val_under_sqrt) + c.z;
    }

    // Lean-aware calculation: Solve h^2 + B*h + C_prime = 0
    // arm2 = (c.x - (base_tower_x + h*lean_dx_dh))^2
    //      + (c.y - (base_tower_y + h*lean_dy_dh))^2
    //      + (c.z - h*lean_dz_dh)^2
    // Let dx0 = c.x - base_tower_x, dy0 = c.y - base_tower_y
    double dx0 = c.x - ds->base_tower_x;
    double dy0 = c.y - ds->base_tower_y;

    // A = lean_dx_dh^2 + lean_dy_dh^2 + lean_dz_dh^2;  Should be 1 if factors are from unit vector.
    // For our definition, lean_dx_dh and lean_dy_dh are sin_components, lean_dz_dh is cos_component.
    // So A = (sin_x_comp)^2 + (sin_y_comp)^2 + (cos_total_lean)^2 = (sin_total_lean_xy_plane)^2 + cos_total_lean^2 = 1.
    // So we solve h^2 + B*h + C_prime = 0
    double B = -2. * (dx0 * ds->lean_dx_dh + dy0 * ds->lean_dy_dh + c.z * ds->lean_dz_dh);
    double C_prime = dx0*dx0 + dy0*dy0 + c.z*c.z - ds->arm2;

    double discriminant = B*B - 4.*C_prime;

    if (isnan(discriminant) || discriminant < -MAX_SQRT_ARG) {
        // Unreachable point with current lean model. Propagate NAN.
        errorf("Delta kinematics: Unreachable point (lean) at (%.3f,%.3f,%.3f), D=%.6e", c.x,c.y,c.z, discriminant);
        return NAN;
    }
    if (discriminant < 0.) discriminant = 0.; // Clamp if slightly negative

    // Two solutions for h: h1, h2
    // h = (-B +/- sqrt(discriminant)) / 2
    double sqrt_D = sqrt(discriminant);
    double h1 = (-B + sqrt_D) / 2.;
    double h2 = (-B - sqrt_D) / 2.;

    // Choose the physically correct root.
    // The carriage pivot is typically above the nozzle (h > z for arms pointing down).
    // Also, h should be positive (distance along rail from Z=0).
    // The Z coordinate of the carriage is h * lean_dz_dh.
    // We expect c.z (nozzle_z) to be roughly C_z - arm_projection_z.
    // So, h * lean_dz_dh should be greater than c.z for typical configurations.
    // If lean_dz_dh is close to 1 (small lean), then h should be greater than c.z.
    // The original non-lean formula is c.z + sqrt(...), so h is c.z + positive_value.
    // This implies we usually want the solution that is "c.z + something".
    // If B is large and negative (common), -B is positive.
    // We typically want the solution that places the carriage above the nozzle.
    // Consider nozzle at (0,0,200), arm 250. Carriage height is ~200 + sqrt(250^2 - R^2).
    // Let's test which solution gives a carriage Z (h * lean_dz_dh) that makes sense.
    // Usually, it's the one that results in a Z value for the carriage pivot
    // that is 'above' the nozzle Z by a significant amount (projection of arm).
    // If lean_dz_dh is ~1, h1 is generally preferred.

    // A simpler way: the value of h (distance along rail) should be positive and result
    // in the arm pointing generally "downwards" from carriage to effector.
    // The solution corresponding to (c.z + sqrt(L^2 - R_eff^2)) is generally h1.
    // Choose the root that corresponds to the arm being below the carriage.
    // h1 = (-B + sqrt_D) / 2.0; This is generally the correct physical solution.
    // h2 = (-B - sqrt_D) / 2.0;
    // We need h > 0 (distance along rail).
    // If h1 is positive, it's preferred. If h1 is negative and h2 is positive, use h2.
    // If both are negative, point is likely geometrically impossible for the "arm down" config.
    if (h1 >= -MAX_SQRT_ARG) { // Allow h1 to be very slightly negative due to precision
        return (h1 < 0.) ? 0. : h1;
    }
    if (h2 >= -MAX_SQRT_ARG) {
        // This case (h1 < 0 and h2 > 0) should be rare for valid delta positions.
        // errorf("Delta kinematics: IK h1 negative (%.2f), using h2 (%.2f)", h1, h2);
        return (h2 < 0.) ? 0. : h2;
    }

    // Both solutions significantly negative, or other issue.
    // errorf("Delta kinematics: Both IK solutions for h are negative (lean). D=%.6e, h1=%.2f, h2=%.2f", discriminant, h1, h2);
    return NAN; // Fallback if no valid positive solution
}

struct stepper_kinematics * __visible
delta_stepper_alloc(double arm2, double base_tower_x, double base_tower_y,
                    double lean_dx_dh, double lean_dy_dh, double lean_dz_dh)
{
    struct delta_stepper *ds = malloc(sizeof(*ds));
    if (!ds) {
        return NULL; // Handle malloc failure
    }
    memset(ds, 0, sizeof(*ds));
    
    // Validate inputs
    if (arm2 <= 0.) {
        free(ds);
        return NULL;
    }
    if (isnan(arm2) || isnan(base_tower_x) || isnan(base_tower_y) || 
        isnan(lean_dx_dh) || isnan(lean_dy_dh) || isnan(lean_dz_dh)) {
        free(ds);
        return NULL;
    }
    
    ds->arm2 = arm2;
    ds->base_tower_x = base_tower_x;
    ds->base_tower_y = base_tower_y;
    ds->lean_dx_dh = lean_dx_dh;
    ds->lean_dy_dh = lean_dy_dh;
    ds->lean_dz_dh = lean_dz_dh;
    ds->sk.calc_position_cb = delta_stepper_calc_position;
    ds->sk.active_flags = AF_X | AF_Y | AF_Z;
    return &ds->sk;
}
