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
        errorf("Delta kinematics: Unreachable point (lean) at (%.3f,%.3f,%.3f), D=%.3e", c.x,c.y,c.z, discriminant);
        logging.info("Delta kinematics: Unreachable point (lean) at (%.3f,%.3f,%.3f), D=%.3e", c.x,c.y,c.z, discriminant);
        return NAN;
    }
    if (discriminant < 0.) discriminant = 0.; // Clamp if slightly negative

    // Two solutions for h: h1, h2
    // h = (-B +/- sqrt(discriminant)) / 2
    double sqrt_D = sqrt(discriminant);
    double h1 = (-B + sqrt_D) / 2.;
    double h2 = (-B - sqrt_D) / 2.;

    // Choose the physically correct root. The carriage pivot must be above
    // the nozzle. We can determine this by checking the Z coordinate of the
    // carriage for each solution. The correct solution is the one that yields
    // a larger Z value for the carriage.
    double cz1 = h1 * ds->lean_dz_dh;
    double cz2 = h2 * ds->lean_dz_dh;

    if (cz1 >= cz2) {
        return h1;
    } else {
        return h2;
    }
}

struct stepper_kinematics * __visible
delta_stepper_alloc(double arm2, double base_tower_x, double base_tower_y,
                    double lean_dx_dh, double lean_dy_dh, double lean_dz_dh)
{
    struct delta_stepper *ds = malloc(sizeof(*ds));
    memset(ds, 0, sizeof(*ds));
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
