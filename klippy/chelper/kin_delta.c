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

struct delta_stepper {
    struct stepper_kinematics sk;
    double arm2, tower_x, tower_y;
    double lean_dx_dh, lean_dy_dh, lean_dz_dh;
};

static double
delta_stepper_calc_position(struct stepper_kinematics *sk, struct move *m
                            , double move_time)
{
    struct delta_stepper *ds = container_of(sk, struct delta_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    double dx = ds->tower_x - c.x, dy = ds->tower_y - c.y;
    double height = sqrt(ds->arm2 - dx*dx - dy*dy) + c.z;
    
    // Apply lean compensation
    if (ds->lean_dx_dh != 0.0 || ds->lean_dy_dh != 0.0 || ds->lean_dz_dh != 0.0) {
        height += ds->lean_dx_dh * c.x + ds->lean_dy_dh * c.y + ds->lean_dz_dh * c.z;
    }
    
    return height;
}

struct stepper_kinematics * __visible
delta_stepper_alloc(double arm2, double base_tower_x, double base_tower_y,
                    double lean_dx_dh, double lean_dy_dh, double lean_dz_dh)
{
    struct delta_stepper *ds = malloc(sizeof(*ds));
    memset(ds, 0, sizeof(*ds));
    ds->arm2 = arm2;
    ds->tower_x = base_tower_x;
    ds->tower_y = base_tower_y;
    ds->lean_dx_dh = lean_dx_dh;
    ds->lean_dy_dh = lean_dy_dh;
    ds->lean_dz_dh = lean_dz_dh;
    ds->sk.calc_position_cb = delta_stepper_calc_position;
    ds->sk.active_flags = AF_X | AF_Y | AF_Z;
    return &ds->sk;
}
