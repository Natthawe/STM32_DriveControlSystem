/*
 * pid_ctrl.c
 *
 *  Created on: Nov 22, 2025
 *      Author: cg
 */

#include "Control/pid_ctrl.h"

float PID_Step(PID_t *p, float error, float dt)
{
    // P
    float p_out = p->kp * error;

    // I
    p->integrator += error * dt;
    float i_out = p->ki * p->integrator;

    // D
    float d_term = 0.0f;
    if (dt > 0.0f) {
        d_term = (error - p->prev_error) / dt;
    }
    float d_out = p->kd * d_term;

    float out = p_out + i_out + d_out;

    // clamp
    if (out > p->out_max) out = p->out_max;
    if (out < p->out_min) out = p->out_min;

    p->prev_error = error;
    return out;
}
