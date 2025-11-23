/*
 * pid_ctrl.h
 *
 *  Created on: Nov 22, 2025
 *      Author: cg
 */

#ifndef CONTROL_PID_CTRL_H_
#define CONTROL_PID_CTRL_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float integrator;
    float prev_error;
    float out_min;
    float out_max;
} PID_t;

float PID_Step(PID_t *p, float error, float dt);

#endif /* CONTROL_PID_CTRL_H_ */
