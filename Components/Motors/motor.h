/*
 * motor.h
 *
 *  Created on: Nov 22, 2025
 *      Author: cg
 */

#ifndef MOTORS_MOTOR_H_
#define MOTORS_MOTOR_H_

#include "main.h"
#include <stdint.h>

typedef enum {
    MOTOR_DIR_BRAKE = 0,
    MOTOR_DIR_FWD   = 1,
    MOTOR_DIR_REV   = -1
} MotorDir_t;

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           channel;
    GPIO_TypeDef      *portR;
    uint16_t           pinR;
    GPIO_TypeDef      *portL;
    uint16_t           pinL;
} Motor_t;

// มอเตอร์ 8 ตัว
extern Motor_t motors[8];

void Motor_PWM_StartALL(void);
void Motor_set(uint8_t idx, MotorDir_t dir, float duty);

void Motors_Drive_All(MotorDir_t dir, float duty);  // Motor Drive 1,3,5,7
void Motors_Steer_All(MotorDir_t dir, float duty);  // Motor Steer 2,4,6,8

void Robot_Move(MotorDir_t dir, float duty);
void Robot_Steer(MotorDir_t dir, float duty);

#endif /* MOTORS_MOTOR_H_ */
