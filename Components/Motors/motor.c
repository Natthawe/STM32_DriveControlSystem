/*
 * motor.c
 *
 *  Created on: Nov 22, 2025
 *      Author: cg
 */

#include "Motors/motor.h"

Motor_t motors[8] = {
    { &htim2, TIM_CHANNEL_1, MOTOR1_R_GPIO_Port, MOTOR1_R_Pin, MOTOR1_L_GPIO_Port, MOTOR1_L_Pin }, 	// Motor 1	PA5 	TIM2_CH1,	R=PF14,	L=PD15
    { &htim2, TIM_CHANNEL_3, MOTOR2_R_GPIO_Port, MOTOR2_R_Pin, MOTOR2_L_GPIO_Port, MOTOR2_L_Pin },	// Motor 2  PA10 	TIM2_CH3,	R=PD14, L=PE7
    { &htim2, TIM_CHANNEL_4, MOTOR3_R_GPIO_Port, MOTOR3_R_Pin, MOTOR3_L_GPIO_Port, MOTOR3_L_Pin },	// Motor 3  PB11 	TIM2_CH4,  	R=PF10, L=PE8
    { &htim5, TIM_CHANNEL_1, MOTOR4_R_GPIO_Port, MOTOR4_R_Pin, MOTOR4_L_GPIO_Port, MOTOR4_L_Pin },	// Motor 4  PA0 	TIM5_CH1, 	R=PF4, 	L=PF5
    { &htim5, TIM_CHANNEL_4, MOTOR5_R_GPIO_Port, MOTOR5_R_Pin, MOTOR5_L_GPIO_Port, MOTOR5_L_Pin },	// Motor 5  PA3 	TIM5_CH4,	R=PD4, 	L=PD5
    { &htim9, TIM_CHANNEL_1, MOTOR6_R_GPIO_Port, MOTOR6_R_Pin, MOTOR6_L_GPIO_Port, MOTOR6_L_Pin },	// Motor 6  PE5 	TIM9_CH1, 	R=PD6, 	L=PD7
    { &htim9, TIM_CHANNEL_2, MOTOR7_R_GPIO_Port, MOTOR7_R_Pin, MOTOR7_L_GPIO_Port, MOTOR7_L_Pin },	// Motor 7  PE6 	TIM9_CH2, 	R=PG3, 	L=PG2
    { &htim10,TIM_CHANNEL_1, MOTOR8_R_GPIO_Port, MOTOR8_R_Pin, MOTOR8_L_GPIO_Port, MOTOR8_L_Pin },	// Motor 8  PF6 	TIM10_CH1, 	R=PD2, 	L=PC12
};

void Motor_PWM_StartALL(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
}

// idx = 1-8, dir = BRAKE/FWD/REV, duty = 0.0 - 1.0
void Motor_set(uint8_t idx, MotorDir_t dir, float duty)
{
    if (idx < 1 || idx > 8) return;

    Motor_t *m = &motors[idx-1];

    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    uint32_t period = __HAL_TIM_GET_AUTORELOAD(m->htim);
    if (period == 0) period = 1;

    uint32_t duty_ticks = (uint32_t)(duty * (period + 1));
    if (duty_ticks > period) duty_ticks = period;

    __HAL_TIM_SET_COMPARE(m->htim, m->channel, duty_ticks);

//    printf("Motor_set idx=%d, dir=%d, duty=%.2f, ticks=%lu\r\n", idx, (int)dir, duty, duty_ticks);

    if (dir == MOTOR_DIR_FWD) {
        HAL_GPIO_WritePin(m->portR, m->pinR, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->portL, m->pinL, GPIO_PIN_RESET);
    } else if (dir == MOTOR_DIR_REV) {
        HAL_GPIO_WritePin(m->portR, m->pinR, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->portL, m->pinL, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(m->portR, m->pinR, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->portL, m->pinL, GPIO_PIN_RESET);
    }
}

void Motors_Drive_All(MotorDir_t dir, float duty)
{
    uint8_t drive_ids[] = {1, 3, 5, 7};
    for (uint8_t i = 0; i < sizeof(drive_ids)/sizeof(drive_ids[0]); i++) {
        Motor_set(drive_ids[i], dir, duty);
    }
}

void Motors_Steer_All(MotorDir_t dir, float duty)
{
    uint8_t steer_ids[] = {2, 4, 6, 8};
    for (uint8_t i = 0; i < sizeof(steer_ids)/sizeof(steer_ids[0]); i++) {
        Motor_set(steer_ids[i], dir, duty);
    }
}

void Robot_Move(MotorDir_t dir, float duty)
{
    Motors_Drive_All(dir, duty);
}

void Robot_Steer(MotorDir_t dir, float duty)
{
    Motors_Steer_All(dir, duty);
}
