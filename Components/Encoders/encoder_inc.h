/*
 * encoder_inc.h
 *
 *  Created on: Nov 23, 2025
 *      Author: cg
 */

#ifndef ENCODERS_ENCODER_INC_H_
#define ENCODERS_ENCODER_INC_H_

#include "main.h"
#include <stdint.h>

typedef struct {
    TIM_HandleTypeDef *htim;
    const char        *name;
    uint32_t           counts_per_rev;
    int8_t             sign;             // + FWD , -1 REV
    uint16_t           last_raw;         // Previous CNT
    int32_t            multi_ticks;      // tick สะสม (signed, multi-turn)
} DriveEnc_t;

#define DRIVE_NUM  4

extern DriveEnc_t drive_enc[DRIVE_NUM];

// เรียกตอน init: start encoder ทุกตัว + reset counter
void DriveEnc_InitAll(void);

// เรียกใน control loop: update multi_ticks จากแต่ละ timer
void DriveEnc_UpdateAll(void);

// อ่าน ticks สะสมของล้อ idx (0..3) ; ถ้า index ผิด -> คืน 0
int32_t DriveEnc_GetTicks(uint8_t idx);

// แปลง ticks เป็นจำนวนรอบ (revolutions) ; ถ้า index ผิด -> คืน 0.0f
float DriveEnc_GetRevs(uint8_t idx);

// พิมพ์ค่า encoder ทีละตัว
void DriveEnc_Debug_Print_One(uint8_t idx);

// พิมพ์ค่า encoder ทั้ง 4 ตัว
void DriveEnc_Debug_Print_All(void);

#endif /* ENCODERS_ENCODER_INC_H_ */
