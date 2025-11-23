/*
 * encoder_abs.h
 *
 *  Created on: Nov 22, 2025
 *      Author: cg
 */

#ifndef ENCODERS_ENCODER_ABS_H_
#define ENCODERS_ENCODER_ABS_H_

#include "main.h"
#include <stdint.h>

// จำนวน absolute encoder ที่มี (ENC2..ENC5 = 4 ตัว)
#define ABS_ENCODER_COUNT   4

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    const char        *name;
} SteerEnc_t;

// เซ็ต CS ของ encoder ทุกตัวให้เป็น HIGH (inactive) ตอนเริ่มต้น
void ENC_StartALL(void);

// อ่านค่า raw ticks (0..1023) จาก encoder ตาม index (0..3)
// คืนค่า 0xFFFF ถ้ามี error
uint16_t ENC_ReadRaw_ByIndex(int idx);

// แปลง ticks -> องศา (0..360) ; คืน -1.0f ถ้า ticks == 0xFFFF
float ENC_TicksToDeg(uint16_t ticks);

// current to -> target
int16_t ENC10_Diff(uint16_t from, uint16_t to);

// Debug: อ่าน encoder ทั้ง 4 ตัวแล้ว printf ค่าออกมา
void Read_ENC_INDEX(void);

// อ่าน encoder ตาม index แล้วคืนเป็นองศา (ถ้า error คืน -1.0f)
float ENC_ReadDeg_ByIndex(int idx);

// Debug: อ่านและ print เฉพาะตัว idx (0..3)
void ENC_Debug_Print_One(int idx);

#endif /* ENCODERS_ENCODER_ABS_H_ */
