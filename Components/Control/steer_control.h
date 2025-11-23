/*
 * steer_control.h
 *
 *  Created on: Nov 22, 2025
 *      Author: cg
 */

#ifndef CONTROL_STEER_CONTROL_H_
#define CONTROL_STEER_CONTROL_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "Control/pid_ctrl.h"

#define STEER_MAX_DEG  40.0f   // มุมเลี้ยวสูงสุด (+/- 40°)

typedef enum {
    RUN_MODE_STEER_CALIB = 0,   // โหมด CALIB : Calibrate ตั้ง zero_offset
    RUN_MODE_STEER_PID          // โหมด PID   : รักษามุมให้ตรง zero_offset
} RunMode_t;

typedef struct {
    const char *name;
    int         enc_index;      // index ใน enc_if[]  (0..3)
    uint16_t    zero_offset;    // tick เมื่อล้อตั้งตรง
    uint8_t     motor_idx;      // motor index 1..8
    int8_t      enc_dir;        // +1 / -1 สำหรับแกนนี้
    uint16_t    target_ticks;   // tick เป้าหมาย (0..1023)
    float       duty_base;      // duty ขั้นต่ำต่อแกน
    PID_t       pid;            // PID สำหรับควบคุมมุม
    float       last_duty;      // duty ล่าสุดที่ส่งให้มอเตอร์ (ไว้ debug)
} SteerAxis_t;

extern RunMode_t g_steer_mode;

/** เรียกตอน init: เซ็ตเป้าหมายทั้ง 4 ล้อให้เป็นศูนย์ (มุม 0°) */
void Steer_InitTargetsToZero(void);

/** อัปเดตมุมเป้าหมายแบบ ramp จาก cmd -> current แล้วเซ็ต target_ticks ของทุกล้อ */
void Steer_UpdateTargetWithRamp(float dt_s);

/** รัน P-control ของ steer ทั้ง 4 ล้อ ให้เข้าใกล้ target_ticks (เรียกทุก control loop) */
void Steer_UpdateAll(float dt_s);

/** สำหรับโหมด CALIB: อ่าน UART แล้วทำ JOG / ปริ้นค่า / เปลี่ยนโหมด (เรียกใน while(1) เมื่ออยู่ CALIB) */
void Steer_JogCalib_HandleUart(void);

/** พิมพ์ help ของโหมดบังคับเลี้ยว */
void Steer_PrintModeHelp(RunMode_t mode);

/** ตั้งมุมคำสั่งจาก robot (เช่นจาก Robot_ApplyTwist), หน่วย degree */
void Steer_SetCmdTargetDeg(float target_deg);

#endif /* CONTROL_STEER_CONTROL_H_ */
