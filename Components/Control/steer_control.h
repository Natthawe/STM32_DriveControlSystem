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

#define STEER_MAX_DEG  		20.0f   // มุมเลี้ยวสูงสุด (+/- 40°) ที่ต้องเปลี่ยนมุมเลี้ยวเพราะล้อชนกัน
#define STEER_SPIN_MAX_DEG  38.0f   // ใช้เฉพาะ spin-in-place

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

/* ===== Calib debug logging =====
 * ใช้ดูทิศทาง encoder tick ตอน JOG (A/D) ในโหมด STEER CALIB
 * - เลือก 1..4 = log เฉพาะแกนที่เลือก
 * - เลือก 0     = log ทุกแกน
 */
extern uint8_t  g_steer_log_enable;     // 0/1
extern uint32_t g_steer_log_period_ms;  // default 100ms
extern int8_t   g_steer_calib_sel;      // -1=all, 0..3=axis

void Steer_Calib_LogTick(uint32_t now_ms);


/** เรียกตอน init: เซ็ตเป้าหมายทั้ง 4 ล้อให้เป็นศูนย์ (มุม 0 องศา) */
void Steer_InitTargetsToZero(void);

/** อัปเดตมุมเป้าหมายแบบ ramp จาก cmd -> current แล้วเซ็ต target_ticks ของทุกล้อ */
void Steer_UpdateTargetWithRamp(float dt_s);

/** รัน P-control ของ steer ทั้ง 4 ล้อ ให้เข้าใกล้ target_ticks (เรียกทุก control loop) */
void Steer_UpdateAll(float dt_s);

void Steer_DebugPrintAngles(void);

/** พิมพ์ help ของโหมดบังคับเลี้ยว */
void Steer_PrintModeHelp(RunMode_t mode);

/** ตั้งมุมคำสั่งจาก robot (เช่นจาก Robot_ApplyTwist), หน่วย degree */
void Steer_SetCmdTargetDeg(float target_deg);

/** ตั้งมุมเลี้ยวแพทเทิร์น SPIN-IN-PLACE */
void Steer_SetSpinAngleDeg(float angle_deg);

bool Steer_IsAtSpinTarget(void);

/* ===== NEW: UART char-dispatch (แก้ปัญหา UART ชนกัน) ===== */
void Steer_JogCalib_HandleChar(uint8_t ch);

/* เดิม: ยังเก็บไว้เผื่อใช้แบบเก่า */
void Steer_JogCalib_HandleUart(void);

/** CALIB: จับค่าปัจจุบันของ encoder ทุกล้อเป็น zero_offset และตั้ง target_ticks = zero */
void Steer_CalibCommitZeroAll(void);

/** เช็คว่าล้อทุกล้ออยู่ใกล้ target_ticks แล้วหรือยัง (ใช้ตอน align) */
bool Steer_IsNearTargetAll(float threshold_deg);

#endif /* CONTROL_STEER_CONTROL_H_ */
