/*
 * drive_control.h
 *
 *  Created on: Nov 22, 2025
 *      Author: cg
 */

#ifndef CONTROL_DRIVE_CONTROL_H_
#define CONTROL_DRIVE_CONTROL_H_

#include "main.h"
#include <stdint.h>
#include "Motors/motor.h"
#include "Encoders/encoder_inc.h"   // DRIVE_NUM, DriveEnc_t
#include "Control/pid_ctrl.h"       // PID_t, PID_Step

/** โหมดการทำงานของ drive */
typedef enum {
    RUN_MODE_DRIVE_CALIB = 0,   // เทสทีละล้อ
    RUN_MODE_DRIVE_PID   = 1    // ขับด้วย PID จาก encoder
} DriveMode_t;

/** หนึ่งแกนของล้อ drive */
typedef struct {
    const char     *name;       // ชื่อเอาไว้ printf debug
    uint8_t         motor_idx;  // motor index 1,3,5,7
    DriveEnc_t     *wheel;      // pointer ไปที่ encoder ของล้อนั้น
    PID_t           pid;        // PID สำหรับควบคุมความเร็ว
    float           target_tps; // ความเร็วเป้าหมาย (ticks/sec) มีเครื่องหมาย
    int32_t         last_ticks; // multi_ticks ก่อนหน้า ใช้คำนวณความเร็ว
    float           max_tps;    // ความเร็วสูงสุด (ticks/sec) ของแต่ละล้อ  ใช้เป็น ref/limit
    float           duty_base;  // duty ขั้นต่ำของล้อนั้น (ค่านี้เอาไว้ชน friction เพราะแต่ละล้อ friction ไม่เท่ากัน)
    float           meas_tps;   // ความเร็วที่วัดได้ล่าสุด
    float           last_duty;  // duty ล่าสุดที่ส่งให้มอเตอร์
} DriveAxis_t;

/* drive axis ทั้ง 4 ล้อ (index 0..3) */
extern DriveAxis_t drive_axes[DRIVE_NUM];

/* global state สำหรับโหมด drive + test */
extern DriveMode_t g_drive_mode;     // RUN_MODE_DRIVE_PID / RUN_MODE_DRIVE_CALIB
extern int8_t      g_test_drive_idx; // -1 = all, 0..3 = ล้อที่เลือก
extern float       g_test_duty;      // duty ใช้เทส
extern int8_t      g_test_dir;       // 1 = forward, -1 = reverse
extern float g_cmd_dir_sign;
extern float g_cmd_speed_norm;
extern float g_current_speed_norm;
extern float g_cmd_target_tps;
extern float g_current_target_tps;

/* ===== Calib debug logging =====
 * ใช้ดู sign ของ encoder ขณะสั่ง F/R ในโหมด DRIVE CALIB
 * - g_test_drive_idx = -1 (เลือก 0) -> log ทุกล้อ
 * - g_test_drive_idx = 0..3 (เลือก 1..4) -> log เฉพาะล้อที่เลือก
 */
extern uint8_t  g_drive_log_enable;     // 0/1
extern uint32_t g_drive_log_period_ms;  // default 100ms

// อัปเดต meas_tps/last_ticks แบบ "วัดอย่างเดียว" (ไม่สั่งมอเตอร์) สำหรับโหมด CALIB
void Drive_UpdateMeasOnly(float dt_s);

// เรียกเป็นระยะเพื่อ printf log (throttle ตาม g_drive_log_period_ms)
void Drive_Calib_LogTick(uint32_t now_ms);

/* ===== encoder sign ต่อ wheel (หลังถอด encoder ต้องเช็คอันนี้) ===== */
extern int8_t g_drive_enc_sign[DRIVE_NUM];

/* เรียกตอน boot: reset ค่า PID / target */
void Drive_InitAll(void);

/* reset PID และเบรคทุกล้อ (ใช้ตอน STOP / timeout) */
void Drive_StopAll(void);

/* reset PID ของล้อขับทุกล้อ */
void Drive_ResetPIDAll(void);

/* สั่งเบรคมอเตอร์ drive ทุกล้อ */
void Drive_BrakeAll(void);

/* อัปเดต target_tps ของทุกล้อด้วย ramp */
void Drive_UpdateTargetsWithRamp(float dt_s, float *cmd_target_tps, float *current_target_tps);

/* อัปเดต PID ของ wheel encoders ทุกล้อ (เรียกทุก control loop) */
void Drive_UpdateAll(float dt_s);

/* เดิม: wrapper เผื่อใช้แบบเก่า | handle UART command สำหรับโหมด test drive (กดปุ่มเลือกโหมด/ล้อ/duty) */
void Process_UART_TestDrive(void);

/* ===== NEW: handle one char (ใช้กับ main.c dispatcher) ===== */
void Drive_Test_HandleChar(uint8_t ch);

void Drive_SetSpinTargets(float base_tps, float spin_dir);

#endif /* CONTROL_DRIVE_CONTROL_H_ */
