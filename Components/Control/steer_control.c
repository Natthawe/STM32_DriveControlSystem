/*
 * steer_control.c
 *
 *  Created on: Nov 22, 2025
 *      Author: cg
 */

#include "Control/steer_control.h"
#include "Motors/motor.h"
#include "Encoders/encoder_abs.h"
#include "Control/pid_ctrl.h"

#include <math.h>
#include <stdio.h>
#include <stdbool.h>

// ====== Steer PID (MOTOR 2,4,6,8) ======
#define STEER_NUM   4

#define STEER_DEADBAND_TICKS    5.0f       	// ถ้าอยู่ในระยะนี้ถือว่าตรงแล้ว ไม่ต้องหมุน

#define STEER_JOG_DUTY_DEFAULT  0.8f     	// duty เวลา jog
#define STEER_JOG_MS_DEFAULT    50U     	// หมุนครั้งละ x ms

#define STEER_TICKS_MAX      	1024U
#define ENC_TICKS_PER_REV    	1024.0f
#define ENC_TICKS_PER_DEG    	(ENC_TICKS_PER_REV / 360.0f)  // ≈ 2.8444 tick/deg
#define STEER_RAMP_DEG_PER_SEC  90.0f       // deg/s เปลี่ยนให้เร็วขึ้น เพราะถ้าช้าล้อมันจะสั่น

RunMode_t g_steer_mode = RUN_MODE_STEER_PID;

float g_cmd_target_deg     = 0.0f;          // มุมที่ ROS/serial สั่งมา
float g_current_target_deg = 0.0f;          // มุมเป้าหมายที่ ramp แล้ว [deg]

// ===== Calib debug logging (Steer) =====
uint8_t  g_steer_log_enable    = 0;      // เปิด/ปิดการพิมพ์ log
uint32_t g_steer_log_period_ms = 100U;   // ความถี่ log (ms)
int8_t   g_steer_calib_sel     = 0;      // -1=ALL, 0..3=axis (ใช้กับ log)
static uint32_t s_steer_log_last_ms = 0U;

// map encoder index -> motor index 2,4,6,8
const uint8_t steer_motor_ids[STEER_NUM] = { 2, 4, 6, 8 };

SteerAxis_t steer_axes[] = {
    //  name, enc_idx, zero,  motor, enc_dir target, duty_base, PID
	{ "STEER_FR", 0, 131, 2, -1, 131, 0.7f,
	  { .kp=0.01f, .ki=0.002f, .kd=0.0f,
		.integrator=0, .prev_error=0,
		.out_min=-1.0f, .out_max=1.0f }, 0.0f },    // ENC2 -> Motor2 (front-right steer)

	{ "STEER_RR", 1, 468, 4, -1, 468, 0.6f,
	  { .kp=0.007f, .ki=0.002f, .kd=0.0f,
		.integrator=0, .prev_error=0,
		.out_min=-1.0f, .out_max=1.0f }, 0.0f },    // ENC3 -> Motor4 (rear-right steer)

	{ "STEER_RL", 2, 803, 6, -1, 803, 0.6f,
	  { .kp=0.006f, .ki=0.002f, .kd=0.0f,
		.integrator=0, .prev_error=0,
		.out_min=-1.0f, .out_max=1.0f }, 0.0f },    // ENC4 -> Motor6 (rear-left steer)

	{ "STEER_FL", 3, 83, 8, -1, 83, 0.75f,
	  { .kp=0.015f, .ki=0.01f, .kd=0.0f,
		.integrator=0, .prev_error=0,
		.out_min=-1.0f, .out_max=1.0f }, 0.0f },    // ENC5 -> Motor8 (front-left steer)
};

// +1 = เวลาสั่งเลี้ยวมุม +deg ให้หมุนไปทางเดียวกับ angle_deg
// -1 = เวลาสั่งเลี้ยวมุม +deg ให้หมุนไปทางตรงข้าม (สำหรับล้อหลังใน all-wheel-steer)
const int8_t steer_turn_sign[STEER_NUM] = {
    +1,  // 0: STEER_FR (front-right)
    -1,  // 1: STEER_RR (rear-right)
    -1,  // 2: STEER_RL (rear-left)
    +1   // 3: STEER_FL (front-left)
};

const int8_t steer_spin_sign[STEER_NUM] = {
    +1,  // 0: STEER_FR -> ซ้าย
    -1,  // 1: STEER_RR -> ขวา
    +1,  // 2: STEER_RL -> ซ้าย
    -1   // 3: STEER_FL -> ขวา
};

// ===== motor direction sign (แก้กรณีมอเตอร์บางล้อกลับด้าน) =====
// +1 = ปกติ
// -1 = สลับ FWD/REV เฉพาะแกนนั้น
static const int8_t steer_motor_dir_sign[STEER_NUM] = {
    +1,  // 0: STEER_FR
    +1,  // 1: STEER_RR
    +1,  // 2: STEER_RL
    -1   // 3: STEER_FL  <<<<< ล้อหน้าซ้าย jog กลับด้าน ให้เป็น -1
};

static inline MotorDir_t Steer_ApplyMotorDirSign(uint32_t axis, MotorDir_t dir)
{
    if (axis >= STEER_NUM) return dir;

    if (steer_motor_dir_sign[axis] > 0) {
        return dir; // ปกติ
    }

    // กลับทิศเฉพาะ FWD/REV (BRAKE ไม่ต้องสลับ)
    if (dir == MOTOR_DIR_FWD) return MOTOR_DIR_REV;
    if (dir == MOTOR_DIR_REV) return MOTOR_DIR_FWD;
    return dir;
}

uint16_t wrap_ticks(int32_t t)
{
    while (t < 0)                 			t += STEER_TICKS_MAX;
    while (t >= (int32_t)STEER_TICKS_MAX)	t -= STEER_TICKS_MAX;
    return (uint16_t)t;
}

void Steer_SetTargetAngleDeg(float angle_deg)
{
    // clamp มุมไม่เกิน +- STEER_MAX_DEG
    if (angle_deg >  STEER_MAX_DEG) angle_deg =  STEER_MAX_DEG;
    if (angle_deg < -STEER_MAX_DEG) angle_deg = -STEER_MAX_DEG;

    for (uint32_t i = 0; i < STEER_NUM; ++i) {
        //  ล้อหน้า: steer_turn_sign = +1  -> ได้มุม +angle_deg
        //  ล้อหลัง: steer_turn_sign = -1  -> ได้มุม -angle_deg
        float delta_ticks_f = angle_deg * (float)steer_turn_sign[i] * ENC_TICKS_PER_DEG;

        int32_t base  = (int32_t)steer_axes[i].zero_offset;
        int32_t delta = (int32_t)lrintf(delta_ticks_f);

        steer_axes[i].target_ticks = wrap_ticks(base + delta);
    }
}

// ตั้งมุมเลี้ยวแบบ SPIN-IN-PLACE ตามแพทเทิร์นด้านบน
void Steer_SetSpinAngleDeg(float angle_deg)
{
    // เก็บค่า request ไว้ debug
    float angle_req = angle_deg;

    // clamp มุมไม่เกิน +- STEER_SPIN_MAX_DEG
    if (angle_deg >  STEER_SPIN_MAX_DEG) angle_deg =  STEER_SPIN_MAX_DEG;
    if (angle_deg < -STEER_SPIN_MAX_DEG) angle_deg = -STEER_SPIN_MAX_DEG;

    for (uint32_t i = 0; i < STEER_NUM; ++i) {
        float delta_ticks_f = angle_deg * (float)steer_spin_sign[i] * ENC_TICKS_PER_DEG;

        int32_t base  = (int32_t)steer_axes[i].zero_offset;
        int32_t delta = (int32_t)lrintf(delta_ticks_f);

        steer_axes[i].target_ticks = wrap_ticks(base + delta);
    }

    // DEBUG
    printf("[SPIN] Steer_SetSpinAngleDeg req=%.2f, clamped=%.2f (max=%.2f)\r\n",
           angle_req, angle_deg, (float)STEER_SPIN_MAX_DEG);
}

// ใช้สำหรับโหมด SPIN: เช็คว่าล้อเลี้ยวทุกล้อเข้าใกล้มุมเป้าหมายแล้วหรือยัง
// ถ้า error ทุกล้อน้อยกว่าเกณฑ์ -> return true
bool Steer_IsAtSpinTarget(void)
{
    // ใช้ threshold ประมาณ 2 องศา
    const float READY_DEG   = 2.0f;
    const float READY_TICKS = READY_DEG * ENC_TICKS_PER_DEG;

    for (uint32_t i = 0; i < STEER_NUM; ++i) {
        SteerAxis_t *ax = &steer_axes[i];

        uint16_t ticks = ENC_ReadRaw_ByIndex(ax->enc_index);
        if (ticks == 0xFFFF) {
            // ถ้าอ่าน encoder ไม่ได้ ถือว่ายังไม่ ready
            return false;
        }

        int16_t err_ticks = ENC10_Diff(ticks, ax->target_ticks);
        err_ticks = (int16_t)(err_ticks * ax->enc_dir);

        if (fabsf((float)err_ticks) > READY_TICKS) {
            // ล้อนี้ยังห่างจากเป้าหมายมากไป -> ยังไม่พร้อม
            return false;
        }
    }

    // ทุกล้ออยู่ในเกณฑ์แล้ว
    return true;
}

void SteerMotor_Jog(uint8_t axis, MotorDir_t dir, float duty)
{
    if (axis >= STEER_NUM) return;
    uint8_t motor_idx = steer_motor_ids[axis];

    MotorDir_t dir2 = Steer_ApplyMotorDirSign(axis, dir);
    Motor_set(motor_idx, dir2, duty);
}

void Steer_Calib_LogTick(uint32_t now_ms)
{
    if (!g_steer_log_enable) return;

    uint32_t dt = now_ms - s_steer_log_last_ms;
    if (dt < g_steer_log_period_ms) return;
    s_steer_log_last_ms = now_ms;

    static uint16_t last_ticks[STEER_NUM] = {0};
    static uint8_t  inited = 0;

    if (!inited) {
        for (uint32_t i = 0; i < STEER_NUM; ++i) {
            uint16_t t = ENC_ReadRaw_ByIndex(steer_axes[i].enc_index);
            last_ticks[i] = (t == 0xFFFF) ? 0 : t;
        }
        inited = 1;
    }

    if (g_steer_calib_sel < 0) {
        printf("[SLOG] %lu ms sel=ALL\r\n", (unsigned long)now_ms);
        for (uint32_t i = 0; i < STEER_NUM; ++i) {
            SteerAxis_t *ax = &steer_axes[i];
            uint16_t t = ENC_ReadRaw_ByIndex(ax->enc_index);
            if (t == 0xFFFF) {
                printf("  a%lu %s ENC ERROR\r\n", (unsigned long)(i+1), ax->name);
                continue;
            }
            int16_t d_raw = ENC10_Diff(t, last_ticks[i]);
            int16_t d_enc = (int16_t)(d_raw * ax->enc_dir);
            last_ticks[i] = t;

            printf("  a%lu %s m%u enc_dir=%+d tick=%4u deg=%6.2f dRaw=%+4d dEnc=%+4d zero=%4u tgt=%4u\r\n",
                   (unsigned long)(i+1),
                   ax->name,
                   (unsigned)ax->motor_idx,
                   (int)ax->enc_dir,
                   (unsigned)t,
                   (double)ENC_TicksToDeg(t),
                   (int)d_raw,
                   (int)d_enc,
                   (unsigned)ax->zero_offset,
                   (unsigned)ax->target_ticks);
        }
    } else {
        uint32_t i = (uint32_t)g_steer_calib_sel;
        if (i >= STEER_NUM) return;

        SteerAxis_t *ax = &steer_axes[i];
        uint16_t t = ENC_ReadRaw_ByIndex(ax->enc_index);
        if (t == 0xFFFF) {
            printf("[SLOG] %lu ms sel=a%lu %s ENC ERROR\r\n",
                   (unsigned long)now_ms,
                   (unsigned long)(i+1),
                   ax->name);
            return;
        }
        int16_t d_raw = ENC10_Diff(t, last_ticks[i]);
        int16_t d_enc = (int16_t)(d_raw * ax->enc_dir);
        last_ticks[i] = t;

        printf("[SLOG] %lu ms sel=a%lu | %s m%u enc_dir=%+d tick=%4u deg=%6.2f dRaw=%+4d dEnc=%+4d zero=%4u tgt=%4u\r\n",
               (unsigned long)now_ms,
               (unsigned long)(i+1),
               ax->name,
               (unsigned)ax->motor_idx,
               (int)ax->enc_dir,
               (unsigned)t,
               (double)ENC_TicksToDeg(t),
               (int)d_raw,
               (int)d_enc,
               (unsigned)ax->zero_offset,
               (unsigned)ax->target_ticks);
    }
}

// พิมพ์ค่า zero_offset ปัจจุบันของทุกล้อในรูปโค้ด steer_axes[]
void Steer_PrintCurrentZeroConfig(void)
{
    uint16_t ticks[STEER_NUM];
    float    degs[STEER_NUM];

    // อ่าน encoder ทุกแกน
    for (uint32_t i = 0; i < STEER_NUM; ++i) {
        uint16_t t = ENC_ReadRaw_ByIndex(steer_axes[i].enc_index);
        ticks[i] = t;
        degs[i]  = ENC_TicksToDeg(t);
    }

    printf("\r\n==== STEER ZERO CAL RESULT ====\r\n");
    for (uint32_t i = 0; i < STEER_NUM; ++i) {
        printf("[ZCAL] %s: tick=%4u (%.2f deg)\r\n",
               steer_axes[i].name,
               (unsigned)ticks[i],
               degs[i]);
    }

    printf("\r\n/* Paste config below into your code (steer_axes[]): */\r\n");
    printf("static SteerAxis_t steer_axes[] = {\r\n");

    for (uint32_t i = 0; i < STEER_NUM; ++i) {
        SteerAxis_t *ax = &steer_axes[i];
        uint16_t z = ticks[i];

        printf("    { \"%s\", %d, %u, %u, %d, %u, %.2ff,\r\n"
               "      { .kp=%.6ff, .ki=%.6ff, .kd=%.6ff,\r\n"
               "        .integrator=0, .prev_error=0,\r\n"
               "        .out_min=%.1ff, .out_max=%.1ff }, 0.0f },\r\n",
               ax->name,
               ax->enc_index,
               (unsigned)z,        // zero_offset ใหม่
               ax->motor_idx,
               (int)ax->enc_dir,
               (unsigned)z,        // target_ticks เริ่มต้น = zero_offset
               ax->duty_base,
               ax->pid.kp,
               ax->pid.ki,
               ax->pid.kd,
               ax->pid.out_min,
               ax->pid.out_max);
    }

    printf("};\r\n\n");
}

void Steer_SetCmdTargetDeg(float angle_deg)
{
    // clamp อีกรอบเผื่อ caller ส่งเกิน
    if (angle_deg >  STEER_MAX_DEG) angle_deg =  STEER_MAX_DEG;
    if (angle_deg < -STEER_MAX_DEG) angle_deg = -STEER_MAX_DEG;
    g_cmd_target_deg = angle_deg;
}

// เริ่มต้น: ให้ target = zero ทั้งหมด
void Steer_InitTargetsToZero(void)
{
    g_cmd_target_deg     = 0.0f;
    g_current_target_deg = 0.0f;
    Steer_SetTargetAngleDeg(0.0f);
}

// อัปเดตมุมเป้าหมายแบบ ramp จาก g_cmd_target_deg -> g_current_target_deg
void Steer_UpdateTargetWithRamp(float dt_s)
{
    if (dt_s <= 0.0f) dt_s = 0.001f;

    float delta = g_cmd_target_deg - g_current_target_deg;

    // limit การเปลี่ยนแปลงต่อรอบ (deg)
    float max_step = STEER_RAMP_DEG_PER_SEC * dt_s;

    if (delta >  max_step) delta =  max_step;
    if (delta < -max_step) delta = -max_step;

    g_current_target_deg += delta;

    // deadzone เล็ก ๆ รอบ 0 กันค้าง (ค่าน้อยมากๆ)
    if (fabsf(g_current_target_deg) < 0.05f && fabsf(g_cmd_target_deg) < 0.05f) {
        g_current_target_deg = 0.0f;
    }

    // เซ็ต target_ticks ของทุกล้อจากมุมที่ ramp แล้ว
    Steer_SetTargetAngleDeg(g_current_target_deg);
}

// รัน P-control ของทั้ง 4 ล้อให้เข้า target_ticks (ใช้ encoder_abs)
void Steer_UpdateAll(float dt_s)
{
    if (dt_s <= 0.0f) dt_s = 0.001f;

    static uint32_t dbg_cnt = 0;
    dbg_cnt++;

    // ถ้า |cmd - current| > 0.5° แปลว่ายัง ramp อยู่
    bool steering_ramping = (fabsf(g_cmd_target_deg - g_current_target_deg) > 0.5f);

    for (uint32_t i = 0; i < STEER_NUM; ++i) {
        SteerAxis_t *ax = &steer_axes[i];

        uint16_t ticks = ENC_ReadRaw_ByIndex(ax->enc_index);
        if (ticks == 0xFFFF) {
            Motor_set(ax->motor_idx, MOTOR_DIR_BRAKE, 0.0f);
            ax->last_duty = 0.0f;
            continue;
        }

        int16_t err_ticks = ENC10_Diff(ticks, ax->target_ticks);
        err_ticks = (int16_t)(err_ticks * ax->enc_dir);
        float err_f = (float)err_ticks;

        bool in_deadband = (err_f > -STEER_DEADBAND_TICKS &&
                            err_f <  STEER_DEADBAND_TICKS);

        // ถ้าไม่ ramp แล้ว และอยู่ใน deadband -> hold เฉย ๆ
        if (!steering_ramping && in_deadband) {
            Motor_set(ax->motor_idx, MOTOR_DIR_BRAKE, 0.0f);
            ax->last_duty = 0.0f;
            continue;
        }

        // ----- P control ล้วน ๆ -----
        // u = Kp * error  (หน่วยเดียวกับ out_min/out_max = -1..+1)
        float u = ax->pid.kp * err_f;

        // ----- PID control -----
        // float u = PID_Step(&ax->pid, err_f, dt_s);

        // clamp ตามขอบเขต PID เดิม
        if (u > ax->pid.out_max) u = ax->pid.out_max;
        if (u < ax->pid.out_min) u = ax->pid.out_min;

        MotorDir_t dir;
        float duty_raw;

        if (u >= 0.0f) {
            dir      = MOTOR_DIR_FWD;
            duty_raw =  u;
        } else {
            dir      = MOTOR_DIR_REV;
            duty_raw = -u;
        }

        // base duty + clamp
        float duty = duty_raw;
        if (duty < ax->duty_base) duty = ax->duty_base;
        if (duty > 1.0f)          duty = 1.0f;

        ax->last_duty = duty;
//        Motor_set(ax->motor_idx, dir, duty);

        MotorDir_t dir2 = Steer_ApplyMotorDirSign(i, dir);
        Motor_set(ax->motor_idx, dir2, duty);
        // debug
        /*
        if (dbg_cnt % 20 == 0) {
            float err_deg = err_f * 360.0f / 1024.0f;
            printf("[%s] err=%4d (%.2f deg), u=%.3f, duty=%.2f, dir=%d (ramp=%d)\r\n",
                   ax->name,
                   (int)err_ticks,
                   err_deg,
                   u,
                   duty,
                   (int)dir,
                   (int)steering_ramping);
        }
        */
    }
}

void Steer_DebugPrintAngles(void)
{
    printf("\r\n===== STEER ANGLES DEBUG =====\r\n");

    for (uint32_t i = 0; i < STEER_NUM; ++i) {
        SteerAxis_t *ax = &steer_axes[i];

        // อ่าน encoder
        uint16_t t = ENC_ReadRaw_ByIndex(ax->enc_index);
        if (t == 0xFFFF) {
            printf("[%s] ENC ERROR\r\n", ax->name);
            continue;
        }

        // องศา absolute (0..360 จากฟังก์ชัน ENC_TicksToDeg)
        float abs_deg = ENC_TicksToDeg(t);

        // องศา relative เทียบกับ zero_offset ของล้อนั้น (มุมจริงที่เราสนใจ)
        int16_t diff_zero = ENC10_Diff(t, ax->zero_offset);
        float rel_deg = (float)diff_zero / ENC_TICKS_PER_DEG;

        // องศา target ของล้อเทียบกับ zero_offset เช่น เป้า 45° / -45°
        int16_t diff_target = ENC10_Diff(ax->target_ticks, ax->zero_offset);
        float target_deg = (float)diff_target / ENC_TICKS_PER_DEG;

        printf("[%s] enc=%4u | abs=%.2f deg | rel=%.2f deg | target=%.2f deg\r\n",
               ax->name,
               (unsigned)t,
               abs_deg,
               rel_deg,
               target_deg);
    }

    printf("================================\r\n");
}

void Steer_CalibCommitZeroAll(void)
{
    printf("\r\n[ZCAL] Commit ZERO from current encoder ticks...\r\n");

    for (uint32_t i = 0; i < STEER_NUM; ++i) {
        SteerAxis_t *ax = &steer_axes[i];

        uint16_t t = ENC_ReadRaw_ByIndex(ax->enc_index);
        if (t == 0xFFFF) {
            printf("[ZCAL] %s: ENC ERROR -> skip\r\n", ax->name);
            continue;
        }

        ax->zero_offset  = t;
        ax->target_ticks = t;

        // รีเซ็ต integrator กันค้างค่า (เผื่อคุณสลับไปใช้ PID_Step ภายหลัง)
        ax->pid.integrator = 0.0f;
        ax->pid.prev_error = 0.0f;

        printf("[ZCAL] %s: zero_offset=%u (%.2f deg)\r\n",
               ax->name, (unsigned)t, ENC_TicksToDeg(t));
    }

    // ทำให้ cmd/current กลับ 0 เพื่อไม่ให้สั่งมุมอื่นค้างอยู่
    g_cmd_target_deg     = 0.0f;
    g_current_target_deg = 0.0f;

    printf("[ZCAL] DONE. Now you can switch to PID safely.\r\n");
}

bool Steer_IsNearTargetAll(float threshold_deg)
{
    float thr_ticks = threshold_deg * ENC_TICKS_PER_DEG;

    for (uint32_t i = 0; i < STEER_NUM; ++i) {
        SteerAxis_t *ax = &steer_axes[i];

        uint16_t t = ENC_ReadRaw_ByIndex(ax->enc_index);
        if (t == 0xFFFF) return false;

        int16_t err_ticks = ENC10_Diff(t, ax->target_ticks);
        err_ticks = (int16_t)(err_ticks * ax->enc_dir);

        if (fabsf((float)err_ticks) > thr_ticks) {
            return false;
        }
    }
    return true;
}


void Steer_PrintModeHelp(RunMode_t mode)
{
    printf("\r\n=============================\r\n");
    if (mode == RUN_MODE_STEER_CALIB) {
        printf(" RUN MODE: STEER CALIB (JOG)\r\n");
        printf("  1-4 : select steer axis (1..4)\r\n");
        printf("  a   : turn LEFT  (REV)\r\n");
        printf("  d   : turn RIGHT (FWD)\r\n");
        printf("  s   : stop motor\r\n");
        printf("  z   : read encoder tick for current axis\r\n");
        printf("  x   : SET zero_offset for current axis (and target=zero)\r\n");
        printf("  Z   : print all axis zeros (steer_axes[] config)\r\n");
        printf("  p   : switch to PID mode\r\n");
    } else {
        printf(" RUN MODE: STEER PID (from ROS /cmd_vel)\r\n");
        printf("  - Drive: from serial/UDP Twist\r\n");
        printf("  - Steer: PID to target angle\r\n");
        printf("  c : switch to CALIB mode\r\n");
    }
    printf("=============================\r\n");
}

/* ===== handle one char (ใช้กับ main.c dispatcher) ===== */
void Steer_JogCalib_HandleChar(uint8_t ch)
{
    static uint8_t current_axis = 0;              // 0..3 -> STEER1..4

    const float    jog_duty = STEER_JOG_DUTY_DEFAULT;
    const uint32_t JOG_MS   = STEER_JOG_MS_DEFAULT;

    switch (ch)
    {
    case '1':
    case '2':
    case '3':
    case '4':
        current_axis = (uint8_t)(ch - '1');  // '1'->0, '2'->1 ...
        g_steer_calib_sel = (int8_t)current_axis;
        printf("Selected STEER axis %d\r\n", (int)(current_axis + 1));
        break;

    case '0':
        g_steer_calib_sel = -1;
        printf("Selected STEER axis ALL\r\n");
        break;

    case 'a':
    case 'A':
        SteerMotor_Jog(current_axis, MOTOR_DIR_REV, jog_duty);
        HAL_Delay(JOG_MS);
        SteerMotor_Jog(current_axis, MOTOR_DIR_BRAKE, 0.0f);
        printf("Axis %d: JOG LEFT  (%lu ms, duty=%.2f)\r\n",
               (int)(current_axis + 1),
               (unsigned long)JOG_MS,
               jog_duty);
        break;

    case 'd':
    case 'D':
        SteerMotor_Jog(current_axis, MOTOR_DIR_FWD, jog_duty);
        HAL_Delay(JOG_MS);
        SteerMotor_Jog(current_axis, MOTOR_DIR_BRAKE, 0.0f);
        printf("Axis %d: JOG RIGHT (%lu ms, duty=%.2f)\r\n",
               (int)(current_axis + 1),
               (unsigned long)JOG_MS,
               jog_duty);
        break;

    case 's':
    case 'S':
        SteerMotor_Jog(current_axis, MOTOR_DIR_BRAKE, 0.0f);
        printf("Axis %d: STOP\r\n", (int)(current_axis + 1));
        break;

    case 'z':
    {
        // FIX: ต้องอ่าน enc_index จริงของแกนนั้น
        SteerAxis_t *ax = &steer_axes[current_axis];
        uint16_t ticks = ENC_ReadRaw_ByIndex(ax->enc_index);
        if (ticks == 0xFFFF) {
            printf("CALIB axis %d : ENC ERROR\r\n", (int)(current_axis + 1));
            break;
        }
        float deg = ENC_TicksToDeg(ticks);
        printf("CALIB axis %d : ticks = %u (%.2f deg)\r\n",
               (int)(current_axis + 1),
               (unsigned)ticks,
               deg);
        break;
    }

    case 'x':
    case 'X':
    {
        // NEW: set zero_offset จริง
        SteerAxis_t *ax = &steer_axes[current_axis];
        uint16_t ticks = ENC_ReadRaw_ByIndex(ax->enc_index);
        if (ticks == 0xFFFF) {
            printf("Axis %d: ENC ERROR\r\n", (int)(current_axis + 1));
            break;
        }
        ax->zero_offset  = ticks;
        ax->target_ticks = ticks;
        printf("Axis %d: SET ZERO -> %u (%.2f deg)\r\n",
               (int)(current_axis + 1),
               (unsigned)ticks,
               ENC_TicksToDeg(ticks));
        break;
    }

    case 'Z':
        Steer_PrintCurrentZeroConfig();
        break;

    case 'c':
    case 'C':
        g_steer_mode = RUN_MODE_STEER_CALIB;
        printf("\r\n[MODE] Stay in STEER CALIB mode\r\n");
        Steer_PrintModeHelp(g_steer_mode);
        break;

    case 'p':
    case 'P':
        g_steer_mode = RUN_MODE_STEER_PID;
        printf("\r\n[MODE] Switch to STEER PID mode\r\n");
        Steer_PrintModeHelp(g_steer_mode);
        break;

    case 'k':
    case 'K':
        // จับค่าปัจจุบันเป็น zero_offset ใหม่ (ไม่ต้อง rebuild)
        Steer_CalibCommitZeroAll();
        break;

    case 't':
    case 'T':
        g_steer_log_enable = (uint8_t)!g_steer_log_enable;
        printf("STEER LOG: %s (period=%lu ms)\r\n",
               g_steer_log_enable ? "ON" : "OFF",
               (unsigned long)g_steer_log_period_ms);
        break;

    default:
        break;
    }
}

/* เดิม: wrapper เผื่อใช้แบบเก่า */
void Steer_JogCalib_HandleUart(void)
{
    uint8_t ch;
    if (HAL_UART_Receive(&huart3, &ch, 1, 0) != HAL_OK) {
        return;
    }
    Steer_JogCalib_HandleChar(ch);
}
