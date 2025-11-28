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

#define STEER_JOG_DUTY_DEFAULT  0.7f     	// duty เวลา jog
#define STEER_JOG_MS_DEFAULT    100U     	// หมุนครั้งละ x ms

#define STEER_TICKS_MAX      	1024U
#define ENC_TICKS_PER_REV    	1024.0f
#define ENC_TICKS_PER_DEG    	(ENC_TICKS_PER_REV / 360.0f)  // ≈ 2.8444 tick/deg
#define STEER_RAMP_DEG_PER_SEC  90.0f    // deg/s เปลี่ยนให้เร็วขึ้น เพราะถ้าช้าล้อมันจะสั่น

RunMode_t g_steer_mode = RUN_MODE_STEER_PID;

float g_cmd_target_deg     = 0.0f;   // มุมที่ ROS/serial สั่งมา
float g_current_target_deg = 0.0f;   // มุมเป้าหมายที่ ramp แล้ว [deg]

// map encoder index -> motor index 2,4,6,8
const uint8_t steer_motor_ids[STEER_NUM] = { 2, 4, 6, 8 };

SteerAxis_t steer_axes[] = {
	//  name, enc_idx, zero,  motor, enc_dir target, duty_base, PID
	{ "STEER_FR", 0, 489, 2, -1, 489, 0.67f,
//	  { .kp=0.01f, .ki=0.02f, .kd=0.0f,
	  { .kp=0.007f, .ki=0.001f, .kd=0.0f,
		.integrator=0, .prev_error=0,
		.out_min=-1.0f, .out_max=1.0f } },	// ENC2 -> Motor2 (front-right steer)

	{ "STEER_RR", 1, 641, 4, -1, 641,  0.66f,
//	  { .kp=0.006f, .ki=0.011f, .kd=0.0f,
	  { .kp=0.006f, .ki=0.0f, .kd=0.0f,
		.integrator=0, .prev_error=0,
		.out_min=-1.0f, .out_max=1.0f } },	// ENC3 -> Motor4 (rear-right steer)

	{ "STEER_RL", 2,  33, 6, -1,  33,  0.65f,
//	  { .kp=0.006f, .ki=0.01f, .kd=0.0f,
	  { .kp=0.0055f, .ki=0.001f, .kd=0.0f,
		.integrator=0, .prev_error=0,
		.out_min=-1.0f, .out_max=1.0f } },	// ENC4 -> Motor6 (rear-left steer)

	{ "STEER_FL", 3,  5,  8,  -1,  5,  0.7f,
//	  { .kp=0.0074f, .ki=0.02f, .kd=0.0f,
	  { .kp=0.007f, .ki=0.0f, .kd=0.0f,
		.integrator=0, .prev_error=0,
		.out_min=-1.0f, .out_max=1.0f } },	// ENC5 -> Motor8 (front-left steer)
};

// +1 = เวลาสั่งเลี้ยวมุม +deg ให้หมุนไปทางเดียวกับ angle_deg
// -1 = เวลาสั่งเลี้ยวมุม +deg ให้หมุนไปทางตรงข้าม (สำหรับล้อหลังใน all-wheel-steer)
const int8_t steer_turn_sign[STEER_NUM] = {
    +1,  // 0: STEER_FR (front-right)
    -1,  // 1: STEER_RR (rear-right)
    -1,  // 2: STEER_RL (rear-left)
    +1   // 3: STEER_FL (front-left)
};

uint16_t wrap_ticks(int32_t t)
{
    while (t < 0)                 			t += STEER_TICKS_MAX;
    while (t >= (int32_t)STEER_TICKS_MAX)	t -= STEER_TICKS_MAX;
    return (uint16_t)t;
}

// คำนวณ target_ticks ของทุกล้อตามมุม angle_deg (ใช้ zero_offset ของแต่ละล้อ) (+ ซ้าย, - ขวา) หมุนเหมือนกันทุกล้อ
void Steer_SetTargetAngleDeg2(float angle_deg)
{
    // clamp มุมไม่เกิน ±40°
    if (angle_deg >  STEER_MAX_DEG) angle_deg =  STEER_MAX_DEG;
    if (angle_deg < -STEER_MAX_DEG) angle_deg = -STEER_MAX_DEG;

    float delta_ticks_f = angle_deg * ENC_TICKS_PER_DEG;

    for (uint32_t i = 0; i < STEER_NUM; ++i) {
        int32_t base  = (int32_t)steer_axes[i].zero_offset;
        int32_t delta = (int32_t)lrintf(delta_ticks_f);
        steer_axes[i].target_ticks = wrap_ticks(base + delta);
    }
}

// คำนวณ target_ticks ของทุกล้อตามมุม angle_deg (ใช้ zero_offset ของแต่ละล้อ) (AWS: ล้อหน้า+, ล้อหลัง-)
void Steer_SetTargetAngleDeg(float angle_deg)
{
    // clamp มุมไม่เกิน ±STEER_MAX_DEG
    if (angle_deg >  STEER_MAX_DEG) angle_deg =  STEER_MAX_DEG;
    if (angle_deg < -STEER_MAX_DEG) angle_deg = -STEER_MAX_DEG;

    for (uint32_t i = 0; i < STEER_NUM; ++i) {
        // AWS:
        //  - ล้อหน้า: steer_turn_sign = +1  -> ได้มุม +angle_deg
        //  - ล้อหลัง: steer_turn_sign = -1  -> ได้มุม -angle_deg
        float delta_ticks_f = angle_deg * (float)steer_turn_sign[i] * ENC_TICKS_PER_DEG;

        int32_t base  = (int32_t)steer_axes[i].zero_offset;
        int32_t delta = (int32_t)lrintf(delta_ticks_f);

        steer_axes[i].target_ticks = wrap_ticks(base + delta);
    }
}

void SteerMotor_Jog(uint8_t axis, MotorDir_t dir, float duty)
{
    if (axis >= 4) return;
    uint8_t motor_idx = steer_motor_ids[axis];
    Motor_set(motor_idx, dir, duty);
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
    // clamp อีกครั้งเผื่อ caller ส่งเกิน
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

    // ต่างของมุม (deg)
    float delta = g_cmd_target_deg - g_current_target_deg;

    // limit การเปลี่ยนแปลงต่อรอบ (deg)
    float max_step = STEER_RAMP_DEG_PER_SEC * dt_s;

    if (delta >  max_step) delta =  max_step;
    if (delta < -max_step) delta = -max_step;

    g_current_target_deg += delta;

    // deadzone เล็ก ๆ รอบ 0 กันค้างค่าจิ๋ว ๆ
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

//        float abs_err = fabsf(err_f);
//        float duty    = duty_raw;
//
//        if (steering_ramping || (abs_err > (2.0f * STEER_DEADBAND_TICKS))) {
//            // ยังหมุนไปเป้าหมายอยู่ หรือ error ยังห่างมาก
//            // → ใช้ base duty ช่วยให้หมุนแน่นอน
//            if (duty < ax->duty_base) {
//                duty = ax->duty_base;
//            }
//        } else {
//            // เข้าใกล้เป้าหมายแล้ว (error ไม่ใหญ่มาก)
//            // → ยอมให้ duty เล็ก ๆ ได้ เพื่อลดอาการสั่น
//            if (duty < 0.0f) {
//                duty = 0.0f;   // จริง ๆ duty_raw เป็นบวกอยู่แล้ว แต่กันเผื่อ
//            }
//        }
//
//        if (duty > 1.0f) {
//            duty = 1.0f;
//        }

        ax->last_duty = duty;
        Motor_set(ax->motor_idx, dir, duty);

        // debug ถ้าอยากดู
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
        printf("  Z   : print all axis zeros (steer_axes[] config)\r\n");
        printf("  p   : switch to PID mode\r\n");
    } else {
        printf(" RUN MODE: STEER PID (from ROS /cmd_vel)\r\n");
        printf("  - Drive: from serial/UDP Twist\r\n");
        printf("  - Steer: PID to target angle (max 40 deg)\r\n");
        printf("  c : switch to CALIB mode\r\n");
    }
    printf("=============================\r\n");
}

void Steer_JogCalib_HandleUart(void)
{
    static uint8_t current_axis = 0;              // 0..3 -> STEER1..4

    const float    jog_duty = STEER_JOG_DUTY_DEFAULT;
    const uint32_t JOG_MS   = STEER_JOG_MS_DEFAULT;

    uint8_t ch;

    if (HAL_UART_Receive(&huart3, &ch, 1, 10) != HAL_OK) {
        return;
    }

    switch (ch)
    {
    case '1':
    case '2':
    case '3':
    case '4':
        current_axis = (uint8_t)(ch - '1');  // '1'->0, '2'->1 ...
        printf("Selected STEER axis %d\r\n", (int)(current_axis + 1));
        break;

    case 'a':
    case 'A':
        SteerMotor_Jog(current_axis, MOTOR_DIR_REV, jog_duty);
        HAL_Delay(JOG_MS);
        SteerMotor_Jog(current_axis, MOTOR_DIR_BRAKE, 0.0f);
        printf("Axis %d: JOG LEFT  (%.0f ms, duty=%.2f)\r\n",
               (int)(current_axis + 1),
               (float)JOG_MS,
               jog_duty);
        break;

    case 'd':
    case 'D':
        SteerMotor_Jog(current_axis, MOTOR_DIR_FWD, jog_duty);
        HAL_Delay(JOG_MS);
        SteerMotor_Jog(current_axis, MOTOR_DIR_BRAKE, 0.0f);
        printf("Axis %d: JOG RIGHT (%.0f ms, duty=%.2f)\r\n",
               (int)(current_axis + 1),
               (float)JOG_MS,
               jog_duty);
        break;

    case 's':
    case 'S':
        SteerMotor_Jog(current_axis, MOTOR_DIR_BRAKE, 0.0f);
        printf("Axis %d: STOP\r\n", (int)(current_axis + 1));
        break;

    case 'z':
    case 'Z':
    {
        if (ch == 'z') {
            		// อ่านเฉพาะแกนปัจจุบัน (ตาม current_axis)
            uint16_t ticks = ENC_ReadRaw_ByIndex(current_axis);
            float    deg   = ENC_TicksToDeg(ticks);
            printf("CALIB axis %d : zero_ticks = %u (%.2f deg)\r\n",
                   (int)(current_axis + 1),
                   (unsigned)ticks,
                   deg);
        } else {
            // 'Z' = อ่านทุกแกนแล้วพิมพ์โค้ด steer_axes[] ครบทั้ง 4 ล้อ
            Steer_PrintCurrentZeroConfig();
        }
//        uint16_t ticks = ENC_ReadRaw_ByIndex(current_axis);
//        float    deg   = ENC_TicksToDeg(ticks);
//        printf("CALIB axis %d : zero_ticks = %u (%.2f deg)\r\n",
//               (int)(current_axis + 1),
//               (unsigned)ticks,
//               deg);
        break;
    }

    // ---- สลับโหมดกลางทางจาก CALIB -> PID / CALIB เอง ----
    case 'c':
    case 'C':
        g_steer_mode = RUN_MODE_STEER_CALIB;
        printf("\r\n[MODE] Stay in CALIB mode\r\n");
        Steer_PrintModeHelp(g_steer_mode);
        break;

    case 'p':
    case 'P':
        g_steer_mode = RUN_MODE_STEER_PID;
        printf("\r\n[MODE] Switch to PID mode\r\n");
        Steer_PrintModeHelp(g_steer_mode);
        break;

    default:
        break;
    }
}
