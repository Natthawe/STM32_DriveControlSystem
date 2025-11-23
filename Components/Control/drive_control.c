/*
 * drive_control.c
 *
 *  Created on: Nov 22, 2025
 *      Author: cg
 */

#include "Control/drive_control.h"
#include <math.h>
#include <stdio.h>

// ramp ของเป้าความเร็วในหน่วย tps
#define DRIVE_TPS_RAMP_UP_PER_SEC    	2000.0f   // เพิ่มเป้า tps ต่อวินาที
#define DRIVE_TPS_RAMP_DOWN_PER_SEC  	4000.0f   // ลดเป้า tps ต่อวินาที
#define DRIVE_DEADBAND_TPS        		20.0f     // ใกล้ 0 แค่ไหนถือว่า "หยุด"

// map: drive_enc[0..3] -> motor 1,3,5,7
DriveAxis_t drive_axes[DRIVE_NUM] = {
    // motor1: Front Right drive
    { "DRV_FR", 1, &drive_enc[0],
      { .kp=0.0006f, .ki=0.000015f, .kd=0.0f,
        .integrator=0, .prev_error=0,
        .out_min=-1.0f, .out_max=1.0f },
      0.0f, 0, 1810.0f, 0.2f, 0.0f, 0.0f },

    // motor3: Rear Right drive
    { "DRV_RR", 3, &drive_enc[1],
      { .kp=0.0005f, .ki=0.00002f, .kd=0.0f,
        .integrator=0, .prev_error=0,
        .out_min=-1.0f, .out_max=1.0f },
      0.0f, 0, 1880.0f, 0.32f, 0.0f, 0.0f },

    // motor5: Rear Left drive
    { "DRV_RL", 5, &drive_enc[2],
      { .kp=0.0006f, .ki=0.000015f, .kd=0.0f,
        .integrator=0, .prev_error=0,
        .out_min=-1.0f, .out_max=1.0f },
      0.0f, 0, 1960.0f, 0.34f, 0.0f, 0.0f },

    // motor7: Front Left drive
    { "DRV_FL", 7, &drive_enc[3],
      { .kp=0.00055f, .ki=0.000001f, .kd=0.0f,
        .integrator=0, .prev_error=0,
        .out_min=-1.0f, .out_max=1.0f },
      0.0f, 0, 1915.0f, 0.45f, 0.0f, 0.0f },
};

DriveMode_t g_drive_mode = RUN_MODE_DRIVE_PID;

int8_t g_test_drive_idx = 0;		// (-1 = ทุกล้อ, 0..3 = DRV1..4)
float  g_test_duty 		= 0.60f;	// duty ที่ใช้เทส (0.0 .. 1.0)
int8_t g_test_dir 		= 1;		// dir 1 = forward, -1 = reverse

// เริ่มต้นค่า drive PID
void Drive_InitAll(void)
{
    for (uint32_t i = 0; i < DRIVE_NUM; ++i) {
        DriveAxis_t *d = &drive_axes[i];
        d->target_tps = 0.0f;
        d->last_ticks = d->wheel->multi_ticks;
        d->pid.integrator = 0.0f;
        d->pid.prev_error = 0.0f;
    }
}

// reset PID และเบรค drive ทั้งหมด (ใช้ตอน STOP / timeout)
void Drive_StopAll(void)
{
    for (uint32_t i = 0; i < DRIVE_NUM; ++i) {
        DriveAxis_t *d = &drive_axes[i];
        d->target_tps = 0.0f;
        d->pid.integrator = 0.0f;
        d->pid.prev_error = 0.0f;
        Motor_set(d->motor_idx, MOTOR_DIR_BRAKE, 0.0f);
    }
}

// อัปเดต target_tps ของล้อ drive ทั้งหมด จาก g_cmd_target_tps โดยค่อย ๆ ramp ทีละนิด (หน่วย tps) จาก g_current_speed_norm -> g_cmd_speed_norm
void Drive_UpdateTargetsWithRamp(float dt_s, float *cmd_target_tps, float *current_target_tps)
{
    if (dt_s <= 0.0f) dt_s = 0.001f;

    // ถ้าเป้าและค่าปัจจุบัน “ใกล้ 0” มาก ๆ → หยุด แล้ว reset PID + เบรค
    if (fabsf(g_cmd_target_tps)     < DRIVE_DEADBAND_TPS &&
        fabsf(g_current_target_tps) < DRIVE_DEADBAND_TPS)
    {
        g_cmd_target_tps     = 0.0f;
        g_current_target_tps = 0.0f;

        for (uint32_t i = 0; i < DRIVE_NUM; ++i) {
            DriveAxis_t *d = &drive_axes[i];
            d->target_tps      = 0.0f;
            d->pid.integrator  = 0.0f;
            d->pid.prev_error  = 0.0f;
            Motor_set(d->motor_idx, MOTOR_DIR_BRAKE, 0.0f);
        }
        return;
    }

    // ---- ramp g_current_target_tps เข้าเป้า g_cmd_target_tps ----
    float delta = g_cmd_target_tps - g_current_target_tps;

    if (delta > 0.0f) {
        float step = DRIVE_TPS_RAMP_UP_PER_SEC * dt_s;
        if (delta > step) delta = step;
    } else if (delta < 0.0f) {
        float step = DRIVE_TPS_RAMP_DOWN_PER_SEC * dt_s;
        if (-delta > step) delta = -step;
    }

    g_current_target_tps += delta;

    // deadzone รอบ ๆ 0 กันแกว่ง
    if (fabsf(g_current_target_tps) < DRIVE_DEADBAND_TPS) {
        g_current_target_tps = 0.0f;
    }

    // ---- set target_tps ให้ทุกล้อ = ค่าเดียวกัน (target_tps_common) ----
    for (uint32_t i = 0; i < DRIVE_NUM; ++i) {
        drive_axes[i].target_tps = g_current_target_tps;
    }
}

// อัพเดต PID ความเร็วล้อ drive ทั้งหมด
void Drive_UpdateAll(float dt_s)
{
    if (dt_s <= 0.0f) dt_s = 1e-3f;

    static uint32_t debug_cnt = 0;
    debug_cnt++;

    for (uint32_t i = 0; i < DRIVE_NUM; ++i) {
        DriveAxis_t *d = &drive_axes[i];
        DriveEnc_t  *w = d->wheel;

        int32_t now_ticks  = w->multi_ticks;
        int32_t diff_ticks = now_ticks - d->last_ticks;
        d->last_ticks = now_ticks;

        float meas_tps = diff_ticks / dt_s;
        d->meas_tps = meas_tps;   // <-- เก็บไว้ใน struct

        // ถ้า target และ measured ใกล้ 0 มาก → หยุด + reset PID
        if (fabsf(d->target_tps) < DRIVE_DEADBAND_TPS &&
            fabsf(meas_tps)     < DRIVE_DEADBAND_TPS)
        {
            d->pid.integrator = 0.0f;
            d->pid.prev_error = 0.0f;
            d->last_duty      = 0.0f;

            Motor_set(d->motor_idx, MOTOR_DIR_BRAKE, 0.0f);
            continue;
        }

        float error = d->target_tps - meas_tps;

        // ใช้ error → u (สเกล -1..+1 ตาม out_min/out_max ของ PID)
        float u = PID_Step(&d->pid, error, dt_s);

        MotorDir_t dir;
        float duty;

        if (u >= 0.0f) {
            dir  = MOTOR_DIR_FWD;
            duty = u;
        } else {
            dir  = MOTOR_DIR_REV;
            duty = -u;
        }

        // *** จุดสำคัญ: duty = base + สัดส่วนจาก PID ***
        if (d->target_tps != 0.0f) {
            duty = d->duty_base + duty;   // base ต่อ-ล้อ + สัดส่วนจาก PID
        } else {
            duty = 0.0f;                  // target = 0 -> ไม่ต้องใส่ base
        }

        // clamp
        if (duty > 1.0f) duty = 1.0f;
        if (duty < 0.0f) duty = 0.0f;

        d->last_duty = duty;              // <-- เก็บ duty ล่าสุด
        Motor_set(d->motor_idx, dir, duty);

        // debug
//        if (debug_cnt % 10 == 0) {
//            printf("[%s] tgt=%.0f tps, meas=%.0f tps, l_duty=%.2f (dt_base=%.2f) dir=%d\r\n",
//                   d->name,
//                   d->target_tps,
//                   d->meas_tps,
//                   d->last_duty,
//                   d->duty_base,
//                   (int)dir);
//        }
    }
}

void Process_UART_TestDrive(void)
{
    uint8_t ch;
    if (HAL_UART_Receive(&huart3, &ch, 1, 0) != HAL_OK) {
        return;
    }

    switch (ch) {

    // --- เลือกโหมด ---
    case 'p':   // PID
    case 'P':
        g_drive_mode = RUN_MODE_DRIVE_PID;
        printf("DRIVE MODE: NORMAL\r\n");
        g_cmd_dir_sign        = 0.0f;
        g_cmd_speed_norm      = 0.0f;
        g_current_speed_norm  = 0.0f;
        Drive_StopAll();
        break;

    case 'c':   // CALIB
    case 'C':
        g_drive_mode = RUN_MODE_DRIVE_CALIB;
        printf("DRIVE MODE: TEST\r\n");
        break;

    // --- เลือกล้อ 1..4 ---
    case '1':
    case '2':
    case '3':
    case '4':
        g_test_drive_idx = (int8_t)(ch - '1');   // '1'->0, '2'->1 ...
        printf("TEST DRIVE: wheel %d selected\r\n", (int)(g_test_drive_idx + 1));
        break;
    case '0':   // 0 = ALL wheels
        g_test_drive_idx = -1;
        printf("TEST DRIVE: ALL wheels selected\r\n");
        break;
    // --- ทิศทาง ---
    case 'f':   // forward
    case 'F':
        g_test_dir = 1;
        printf("TEST DRIVE dir = FORWARD\r\n");
        break;

    case 'r':   // reverse
    case 'R':
        g_test_dir = -1;
        printf("TEST DRIVE dir = REVERSE\r\n");
        break;

    // --- ปรับ duty ---
    case '+':
    case '=':
        g_test_duty += 0.05f;
        if (g_test_duty > 1.0f) g_test_duty = 1.0f;
        printf("TEST DRIVE duty = %.2f\r\n", g_test_duty);
        break;

    case '-':
    case '_':
        g_test_duty -= 0.05f;
        if (g_test_duty < 0.0f) g_test_duty = 0.0f;
        printf("TEST DRIVE duty = %.2f\r\n", g_test_duty);
        break;

    // --- stop ทั้งหมด ---
    case 's':
    case 'S':
        g_test_duty = 0.0f;
        printf("TEST DRIVE STOP (duty=0)\r\n");
        break;

    default:
        break;
    }
}
