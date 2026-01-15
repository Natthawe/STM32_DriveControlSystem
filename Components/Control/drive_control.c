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
#define DRIVE_TPS_RAMP_UP_PER_SEC    	1000.0f   	// target_tps (tick/sec) -> accel
#define DRIVE_TPS_RAMP_DOWN_PER_SEC  	2000.0f   	// target_tps (tick/sec) -> decel
#define DRIVE_DEADBAND_TPS        		100.0f     	// ใกล้ 0 แค่ไหนถือว่า "หยุด"

#define DRIVE_HILL_KI_GAIN        		22.0f   	// คูณ Ki ตอนอยู่บนเนิน
#define DRIVE_HILL_DUTY_MAX       		1.0f    	// duty สูงสุดที่ยอมให้ค้ำเนิน
#define DRIVE_HILL_MIN_SPEED_TPS  		1.0f    	// ความเร็วที่ถือว่า "แทบจะหยุด" ใช้ตัด noise
#define DRIVE_HILL_BRAKE_TPS      		1000.0f   	// tps เหนือกว่านี้ถือว่า "ยังวิ่งเร็ว" -> โซนเบรก
#define DRIVE_HILL_HOLD_TPS       		500.0f   	// tps ต่ำกว่านี้ถือว่า "เกือบหยุด" -> โซน hold

#define DRIVE_HILL_MIN_DUTY       		0.35f    	// duty hill hold
#define DRIVE_HILL_MIN_DUTY_THRESH 		0.1f  		// ถ้า duty จาก PID เล็กกว่านี้จะไม่ boost

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
      { .kp=0.0005f, .ki=0.000015f, .kd=0.0f,
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

// รีเซ็ตสถานะ PID ของล้อขับทุกล้อ (แต่ไม่ไปยุ่ง target_tps)
void Drive_ResetPIDAll(void)
{
    for (uint32_t i = 0; i < DRIVE_NUM; ++i) {
        DriveAxis_t *d = &drive_axes[i];
        d->pid.integrator = 0.0f;
        d->pid.prev_error = 0.0f;
    }
}

// สั่งเบรคมอเตอร์ drive ทุกล้อ (ใช้ตอนกำลังเลี้ยว)
void Drive_BrakeAll(void)
{
    for (uint32_t i = 0; i < DRIVE_NUM; ++i) {
        DriveAxis_t *d = &drive_axes[i];
        Motor_set(d->motor_idx, MOTOR_DIR_BRAKE, 0.0f);
        d->last_duty = 0.0f;
    }
}

// ตั้ง target_tps สำหรับโหมด SPIN-IN-PLACE
// spin_dir = +1 หมุนทิศหนึ่ง, -1 หมุนทิศกลับกัน (มาจาก sign ของ angular.z)
void Drive_SetSpinTargets(float base_tps, float spin_dir)
{
    // ป้องกัน sign แปลก ๆ
    if (spin_dir >  1.0f) spin_dir =  1.0f;
    if (spin_dir < -1.0f) spin_dir = -1.0f;

    // index: 0=FR, 1=RR, 2=RL, 3=FL
    // เลือก pattern: ขวาเดินหน้า, ซ้ายถอยหลัง (เมื่อ spin_dir = +1)
    drive_axes[0].target_tps = +base_tps * spin_dir;  // FR
    drive_axes[1].target_tps = +base_tps * spin_dir;  // RR
    drive_axes[2].target_tps = -base_tps * spin_dir;  // RL
    drive_axes[3].target_tps = -base_tps * spin_dir;  // FL
}



void Drive_UpdateTargetsWithRamp(float dt_s,
                                 float *cmd_target_tps,
                                 float *current_target_tps)
{
    if (dt_s <= 0.0f) dt_s = 0.001f;

    // 1) กรณี "ปล่อยคันเร่ง" -> คำสั่งใกล้ 0
    if (fabsf(*cmd_target_tps) < DRIVE_DEADBAND_TPS)
    {
        // สั่งหยุดทันที
        *cmd_target_tps     = 0.0f;
        *current_target_tps = 0.0f;

        // กระจาย target = 0 tps ให้ทุกล้อ
        for (uint32_t i = 0; i < DRIVE_NUM; ++i) {
            drive_axes[i].target_tps = 0.0f;
        }

        return;
    }

    // 2) กรณียังมีคำสั่งวิ่ง -> ใช้ ramp ปกติ
    float delta = *cmd_target_tps - *current_target_tps;

    if (delta > 0.0f) {
        float step = DRIVE_TPS_RAMP_UP_PER_SEC * dt_s;
        if (delta > step) delta = step;
    } else if (delta < 0.0f) {
        float step = DRIVE_TPS_RAMP_DOWN_PER_SEC * dt_s;
        if (-delta > step) delta = -step;
    }

    *current_target_tps += delta;

    if (fabsf(*current_target_tps) < DRIVE_DEADBAND_TPS &&
        fabsf(*cmd_target_tps)     < DRIVE_DEADBAND_TPS)
    {
        *current_target_tps = 0.0f;
    }

    // กระจาย target_tps ให้ทุกล้อ (ใช้ common target)
    for (uint32_t i = 0; i < DRIVE_NUM; ++i) {
        drive_axes[i].target_tps = *current_target_tps;
    }
}

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
        d->last_ticks      = now_ticks;

        float meas_tps = diff_ticks / dt_s;
        d->meas_tps = meas_tps;   // เก็บไว้ debug

        float abs_tgt  = fabsf(d->target_tps);
        float abs_meas = fabsf(meas_tps);

        // ---------- จัดโซนการทำงาน ----------
        bool near_zero_target = (abs_tgt < DRIVE_DEADBAND_TPS);

        // โซน hold: target ใกล้ 0 และความเร็วต่ำมาก (เข้าโซนนี้จะเริ่มอัด I-Term)
        bool is_hill_hold = (near_zero_target && abs_meas <= DRIVE_HILL_HOLD_TPS);

        // โซนเบรก: target ใกล้ 0 แต่ยังวิ่งเร็ว
        bool is_braking = (near_zero_target && abs_meas > DRIVE_HILL_HOLD_TPS);

        // ตัด Noise: ถ้า target ใกล้ 0 และวัดได้เบามาก -> treat = 0
        if (near_zero_target && abs_meas < DRIVE_HILL_MIN_SPEED_TPS) {
            meas_tps = 0.0f;
            abs_meas = 0.0f;
        }

        // ถ้าเพิ่งปล่อยคันเร่งแล้วยังวิ่งเร็วมาก (เกินเพดานเบรค) -> reset integrator ป้องกัน I-windup ผิดทาง
        if (is_braking && abs_meas > DRIVE_HILL_BRAKE_TPS) {
            d->pid.integrator = 0.0f;
            d->pid.prev_error = 0.0f;
        }

        float error = d->target_tps - meas_tps;

        // ---------- คำนวณ PID (พร้อม boost Ki เฉพาะตอน hold) ----------
        float ki_backup = d->pid.ki;

        if (is_hill_hold) {
            // เพิ่ม Ki เฉพาะตอนถือเนินที่ "เข้าโซน Hold แล้ว"
            d->pid.ki = ki_backup * DRIVE_HILL_KI_GAIN;
        }

        float u = PID_Step(&d->pid, error, dt_s);

        // คืน Ki กลับปกติไม่ให้ไปกระทบตอนวิ่ง
        d->pid.ki = ki_backup;

        MotorDir_t dir;
        float duty;

        // แปลง PID Output เป็น Dir/Duty
        if (u >= 0.0f) {
            dir  = MOTOR_DIR_FWD;
            duty =  u;
        } else {
            dir  = MOTOR_DIR_REV;
            duty = -u;
        }

        // ---------- ใส่ base duty / limit duty ตามโซน ----------
        if (!near_zero_target) {
            // ============================================
            // CASE 1: RUN MODE (วิ่งปกติ มีเป้าความเร็ว)
            // ============================================
            duty = d->duty_base + duty;      // base + PID ตามเดิม
        }
        else {
            // ============================================
            // CASE 2: STOP / BRAKE / HILL-HOLD MODE
            // ============================================

            // เช็คทิศทางจริง: ถ้าค่าติดลบเยอะกว่า Noise แปลว่ากำลังไหลถอยหลัง
            bool is_moving_reverse = (meas_tps < -10.0f);

            // ตรวจสอบว่าอยู่ในสถานะที่ต้องออกแรงต้านหรือไม่
			if (is_braking || is_hill_hold) {

				// --- TUNE START: Friction Compensation ---
				float friction_compensate = 0.0f;

				// 1. ลดค่าลงให้เหมาะสม (TUNE POINTS)
				float comp_reverse = 0.35f;
				float comp_forward = 0.35f;

				// ตรวจสอบทิศทางการไหลเพื่อเลือกแรงต้าน
				if (is_moving_reverse) {
					 friction_compensate = comp_reverse;
				} else {
					 friction_compensate = comp_forward;
				}

				// 2. เพิ่ม Deadzone: ถ้าหยุดนิ่งสนิทจริงๆ อย่าเพิ่งกระชาก
				// ถ้าความเร็วต่ำกว่า 3 tps ถือว่านิ่งพอแล้ว ไม่ต้องอัด Min Duty
				if (abs_meas < 1.0f) {
					friction_compensate = 0.0f;
				}

				// Boost Duty: ถ้า PID สั่งมาน้อยกว่าแรงต้านขั้นต่ำ ให้ดันขึ้นไป
				if (duty < friction_compensate) {
					duty = friction_compensate;
				}
				// --- TUNE END ---

				// Safety
				if (duty > DRIVE_HILL_DUTY_MAX) {
					duty = DRIVE_HILL_DUTY_MAX;
				}
			}
        }

        // Final Clamp (ป้องกันค่าเกินช่วง PWM)
        if (duty > 1.0f) duty = 1.0f;
        if (duty < 0.0f) duty = 0.0f;

        d->last_duty = duty;
        Motor_set(d->motor_idx, dir, duty);

        /* //debug
        if (debug_cnt % 20 == 0) {
            printf("[%s] tgt=%.0f, meas=%.0f, err=%.0f, duty=%.2f, "
                   "near0=%d, brake=%d, hold=%d\r\n",
                   d->name,
                   d->target_tps,
                   d->meas_tps,
                   error,
                   d->last_duty,
                   (int)near_zero_target,
                   (int)is_braking,
                   (int)is_hill_hold);
        }
        */
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
