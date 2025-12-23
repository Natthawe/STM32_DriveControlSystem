/*
 * robot.c
 *
 *  Created on: Nov 22, 2025
 *      Author: cg
 */

#include "Robot/robot.h"
#include "Motors/motor.h"
#include "Control/steer_control.h"
#include "Control/drive_control.h"
#include "main.h"
#include <math.h>
#include <stdio.h>
#include <stdbool.h>

// === CONFIG สำหรับ SPIN-IN-PLACE ===
#define ROBOT_SPIN_LIN_EPS        0.02f   // ถ้า |linear| < 2 cm/s ถือว่าเป็น 0 (**not use)
#define ROBOT_SPIN_ANG_EPS        0.05f   // ถ้า |angular| < 0.05 rad/s (~3 deg/s) ถือว่าไม่หมุน (**not use)
#define ROBOT_SPIN_STEER_DEG      45.0f   // มุมหักล้อสำหรับ spin (deg)
#define ROBOT_SPIN_TPS_PER_RAD    2000.0f // map |angular.z| -> tps (0.7 * 2000 = 1400 tps -> limit ด้วย ROBOT_SPIN_TPS_MAX) (**not use)
#define ROBOT_SPIN_TPS_MAX        1200.0f // limit tps สำหรับ spin

// ====== SPIN MODE STATE (ควบคุมด้วย Int8) ======
volatile SpinMode_t g_spin_mode    	= SPIN_MODE_OFF;
volatile float      g_spin_cmd_deg 	= 0.0f;				// มุมที่ใช้ตอน SPIN (ปกติ = 45°)
volatile float      g_spin_base_tps = 0.0f;				// ความเร็วพื้นฐานของล้อขับเวลา SPIN
volatile float      g_spin_dir      = 0.0f;				// ทิศการหมุน (+1 หรือ -1)

// ==== Geometry ล้อ ====
#define WHEEL_DIAMETER_M       0.37f
#define WHEEL_COUNTS_PER_REV_F 1440.0f
#define WHEEL_CIRCUM_M         (3.1415926f * WHEEL_DIAMETER_M)
#define TICKS_PER_METER        (WHEEL_COUNTS_PER_REV_F / WHEEL_CIRCUM_M)

// reference สำหรับ scale linear.x -> speed
#define ROBOT_SPIN_LIN_REF   1.0f   // ถ้า linear.x ~ 1.0 = หมุนเร็วสุด

// ~ 1440 / (pi*0.37) ≈ 1239 ticks/m

// ====== map จาก linear.x / angular.z ======
#define MAX_CMD_LINEAR   1.0f   // [m/s] linear.x สูงสุดที่ส่งมา
#define MAX_CMD_ANGULAR  0.5f   // [rad/s] angular.z สูงสุดที่ส่งมา
#define LIN_DEADZONE     0.02f  // deadzone สำหรับ linear (normalized)
#define ANG_DEADZONE     0.02f  // deadzone สำหรับ angular (normalized)

extern float g_cmd_dir_sign;     // -1,0,+1 จากคำสั่งล่าสุด
extern float g_cmd_speed_norm;   // 0..1 จาก drive_pct
extern float g_cmd_target_tps;   // target_tps_common จากคำสั่ง (ticks/sec)
extern uint32_t g_last_cmd_ms;   // timestamp คำสั่งล่าสุด (ms)

static bool g_cmd_active = false;  // ยังไม่เคยได้ cmd_vel -> false

// ====== Robot state ======
RobotCmd_t g_robot_cmd = ROBOT_CMD_STOP;

void Robot_ApplyCommand_WithDuty(RobotCmd_t cmd, float drive_duty, float steer_duty)
{
    // clamp duty
    if (drive_duty < 0.0f) drive_duty = 0.0f;
    if (drive_duty > 1.0f) drive_duty = 1.0f;
    if (steer_duty < 0.0f) steer_duty = 0.0f;
    if (steer_duty > 1.0f) steer_duty = 1.0f;

    g_robot_cmd = cmd;

    switch (cmd) {

    case ROBOT_CMD_FORWARD:
        Motors_Drive_All(MOTOR_DIR_FWD, drive_duty);
        Motors_Steer_All(MOTOR_DIR_BRAKE, 0.0f);
        break;

    case ROBOT_CMD_BACKWARD:
        Motors_Drive_All(MOTOR_DIR_REV, drive_duty);
        Motors_Steer_All(MOTOR_DIR_BRAKE, 0.0f);
        break;

    case ROBOT_CMD_TURN_LEFT:
        Motors_Drive_All(MOTOR_DIR_BRAKE, 0.0f);
        Motors_Steer_All(MOTOR_DIR_REV, steer_duty);
        break;

    case ROBOT_CMD_TURN_RIGHT:
        Motors_Drive_All(MOTOR_DIR_BRAKE, 0.0f);
        Motors_Steer_All(MOTOR_DIR_FWD, steer_duty);
        break;

    case ROBOT_CMD_FWD_LEFT:
        Motors_Drive_All(MOTOR_DIR_FWD, drive_duty);
        Motors_Steer_All(MOTOR_DIR_REV, steer_duty);
        break;

    case ROBOT_CMD_FWD_RIGHT:
        Motors_Drive_All(MOTOR_DIR_FWD, drive_duty);
        Motors_Steer_All(MOTOR_DIR_FWD, steer_duty);
        break;

    case ROBOT_CMD_BACK_LEFT:
        Motors_Drive_All(MOTOR_DIR_REV, drive_duty);
        Motors_Steer_All(MOTOR_DIR_REV, steer_duty);
        break;

    case ROBOT_CMD_BACK_RIGHT:
        Motors_Drive_All(MOTOR_DIR_REV, drive_duty);
        Motors_Steer_All(MOTOR_DIR_FWD, steer_duty);
        break;

    case ROBOT_CMD_STOP:
    default:
        Motors_Drive_All(MOTOR_DIR_BRAKE, 0.0f);
        Motors_Steer_All(MOTOR_DIR_BRAKE, 0.0f);
        break;
    }
}

RobotCmd_t Robot_CmdFromChar(uint8_t c)
{
    switch (c) {
    case 'i': case 'I': return ROBOT_CMD_FORWARD;
    case ',':           return ROBOT_CMD_BACKWARD;

    case 'j': case 'J': return ROBOT_CMD_TURN_LEFT;
    case 'l': case 'L': return ROBOT_CMD_TURN_RIGHT;
    case 'k': case 'K': return ROBOT_CMD_STOP;

    case 'u': case 'U': return ROBOT_CMD_FWD_LEFT;
    case 'o': case 'O': return ROBOT_CMD_FWD_RIGHT;

    case 'm': case 'M': return ROBOT_CMD_BACK_LEFT;
    case '.':           return ROBOT_CMD_BACK_RIGHT;

    case 'w': case 'W': return ROBOT_CMD_FORWARD;
    case 's': case 'S': return ROBOT_CMD_BACKWARD;
    case 'a': case 'A': return ROBOT_CMD_TURN_LEFT;
    case 'd': case 'D': return ROBOT_CMD_TURN_RIGHT;
    case 'x': case 'X': return ROBOT_CMD_STOP;

    default:
        return ROBOT_CMD_NONE;
    }
}

void Robot_ApplyTwist(float linear_x, float angular_z)
{
    // --- จำเวลาคำสั่งล่าสุด ---
    g_last_cmd_ms = HAL_GetTick();
    g_cmd_active  = true;

    // ====== 1) จัดการ linear.x -> target_tps ======

    // จำกัด linear_x ให้อยู่ในช่วงที่เรารับได้
    if (linear_x >  MAX_CMD_LINEAR) linear_x =  MAX_CMD_LINEAR;
    if (linear_x < -MAX_CMD_LINEAR) linear_x = -MAX_CMD_LINEAR;

    // ใช้ค่า normalized เพื่อง่ายต่อการทำ deadzone
    float lin_norm = linear_x / MAX_CMD_LINEAR;   // -1..+1
    if (lin_norm >  1.0f) lin_norm =  1.0f;
    if (lin_norm < -1.0f) lin_norm = -1.0f;

    if (fabsf(lin_norm) < LIN_DEADZONE) {
        lin_norm = 0.0f;
    }

    // linear_x หลัง deadzone แล้ว (m/s)
    float linear_x_cmd = lin_norm * MAX_CMD_LINEAR;

    // map เป็น target_tps_common ด้วย geometry
    float target_tps_cmd = linear_x_cmd * TICKS_PER_METER;

    // เก็บเป็นคำสั่งหลักในหน่วย tps
    g_cmd_target_tps = target_tps_cmd;

    // debug state
    if (lin_norm > 0.0f)      g_cmd_dir_sign = +1.0f;
    else if (lin_norm < 0.0f) g_cmd_dir_sign = -1.0f;
    else                      g_cmd_dir_sign = 0.0f;

    g_cmd_speed_norm = fabsf(lin_norm);   // 0..1

    // ====== 2) จัดการ angular.z -> steering angle ======

    // ใช้ 0.785 rad (~45°) เป็น input max
    const float MAX_STEER_RAD = 0.7853982f;   // pi/4

    // clamp angular_z ไม่ให้เกิน ±0.785 rad
    if (angular_z >  MAX_STEER_RAD) angular_z =  MAX_STEER_RAD;
    if (angular_z < -MAX_STEER_RAD) angular_z = -MAX_STEER_RAD;

    float abs_vx = fabsf(linear_x_cmd);
    float abs_wz = fabsf(angular_z);

#if 0
    // ====== 2.1) โหมด SPIN-IN-PLACE ======
    if (abs_vx < ROBOT_SPIN_LIN_EPS && abs_wz > ROBOT_SPIN_ANG_EPS) {

        // --- map angular.z -> มุมเลี้ยว (0..45°) ---
        // ใช้ 0.785 rad (~45°) เป็น input max
        const float MAX_STEER_RAD = 0.7853982f;   // pi/4

        // angular_z ถูก clamp ก่อนหน้านี้แล้วให้อยู่ในช่วง [-MAX_STEER_RAD, +MAX_STEER_RAD]
        float spin_norm = abs_wz / MAX_STEER_RAD;   // 0..1
        if (spin_norm > 1.0f) spin_norm = 1.0f;

        // มุมที่ "สั่ง" สำหรับ SPIN (0..STEER_SPIN_MAX_DEG)
        float spin_deg = spin_norm * STEER_SPIN_MAX_DEG;   // 0..45°
        g_spin_cmd_deg = spin_deg;                    // เก็บไว้ใช้เช็คใน drive_control

        // ตั้งแพทเทิร์นมุมล้อสำหรับ SPIN (FR/RR/RL/FL ซ้าย-ขวาสลับกัน)
        Steer_SetSpinAngleDeg(spin_deg);

        // --- map angular.z -> ความเร็วหมุนรอบตัว (tps) ---
        float base_tps = abs_wz * ROBOT_SPIN_TPS_PER_RAD;
        if (base_tps > ROBOT_SPIN_TPS_MAX) {
            base_tps = ROBOT_SPIN_TPS_MAX;
        }

        // ทิศหมุนตาม sign ของ angular.z
        float spin_dir = (angular_z >= 0.0f) ? 1.0f : -1.0f;

        // เปิด SPIN mode พร้อม speed/dir สำหรับล้อขับ (แต่ล้อขับจะถูก gating อีกทีใน drive_control)
        Robot_SetSpinParams(base_tps, spin_dir);

        return;  // ไม่เข้า logic เลี้ยวแบบวิ่งปกติ
    }
#endif

    // ====== 2.2) โหมดวิ่ง/เลี้ยวปกติ (AWS) ======
    // ออกจาก spin mode แน่นอน + reset มุม spin
    Robot_SetSpinParams(0.0f, 0.0f);
    g_spin_cmd_deg = 0.0f;

    // normalized angular ในช่วง -1..+1 ตาม input max 0.785 rad
    float ang_norm = angular_z / MAX_STEER_RAD;   // -1..+1

    if (ang_norm >  1.0f) ang_norm =  1.0f;
    if (ang_norm < -1.0f) ang_norm = -1.0f;

    if (fabsf(ang_norm) < ANG_DEADZONE) {
        ang_norm = 0.0f;
    }

    // map -> มุมเลี้ยว ±STEER_MAX_DEG (เช่น ±45°)
    float target_deg = ang_norm * STEER_MAX_DEG;

    // ถ้าวิ่งถอยหลัง ให้กลับมุมเลี้ยว (หน้า/หลังสลับกัน)
    if (linear_x_cmd < -0.000001f) {
        target_deg = -target_deg;
    }

    // ส่งเป้ามุมไปให้โมดูล steer ค่อย ๆ ramp เอง
    Steer_SetCmdTargetDeg(target_deg);

    // เคยได้รับคำสั่งแล้ว
    g_cmd_active = true;

    //    printf("Twist: lin=%.2f m/s (tgt=%.0f tps), ang=%.3f rad/s -> deg=%.1f\r\n",
    //           linear_x_cmd, target_tps_cmd, angular_z, target_deg);
}

bool Robot_IsSpinMode(void)
{
    return (g_spin_mode != SPIN_MODE_OFF);
}

void Robot_GetSpinParams(float *base_tps, float *dir)
{
    if (base_tps) *base_tps = g_spin_base_tps;
    if (dir)      *dir      = g_spin_dir;
}

void Robot_SetSpinParams(float base_tps, float dir)
{
    // ถ้า base_tps > 0 แปลว่าต้องการ spin mode
    if (base_tps > 0.0f) {
        g_spin_mode     = true;
        g_spin_base_tps = base_tps;
        g_spin_dir      = (dir >= 0.0f) ? 1.0f : -1.0f;
    } else {
        // base_tps == 0 -> ออกจาก spin mode
        g_spin_mode     = false;
        g_spin_base_tps = 0.0f;
        g_spin_dir      = 0.0f;
    }
}

// ====== SPIN MODE (Int8) IMPLEMENTATION ======

SpinMode_t Robot_GetSpinMode(void)
{
    return g_spin_mode;
}

void Robot_SetSpinMode(SpinMode_t mode)
{
    g_spin_mode = mode;
}

float Robot_GetSpinCmdDeg(void)
{
    return g_spin_cmd_deg;
}

void Robot_GetSpinDriveParams(float *base_tps, float *dir)
{
    if (base_tps) *base_tps = g_spin_base_tps;
    if (dir)      *dir      = g_spin_dir;
}

/**
 * เรียกเมื่อได้รับคำสั่ง SPIN จาก joystick/UDP/serial
 * cmd:
 *   +1 : เข้าโหมด SPIN -> หักล้อ 45° แล้วค้างไว้ (ล้อขับหยุด)
 *   -1 : ยกเลิก SPIN -> ล้อกลับ 0° และหยุดล้อขับ
 *   +2 : เริ่มหมุนล้อขับ (เมื่อมุม 45° แล้ว)
 *   -2 : หยุดหมุนล้อขับ แต่ยังคงมุม 45°
 */
void Robot_HandleSpinCommand(int8_t cmd)
{
    printf("[SPIN_CMD] cmd=%d\r\n", (int)cmd);

    switch (cmd) {

    case 1: // เข้าโหมด SPIN ซ้าย: หักล้อ +45° ค้าง
        g_spin_mode     = SPIN_MODE_ALIGN;
        g_spin_cmd_deg  = +ROBOT_SPIN_STEER_DEG;   // +45°
        g_spin_base_tps = ROBOT_SPIN_TPS_MAX;      // เช่น 1200 tps
        g_spin_dir      = +1.0f;                   // ทิศหมุน (+1)
        printf("[SPIN_CMD] ENTER ALIGN LEFT (deg=%.1f)\r\n", g_spin_cmd_deg);
        break;

    case 3: // เข้าโหมด SPIN ขวา: หักล้อ -45° ค้าง
        g_spin_mode     = SPIN_MODE_ALIGN;
        g_spin_cmd_deg  = -ROBOT_SPIN_STEER_DEG;   // -45°
        g_spin_base_tps = ROBOT_SPIN_TPS_MAX;
        g_spin_dir      = -1.0f;                   // ทิศหมุน (-1)
        printf("[SPIN_CMD] ENTER ALIGN RIGHT (deg=%.1f)\r\n", g_spin_cmd_deg);
        break;

    case -1: // ยกเลิก SPIN: กลับ 0°
        g_spin_mode     = SPIN_MODE_OFF;
        g_spin_cmd_deg  = 0.0f;
        g_spin_base_tps = 0.0f;
        g_spin_dir      = 0.0f;

        Steer_InitTargetsToZero();
        Drive_StopAll();
        printf("[SPIN_CMD] CANCEL -> OFF (back to 0 deg)\r\n");
        break;

    case 2: // เริ่มหมุนล้อขับ (ต้องอยู่ใน ALIGN หรือ READY อยู่แล้ว)
        if (g_spin_mode == SPIN_MODE_ALIGN || g_spin_mode == SPIN_MODE_READY) {
            g_spin_mode = SPIN_MODE_DRIVE;
            printf("[SPIN_CMD] DRIVE START\r\n");
        }
        break;

    case -2: // หยุดหมุนล้อขับ แต่ค้างมุม 45°
        if (g_spin_mode == SPIN_MODE_DRIVE) {
            g_spin_mode = SPIN_MODE_READY;
            printf("[SPIN_CMD] DRIVE STOP (KEEP %.1f deg)\r\n", g_spin_cmd_deg);
        }
        break;

    default:
        printf("[SPIN_CMD] ignored (unknown)\r\n");
        break;
    }
}

void Robot_UpdateSpinSpeedFromLinear(float linear_x)
{
    if (fabsf(linear_x) < 0.05f) {
        g_spin_base_tps = 0.0f;
        return;
    }

    float dir = (linear_x >= 0.0f) ? 1.0f : -1.0f;

    float norm = fabsf(linear_x) / ROBOT_SPIN_LIN_REF;
    if (norm > 1.0f) norm = 1.0f;

    // map ไปเป็น tps (0..ROBOT_SPIN_TPS_MAX)
    float tps = norm * (float)ROBOT_SPIN_TPS_MAX;

    g_spin_dir      = dir;
    g_spin_base_tps = tps;
}

void Robot_CommandTimeoutCheck(void)
{
    // ถ้ายังไม่เคยได้รับ cmd เลย ก็ไม่ต้องทำอะไร
    if (!g_cmd_active) {
        return;
    }

    const uint32_t CMD_TIMEOUT_MS = 500;  // ไม่มี cmd ใหม่เกิน 300ms -> STOP

    uint32_t now = HAL_GetTick();
    uint32_t dt  = now - g_last_cmd_ms;

    if (dt > CMD_TIMEOUT_MS) {
        // --- ไม่มีคำสั่งใหม่มานานเกิน timeout -> สั่งหยุดปลอดภัย ---

        g_cmd_target_tps = 0.0f;
        g_cmd_speed_norm = 0.0f;
        g_cmd_dir_sign   = 0.0f;

        // reset flag เพื่อไม่ให้พิมพ์ซ้ำ/สั่ง stop ซ้ำไปเรื่อย ๆ
        g_cmd_active = false;

        // สั่ง drive หยุด และ reset PID ภายใน Drive_StopAll()
        Drive_StopAll();

        printf("Robot: CMD TIMEOUT (%lu ms) -> STOP ALL\r\n", (unsigned long)dt);
    }
}

