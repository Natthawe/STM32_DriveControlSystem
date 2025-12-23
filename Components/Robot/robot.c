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
#define ROBOT_SPIN_LIN_EPS        0.02f   // ถ้า |linear| < 2 cm/s ถือว่าเป็น 0
#define ROBOT_SPIN_ANG_EPS        0.05f   // ถ้า |angular| < 0.05 rad/s (~3 deg/s) ถือว่าไม่หมุน
#define ROBOT_SPIN_STEER_DEG      45.0f   // มุมหักล้อสำหรับ spin (deg)
#define ROBOT_SPIN_TPS_PER_RAD    2000.0f // map |angular.z| -> tps (0.7 * 2000 = 1400 tps -> limit ด้วย ROBOT_SPIN_TPS_MAX)
#define ROBOT_SPIN_TPS_MAX        1200.0f // limit tps สำหรับ spin

// ====== SPIN MODE STATE ======
static bool  g_spin_mode      = false;
static float g_spin_base_tps  = 0.0f;
static float g_spin_dir       = 0.0f;   // +1 หรือ -1

// ==== Geometry ล้อ ====
#define WHEEL_DIAMETER_M       0.37f
#define WHEEL_COUNTS_PER_REV_F 1440.0f
#define WHEEL_CIRCUM_M         (3.1415926f * WHEEL_DIAMETER_M)
#define TICKS_PER_METER        (WHEEL_COUNTS_PER_REV_F / WHEEL_CIRCUM_M)
// ~ 1440 / (pi*0.37) ≈ 1239 ticks/m

// ====== map จาก linear.x / angular.z ======
#define MAX_CMD_LINEAR   1.0f   // [m/s] linear.x สูงสุดที่คาดว่าจะส่งมา
#define MAX_CMD_ANGULAR  0.5f   // [rad/s] angular.z สูงสุดที่คาดว่าจะส่งมา
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

// /cmd_vel -> linear_x [m/s], angular_z [rad/s] -> g_cmd_target_tps + Steer_SetCmdTargetDeg()
//void Robot_ApplyTwist(float linear_x, float angular_z)
//{
//    // จำเวลาคำสั่งล่าสุด
//    g_last_cmd_ms = HAL_GetTick();
//
//    g_cmd_active  = true;
//
//    // --- จำกัด linear_x ไม่ให้เกิน MAX_CMD_LINEAR ---
//    if (linear_x >  MAX_CMD_LINEAR) linear_x =  MAX_CMD_LINEAR;
//    if (linear_x < -MAX_CMD_LINEAR) linear_x = -MAX_CMD_LINEAR;
//
//    // ใช้ lin_norm เพื่อ deadzone
//    float lin_norm = linear_x / MAX_CMD_LINEAR;   // -1..+1
//    if (lin_norm >  1.0f) lin_norm =  1.0f;
//    if (lin_norm < -1.0f) lin_norm = -1.0f;
//
//    if (fabsf(lin_norm) < LIN_DEADZONE) {
//        lin_norm = 0.0f;
//    }
//
//    // linear_x_cmd หลัง deadzone แล้ว (m/s)
//    float linear_x_cmd = lin_norm * MAX_CMD_LINEAR;
//
//    // --- map เป็น target_tps_common ด้วย geometry ---
//    float target_tps_cmd = linear_x_cmd * TICKS_PER_METER;
//
//    // เก็บเป็นคำสั่งหลักในหน่วย tps
//    g_cmd_target_tps = target_tps_cmd;
//
//    // เก็บ dir + speed_norm ไว้ debug
//    if (lin_norm > 0.0f)      g_cmd_dir_sign = +1.0f;
//    else if (lin_norm < 0.0f) g_cmd_dir_sign = -1.0f;
//    else                      g_cmd_dir_sign = 0.0f;
//
//    g_cmd_speed_norm = fabsf(lin_norm);   // 0..1
//
//    // --- มุมเลี้ยวจาก angular_z ---
//    float ang_norm = angular_z / MAX_CMD_ANGULAR;  // -1..+1
//    if (ang_norm >  1.0f) ang_norm =  1.0f;
//    if (ang_norm < -1.0f) ang_norm = -1.0f;
//    if (fabsf(ang_norm) < ANG_DEADZONE) ang_norm = 0.0f;
//
//    // map -> มุมเลี้ยว +-STEER_MAX_DEG
//    float target_deg = ang_norm * STEER_MAX_DEG;
//
//    if (linear_x_cmd < -0.000001f)
//    {
//        target_deg = -target_deg;
//    }
//
//    // ส่งเป้ามุมไปให้ steer module จัดการ ramp เอง
//    Steer_SetCmdTargetDeg(target_deg);
//
//    g_cmd_active = true;   // เคยได้รับคำสั่งแล้ว
//
////    printf("Twist: lin=%.2f m/s (tgt=%.0f tps), ang=%.2f rad/s -> dir=%.0f, speed_norm=%.2f, deg=%.1f\r\n",
////           linear_x_cmd, target_tps_cmd, angular_z,
////           g_cmd_dir_sign, g_cmd_speed_norm, target_deg);
//}

void Robot_ApplyTwist(float linear_x, float angular_z)
{
    // จำเวลาคำสั่งล่าสุด
    g_last_cmd_ms = HAL_GetTick();
    g_cmd_active  = true;

    float vx = linear_x;
    float wz = angular_z;

    float abs_vx = fabsf(vx);
    float abs_wz = fabsf(wz);

    // ----------- โหมด SPIN จาก /cmd_vel -----------
    // เงื่อนไข: linear.x ≈ 0 แต่ angular.z มีค่า
    if (abs_vx < ROBOT_SPIN_LIN_EPS && abs_wz > ROBOT_SPIN_ANG_EPS) {

        // เข้าโหมด spin
        g_spin_mode = true;

        // ทิศหมุนตาม sign ของ angular.z
        g_spin_dir = (wz >= 0.0f) ? 1.0f : -1.0f;

        // map |angular.z| -> tps
        float base_tps = abs_wz * ROBOT_SPIN_TPS_PER_RAD;
        if (base_tps > ROBOT_SPIN_TPS_MAX) {
            base_tps = ROBOT_SPIN_TPS_MAX;
        }
        g_spin_base_tps = base_tps;

        // ตั้งมุมเลี้ยวเป็นแพทเทิร์น FL ขวา, RL ซ้าย, RR ขวา, FR ซ้าย
        Steer_SetSpinAngleDeg(ROBOT_SPIN_STEER_DEG);

        // ไม่ใช้ linear common target ในโหมด spin
        g_cmd_target_tps = 0.0f;
        g_cmd_dir_sign   = 0.0f;
        g_cmd_speed_norm = 0.0f;

        // ไม่ต้องตั้งมุมด้วย Steer_SetCmdTargetDeg() เพราะ spin ใช้มุมแพทเทิร์นของตัวเองแล้ว
        return;
    }

    // ----------- โหมดปกติ (วิ่งตรง / เลี้ยว AWS) -----------
    g_spin_mode = false;

    // --- จำกัด linear_x ไม่ให้เกิน MAX_CMD_LINEAR ---
    if (vx >  MAX_CMD_LINEAR) vx =  MAX_CMD_LINEAR;
    if (vx < -MAX_CMD_LINEAR) vx = -MAX_CMD_LINEAR;

    float lin_norm = vx / MAX_CMD_LINEAR;   // -1..+1
    if (lin_norm >  1.0f) lin_norm =  1.0f;
    if (lin_norm < -1.0f) lin_norm = -1.0f;

    if (fabsf(lin_norm) < LIN_DEADZONE) {
        lin_norm = 0.0f;
    }

    float linear_x_cmd = lin_norm * MAX_CMD_LINEAR;

    // map -> tps
    float target_tps_cmd = linear_x_cmd * TICKS_PER_METER;
    g_cmd_target_tps = target_tps_cmd;

    if (lin_norm > 0.0f)      g_cmd_dir_sign = +1.0f;
    else if (lin_norm < 0.0f) g_cmd_dir_sign = -1.0f;
    else                      g_cmd_dir_sign = 0.0f;

    g_cmd_speed_norm = fabsf(lin_norm);

    // --- มุมเลี้ยวจาก angular_z ---
    float ang_norm = angular_z / MAX_CMD_ANGULAR;  // -1..+1
    if (ang_norm >  1.0f) ang_norm =  1.0f;
    if (ang_norm < -1.0f) ang_norm = -1.0f;
    if (fabsf(ang_norm) < ANG_DEADZONE) ang_norm = 0.0f;

    float target_deg = ang_norm * STEER_MAX_DEG;

    if (linear_x_cmd < -0.000001f) {
        target_deg = -target_deg;
    }

    Steer_SetCmdTargetDeg(target_deg);
}

bool Robot_IsSpinMode(void)
{
    return g_spin_mode;
}

void Robot_GetSpinParams(float *base_tps, float *dir)
{
    if (base_tps) *base_tps = g_spin_base_tps;
    if (dir)      *dir      = g_spin_dir;
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

