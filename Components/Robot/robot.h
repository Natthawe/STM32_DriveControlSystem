/*
 * robot.h
 *
 *  Created on: Nov 22, 2025
 *      Author: cg
 */

#ifndef ROBOT_ROBOT_H_
#define ROBOT_ROBOT_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    ROBOT_CMD_NONE = 0,
    ROBOT_CMD_FORWARD,        // เดินหน้า
    ROBOT_CMD_BACKWARD,       // ถอยหลัง
    ROBOT_CMD_TURN_LEFT,      // หมุนเลี้ยวซ้ายอยู่กับที่
    ROBOT_CMD_TURN_RIGHT,     // หมุนเลี้ยวขวาอยู่กับที่
    ROBOT_CMD_STOP,           // หยุด

    ROBOT_CMD_FWD_LEFT,       // เดินหน้า + เลี้ยวซ้าย
    ROBOT_CMD_FWD_RIGHT,      // เดินหน้า + เลี้ยวขวา
    ROBOT_CMD_BACK_LEFT,      // ถอยหลัง + เลี้ยวซ้าย
    ROBOT_CMD_BACK_RIGHT      // ถอยหลัง + เลี้ยวขวา
} RobotCmd_t;

// ===== SPIN MODE (ควบคุมด้วย Int8) =====
typedef enum {
    SPIN_MODE_OFF = 0,   // mode ปกติ ใช้ /cmd_vel ขับ
    SPIN_MODE_ALIGN,     // หักล้อไปมุม SPIN_STEER_DEG (เช่น 45°)
    SPIN_MODE_READY,     // หักล้อถึงเป้าแล้ว แต่ยังไม่หมุนล้อขับ
    SPIN_MODE_DRIVE      // หักล้อถึงเป้าแล้ว + หมุนล้อขับหมุนรอบตัว
} SpinMode_t;

/* state ปัจจุบันของหุ่นยนต์ */
extern RobotCmd_t g_robot_cmd;
extern volatile SpinMode_t g_spin_mode;
extern volatile float g_spin_cmd_deg;     // มุมเป้า 45°
extern volatile float g_spin_base_tps;    // ความเร็วรอบล้อขับตอนหมุนตัว
extern volatile float g_spin_dir;         // ทิศการหมุน (+1 / -1)

/* สั่งหุ่นยนต์ด้วย command + duty (open-loop) */
void Robot_ApplyCommand_WithDuty(RobotCmd_t cmd, float drive_duty, float steer_duty);

/* map char จากคีย์บอร์ด -> RobotCmd_t */
RobotCmd_t Robot_CmdFromChar(uint8_t c);

/** ถูกเรียกใช้ตอนมี /cmd_vel (linear_x, angular_z) เข้ามาผ่าน (UDP/Serial) */
void Robot_ApplyTwist(float linear_x, float angular_z);

/** เช็ค timeout ของคำสั่งขับเคลื่อน (หยุดหุ่นถ้าไม่ได้ cmd ใหม่เกินเวลาที่กำหนด) */
void Robot_CommandTimeoutCheck(void);

bool Robot_IsSpinMode(void);
void Robot_GetSpinParams(float *base_tps, float *dir);
void Robot_SetSpinParams(float base_tps, float dir);

/** เรียกเมื่อได้รับคำสั่ง SPIN จาก joystick/UDP/serial */
void Robot_HandleSpinCommand(int8_t cmd);

/** อ่านสถานะโหมด SPIN ปัจจุบัน */
SpinMode_t Robot_GetSpinMode(void);

/** มุม SPIN ที่ต้องการ (ปกติ = 45°) ไว้ให้ debug / logic ใช้ */
float Robot_GetSpinCmdDeg(void);

/** พารามิเตอร์สำหรับล้อขับตอน SPIN (ความเร็วพื้นฐาน + ทิศทาง) */
void Robot_GetSpinDriveParams(float *base_tps, float *dir);

/** (ถ้าต้องการใช้จากที่อื่น) เซ็ตโหมด SPIN โดยตรง */
void Robot_SetSpinMode(SpinMode_t mode);

void Robot_UpdateSpinSpeedFromLinear(float linear_x);

#endif /* ROBOT_ROBOT_H_ */
