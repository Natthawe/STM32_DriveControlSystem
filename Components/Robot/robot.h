/*
 * robot.h
 *
 *  Created on: Nov 22, 2025
 *      Author: cg
 */

#ifndef ROBOT_ROBOT_H_
#define ROBOT_ROBOT_H_

#include <stdint.h>

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

// ====== Odometry / Kinematics (All-Wheel Steer) ======

typedef enum {
    WHEEL_FL = 0,
    WHEEL_FR = 1,
    WHEEL_RL = 2,
    WHEEL_RR = 3,
    WHEEL_COUNT = 4
} WheelIndex_t;

typedef struct {
    float x;   // [m] +x ไปข้างหน้า
    float y;   // [m] +y ไปทางซ้าย
} WheelPos_t;

typedef struct {
    float speed_mps;   // ความเร็วตามแนวล้อ [m/s]
    float steer_rad;   // มุมเลี้ยวของล้อ (0 = ขนานแกน x) [rad]
} WheelState_t;

typedef struct {
    float vx;   // [m/s] linear x ใน base_link
    float vy;   // [m/s] linear y ใน base_link
    float wz;   // [rad/s] angular z
} RobotTwist_t;

typedef struct {
    float x;      // [m] ตำแหน่งใน odom frame
    float y;      // [m]
    float theta;  // [rad] มุม yaw ใน odom frame
} RobotPose_t;

typedef struct {
    WheelState_t wheels[WHEEL_COUNT];
    RobotTwist_t twist;
    RobotPose_t  pose;
    uint32_t     seq;     // running sequence
} RobotState_t;

/* state ปัจจุบันของหุ่นยนต์ */
extern RobotCmd_t g_robot_cmd;

/* สั่งหุ่นยนต์ด้วย command + duty (open-loop) */
void Robot_ApplyCommand_WithDuty(RobotCmd_t cmd, float drive_duty, float steer_duty);

/* map char จากคีย์บอร์ด -> RobotCmd_t */
RobotCmd_t Robot_CmdFromChar(uint8_t c);

/** ถูกเรียกใช้ตอนมี /cmd_vel (linear_x, angular_z) เข้ามาผ่าน (UDP/Serial) */
void Robot_ApplyTwist(float linear_x, float angular_z);

/** เช็ค timeout ของคำสั่งขับเคลื่อน (หยุดหุ่นถ้าไม่ได้ cmd ใหม่เกินเวลาที่กำหนด) */
void Robot_CommandTimeoutCheck(void);

/** เรียกทุกคาบควบคุม (dt_s วินาที) หลังจาก DriveEnc_UpdateAll() */
void Robot_UpdateKinematics(float dt_s);

/** อ่าน state ปัจจุบัน (pointer ใช้ read-only) */
const RobotState_t* Robot_GetState(void);

#endif /* ROBOT_ROBOT_H_ */
