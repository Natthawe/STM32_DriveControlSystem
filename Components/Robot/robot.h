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

/* state ปัจจุบันของหุ่นยนต์ */
extern RobotCmd_t g_robot_cmd;

/* สั่งหุ่นยนต์ด้วย command + duty (open-loop) */
void Robot_ApplyCommand_WithDuty(RobotCmd_t cmd, float drive_duty, float steer_duty);

/* map char จากคีย์บอร์ด -> RobotCmd_t */
RobotCmd_t Robot_CmdFromChar(uint8_t c);

/** ถูกเรียกใช้ตอนมี /cmd_vel (linear_x, angular_z) เข้ามาผ่าน (UDP/Serial) */
void Robot_ApplyTwist(float linear_x, float angular_z);

#endif /* ROBOT_ROBOT_H_ */
