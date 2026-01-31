/*
 * udp_ctrl.h
 *
 *  Created on: Nov 22, 2025
 *      Author: cg
 */

#ifndef COMM_UDP_CTRL_H_
#define COMM_UDP_CTRL_H_

#include <stdint.h>

// handler สำหรับ cmd_vel (linear.x, angular.z)
typedef void (*UdpTwistHandler_t)(float linear_x, float angular_z);
// UDP สำหรับ /cmd_vel (port 6000)
int UDP_Ctrl_Init(uint16_t port, UdpTwistHandler_t handler);
int UDP_Ctrl_IsReady(void);

// handler สำหรับ spin_cmd (Int8)
typedef void (*UdpSpinHandler_t)(int8_t cmd);
// UDP สำหรับ /spin_cmd (port 6001)
int UDP_Spin_Init(uint16_t port, UdpSpinHandler_t handler);
int UDP_Spin_IsReady(void);

// handler สำหรับ /rec_cmd (Int8)
typedef void (*UdpRecHandler_t)(int8_t cmd);
// UDP สำหรับ /rec_cmd (port 6002)
int UDP_Rec_Init(uint16_t port, UdpRecHandler_t handler);
int UDP_Rec_IsReady(void);

typedef void (*UdpBlockHandler_t)(int8_t cmd);
int UDP_Block_Init(uint16_t port, UdpBlockHandler_t handler);
int UDP_Block_IsReady(void);

typedef void (*Udp_ModeHandler_t)(int8_t cmd);
int UDP_Mode_Init(uint16_t port, Udp_ModeHandler_t handler);
int UDP_Mode_IsReady(void);

#endif /* COMM_UDP_CTRL_H_ */
