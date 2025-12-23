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

// handler สำหรับ spin_cmd (Int8)
typedef void (*UdpSpinHandler_t)(int8_t cmd);

// UDP สำหรับ /cmd_vel (port 6000)
int UDP_Ctrl_Init(uint16_t port, UdpTwistHandler_t handler);
int UDP_Ctrl_IsReady(void);

// UDP สำหรับ /spin_cmd (port 6001)
int UDP_Spin_Init(uint16_t port, UdpSpinHandler_t handler);
int UDP_Spin_IsReady(void);

#endif /* COMM_UDP_CTRL_H_ */
