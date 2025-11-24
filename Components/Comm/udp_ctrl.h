/*
 * udp_ctrl.h
 *
 *  Created on: Nov 22, 2025
 *      Author: cg
 */

#ifndef COMM_UDP_CTRL_H_
#define COMM_UDP_CTRL_H_

#include <stdint.h>


typedef void (*UdpTwistHandler_t)(float linear_x, float angular_z);

int UDP_Ctrl_Init(uint16_t port, UdpTwistHandler_t handler);

int UDP_Ctrl_IsReady(void);

#endif /* COMM_UDP_CTRL_H_ */
