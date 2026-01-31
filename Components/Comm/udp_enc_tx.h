/*
 * udp_enc_tx.h
 *
 *  Created on: Jan 31, 2026
 *      Author: cg
 */

#ifndef COMM_UDP_ENC_TX_H_
#define COMM_UDP_ENC_TX_H_

#include <stdint.h>

int  UDP_EncTx_InitFixed(uint8_t ip0, uint8_t ip1, uint8_t ip2, uint8_t ip3, uint16_t dst_port);
void UDP_EncTx_Task(uint32_t now_ms);

#endif /* COMM_UDP_ENC_TX_H_ */
