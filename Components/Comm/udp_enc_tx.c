/*
 * udp_enc_tx.c
 *
 *  Created on: Jan 31, 2026
 *      Author: cg
 */

#include "Comm/udp_enc_tx.h"

#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"
#include "lwip/ip4_addr.h"

#include <string.h>
#include <stdio.h>

#include "Encoders/encoder_inc.h"
#include "Encoders/encoder_abs.h"

// ========= Packet format =========
typedef struct __attribute__((packed)) {
    uint8_t  magic[4];      // 'E''N''C''1'
    uint8_t  version;       // 1
    uint8_t  flags;         // bit0=steer_filtered_valid (optional)
    uint16_t len;           // sizeof(packet)

    uint32_t seq;
    uint32_t t_ms;

    int32_t  drive_ticks[4];   // FR, RR, RL, FL
    int32_t  steer_abs[4];     // FR, RR, RL, FL (0..1023), error=-1

    uint16_t crc16;
} EncPkt_t;

// ========= CRC16-CCITT =========
static uint16_t crc16_ccitt(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

// ========= Internal =========
static struct udp_pcb *g_tx_pcb = NULL;
static ip_addr_t g_dst_addr;
static uint16_t  g_dst_port = 0;
static uint8_t   g_ready = 0;

static uint32_t  g_seq = 0;

int UDP_EncTx_InitFixed(uint8_t ip0, uint8_t ip1, uint8_t ip2, uint8_t ip3, uint16_t dst_port)
{
    g_dst_port = dst_port;

    if (g_tx_pcb) {
        udp_remove(g_tx_pcb);
        g_tx_pcb = NULL;
    }

    g_tx_pcb = udp_new();
    if (!g_tx_pcb) {
        printf("udp_enc: udp_new() failed\r\n");
        g_ready = 0;
        return -1;
    }

    IP4_ADDR(ip_2_ip4(&g_dst_addr), ip0, ip1, ip2, ip3);
    g_ready = 1;

    printf("udp_enc: fixed dst %u.%u.%u.%u:%u\r\n",
           (unsigned)ip0,(unsigned)ip1,(unsigned)ip2,(unsigned)ip3,(unsigned)dst_port);

    return 0;
}

void UDP_EncTx_Task(uint32_t now_ms)
{
    if (!g_ready || !g_tx_pcb || g_dst_port == 0) return;

    // ส่งทุก 20ms (50Hz)
    static uint32_t last_ms = 0;
    if ((now_ms - last_ms) < 20) return;
    last_ms = now_ms;

    EncPkt_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.magic[0] = 'E'; pkt.magic[1] = 'N'; pkt.magic[2] = 'C'; pkt.magic[3] = '1';
    pkt.version  = 1;
    pkt.flags    = 0;
    pkt.len      = (uint16_t)sizeof(EncPkt_t);

    pkt.seq  = g_seq++;
    pkt.t_ms = now_ms;

    // drive ticks
    for (int i = 0; i < 4; i++) {
        pkt.drive_ticks[i] = DriveEnc_GetTicks((uint8_t)i);
    }

    // steer abs (FILTERED)
    // ต้องเรียก ENC_UpdateFilteredAll() ใน main loop มาก่อนถึงจะนิ่ง
    for (int i = 0; i < 4; i++) {
        uint16_t t = ENC_GetFilteredTicks(i);
        pkt.steer_abs[i] = (t == 0xFFFF) ? -1 : (int32_t)t;
    }
    pkt.flags |= 0x01; // optional: บอกว่าช่อง steer เป็น filtered แล้ว

    pkt.crc16 = crc16_ccitt((const uint8_t*)&pkt, (uint32_t)(sizeof(pkt) - 2));

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(pkt), PBUF_RAM);
    if (!p) return;

    memcpy(p->payload, &pkt, sizeof(pkt));
    udp_sendto(g_tx_pcb, p, &g_dst_addr, g_dst_port);
    pbuf_free(p);
}
