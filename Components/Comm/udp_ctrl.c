/*
 * udp_ctrl.c
 *
 *  Created on: Nov 22, 2025
 *      Author: cg
 */

#include "Comm/udp_ctrl.h"
#include "Comm/udp_enc_tx.h"

#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"

#include <string.h>
#include <stdio.h>

// ====== INTERNAL STATE ======
static struct udp_pcb *g_udp_ctrl_pcb       = NULL;     // สำหรับ cmd_vel
static struct udp_pcb *g_udp_spin_pcb       = NULL;     // สำหรับ spin_cmd
static struct udp_pcb *g_udp_rec_pcb        = NULL;	    // สำหรับ rec_cmd
static struct udp_pcb *g_udp_block_pcb      = NULL; 	// สำหรับ block_state
static struct udp_pcb *g_udp_mode_pcb       = NULL; 	// สำหรับ mode_cmd

static UdpTwistHandler_t g_twist_handler    = NULL;
static UdpSpinHandler_t  g_spin_handler     = NULL;
static UdpRecHandler_t   g_rec_handler      = NULL;
static UdpBlockHandler_t g_block_handler    = NULL;
static Udp_ModeHandler_t g_mode_handler     = NULL;

static uint16_t g_udp_ctrl_port             = 0;
static uint16_t g_udp_spin_port             = 0;
static uint16_t g_udp_rec_port              = 0;
static uint16_t g_udp_block_port            = 0;
static uint16_t g_udp_mode_port             = 0;

// ====== CALLBACK: สำหรับ cmd_vel (float lin, ang) ======
static void udp_ctrl_recv(void *arg, struct udp_pcb *upcb,
                          struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    (void)arg;
    (void)upcb;
    (void)addr;
    (void)port;

    if (p == NULL) {
        return;
    }

//    printf("udp_ctrl: got packet, len=%d\r\n", (int)p->len);

    if (p->len >= 8 && g_twist_handler != NULL) {
        float linear_x  = 0.0f;
        float angular_z = 0.0f;

        memcpy(&linear_x,  p->payload,           4);
        memcpy(&angular_z, (uint8_t*)p->payload + 4, 4);

        g_twist_handler(linear_x, angular_z);
    }

    pbuf_free(p);
}


// ====== CALLBACK: สำหรับ spin_cmd (Int8) ======
static void udp_spin_recv(void *arg, struct udp_pcb *upcb,
                          struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    (void)arg;
    (void)upcb;
    (void)addr;
    (void)port;

    if (p == NULL) {
        return;
    }

    if (p->len >= 1 && g_spin_handler != NULL) {
        int8_t cmd = ((int8_t*)p->payload)[0];
//        printf("udp_spin: got cmd=%d\r\n", (int)cmd);
        g_spin_handler(cmd);
    }

    pbuf_free(p);
}

// ====== CALLBACK: สำหรับ rec_cmd (Int8) ======
static void udp_rec_recv(void *arg, struct udp_pcb *upcb,
                         struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    (void)arg;
    (void)upcb;
    (void)addr;
    (void)port;

    if (p == NULL) {
        return;
    }

    if (p->len >= 1 && g_rec_handler != NULL) {
        int8_t cmd = ((int8_t*)p->payload)[0];
//        printf("udp_rec: got cmd=%d\r\n", (int)cmd);
        g_rec_handler(cmd);
    }

    pbuf_free(p);
}

// ====== CALLBACK: สำหรับ block_state (Int8: 0=clear, 1=blocked) ======
static void udp_block_recv(void *arg, struct udp_pcb *upcb,
                           struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    (void)arg;
    (void)upcb;
    (void)addr;
    (void)port;

    if (p == NULL) {
        return;
    }

//    if (p->len >= 1 && g_block_handler != NULL) {
//        int8_t cmd = ((int8_t*)p->payload)[0];   // 0/1
//        printf("udp_block: got cmd=%d\r\n", (int)cmd);
//        g_block_handler(cmd);
//    }

    if (p->len >= 1 && g_block_handler != NULL) {
        int8_t cmd = ((int8_t*)p->payload)[0];   // 0/1

        // log เฉพาะตอนเปลี่ยน
        static int8_t s_last_block_cmd = 127;
        if (cmd != s_last_block_cmd) {
            printf("udp_block: cmd=%d\r\n", (int)cmd);
            s_last_block_cmd = cmd;
        }

        g_block_handler(cmd);
    }


    pbuf_free(p);
}

// ====== CALLBACK: สำหรับ mode_cmd (Int8: 0=SAFE, 1=RUN(auto align)) ======
static void udp_mode_recv(void *arg, struct udp_pcb *upcb,
                          struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    (void)arg;
    (void)upcb;
    (void)addr;
    (void)port;

    if (p == NULL) {
        return;
    }

    if (p->len >= 1 && g_mode_handler != NULL) {
        int8_t cmd = ((int8_t*)p->payload)[0];   // 0/1/...
        // printf("udp_mode: got cmd=%d\r\n", (int)cmd);

        static int8_t s_last_mode_cmd = 127;
        if (cmd != s_last_mode_cmd) {
            printf("udp_mode: cmd=%d\r\n", (int)cmd);
            s_last_mode_cmd = cmd;
        }

        g_mode_handler(cmd);
    }

    pbuf_free(p);
}


// ====== INIT สำหรับ cmd_vel (/cmd_vel -> port 6000) ======
int UDP_Ctrl_Init(uint16_t port, UdpTwistHandler_t handler)
{
    g_twist_handler = handler;
    g_udp_ctrl_port = port;

    if (g_udp_ctrl_pcb != NULL) {
        udp_remove(g_udp_ctrl_pcb);
        g_udp_ctrl_pcb = NULL;
    }

    g_udp_ctrl_pcb = udp_new();
    if (g_udp_ctrl_pcb == NULL) {
        printf("udp_ctrl: udp_new() failed\r\n");
        return -1;
    }

    err_t err = udp_bind(g_udp_ctrl_pcb, IP_ADDR_ANY, port);
    if (err != ERR_OK) {
        printf("udp_ctrl: udp_bind(port=%u) failed: %d\r\n",
               (unsigned)port, (int)err);
        udp_remove(g_udp_ctrl_pcb);
        g_udp_ctrl_pcb = NULL;
        return -2;
    }

    udp_recv(g_udp_ctrl_pcb, udp_ctrl_recv, NULL);

    printf("udp_ctrl: listening on port %u\r\n", (unsigned)port);
    return 0;
}

int UDP_Ctrl_IsReady(void)
{
    return (g_udp_ctrl_pcb != NULL) ? 1 : 0;
}

// ====== INIT สำหรับ spin_cmd (/spin_cmd -> port 6001) ======
int UDP_Spin_Init(uint16_t port, UdpSpinHandler_t handler)
{
    g_spin_handler  = handler;
    g_udp_spin_port = port;

    if (g_udp_spin_pcb != NULL) {
        udp_remove(g_udp_spin_pcb);
        g_udp_spin_pcb = NULL;
    }

    g_udp_spin_pcb = udp_new();
    if (g_udp_spin_pcb == NULL) {
        printf("udp_spin: udp_new() failed\r\n");
        return -1;
    }

    err_t err = udp_bind(g_udp_spin_pcb, IP_ADDR_ANY, port);
    if (err != ERR_OK) {
        printf("udp_spin: udp_bind(port=%u) failed: %d\r\n",
               (unsigned)port, (int)err);
        udp_remove(g_udp_spin_pcb);
        g_udp_spin_pcb = NULL;
        return -2;
    }

    udp_recv(g_udp_spin_pcb, udp_spin_recv, NULL);

    printf("udp_spin: listening on port %u (spin_cmd)\r\n", (unsigned)port);
    return 0;
}

int UDP_Spin_IsReady(void)
{
    return (g_udp_spin_pcb != NULL) ? 1 : 0;
}

// ====== INIT สำหรับ rec_cmd (/rec_cmd -> port 6002) ======
int UDP_Rec_Init(uint16_t port, UdpRecHandler_t handler)
{
    g_rec_handler  = handler;
    g_udp_rec_port = port;

    if (g_udp_rec_pcb != NULL) {
        udp_remove(g_udp_rec_pcb);
        g_udp_rec_pcb = NULL;
    }

    g_udp_rec_pcb = udp_new();
    if (g_udp_rec_pcb == NULL) {
        printf("udp_rec: udp_new() failed\r\n");
        return -1;
    }

    err_t err = udp_bind(g_udp_rec_pcb, IP_ADDR_ANY, port);
    if (err != ERR_OK) {
        printf("udp_rec: udp_bind(port=%u) failed: %d\r\n",
               (unsigned)port, (int)err);
        udp_remove(g_udp_rec_pcb);
        g_udp_rec_pcb = NULL;
        return -2;
    }

    udp_recv(g_udp_rec_pcb, udp_rec_recv, NULL);

    printf("udp_rec: listening on port %u (rec_cmd)\r\n", (unsigned)port);
    return 0;
}

int UDP_Rec_IsReady(void)
{
    return (g_udp_rec_pcb != NULL) ? 1 : 0;
}

int UDP_Block_Init(uint16_t port, UdpBlockHandler_t handler)
{
    g_block_handler  = handler;
    g_udp_block_port = port;

    if (g_udp_block_pcb != NULL) {
        udp_remove(g_udp_block_pcb);
        g_udp_block_pcb = NULL;
    }

    g_udp_block_pcb = udp_new();
    if (g_udp_block_pcb == NULL) {
        printf("udp_block: udp_new() failed\r\n");
        return -1;
    }

    err_t err = udp_bind(g_udp_block_pcb, IP_ADDR_ANY, port);
    if (err != ERR_OK) {
        printf("udp_block: udp_bind(port=%u) failed: %d\r\n",
               (unsigned)port, (int)err);
        udp_remove(g_udp_block_pcb);
        g_udp_block_pcb = NULL;
        return -2;
    }

    udp_recv(g_udp_block_pcb, udp_block_recv, NULL);

    printf("udp_block: listening on port %u (block_state)\r\n", (unsigned)port);
    return 0;
}

int UDP_Block_IsReady(void)
{
    return (g_udp_block_pcb != NULL) ? 1 : 0;
}

int UDP_Mode_Init(uint16_t port, Udp_ModeHandler_t handler)
{
    g_mode_handler  = handler;
    g_udp_mode_port = port;

    if (g_udp_mode_pcb != NULL) {
        udp_remove(g_udp_mode_pcb);
        g_udp_mode_pcb = NULL;
    }

    g_udp_mode_pcb = udp_new();
    if (g_udp_mode_pcb == NULL) {
        printf("udp_mode: udp_new() failed\r\n");
        return -1;
    }

    err_t err = udp_bind(g_udp_mode_pcb, IP_ADDR_ANY, port);
    if (err != ERR_OK) {
        printf("udp_mode: udp_bind(port=%u) failed: %d\r\n",
               (unsigned)port, (int)err);
        udp_remove(g_udp_mode_pcb);
        g_udp_mode_pcb = NULL;
        return -2;
    }

    udp_recv(g_udp_mode_pcb, udp_mode_recv, NULL);

    printf("udp_mode: listening on port %u (mode_cmd)\r\n", (unsigned)port);
    return 0;
}

int UDP_Mode_IsReady(void)
{
    return (g_udp_mode_pcb != NULL) ? 1 : 0;
}

