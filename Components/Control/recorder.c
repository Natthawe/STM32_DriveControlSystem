/*
 * recorder.c
 *
 *  Created on: Dec 23, 2025
 *      Author: cg
 */

// Control/recorder.c
#include "Control/recorder.h"
#include "Robot/robot.h"   // เพื่อเรียก Robot_ApplyTwist()

#include <stdio.h>

typedef struct {
    RecState_t state;

    // ===== RECORD =====
    int32_t last_ticks;
    uint8_t rec_initialized;
    uint8_t seg_count;
    RecSegment_t segs[REC_MAX_SEGMENTS];

    // ===== PLAY =====
    uint8_t play_idx;
    int32_t play_start_ticks;
    uint8_t play_initialized;

    // ===== LOOP =====
    uint8_t  loop_enabled;   // 0 = ไม่วน, 1 = วน

    // ===== BLOCK STATE =====
    uint8_t blocked;   // 0 = clear, 1 = blocked
} Recorder_t;

static Recorder_t g_rec;

// ความเร็วตอน replay (m/s) ลอง 0.3 ก่อน ค่อยปรับทีหลังได้
#define REC_PLAY_V_MPS   0.6f

void Recorder_Init(void)
{
    g_rec.state 			= REC_STATE_IDLE;
    g_rec.seg_count 		= 0;
    g_rec.rec_initialized 	= 0;
    g_rec.play_initialized 	= 0;
    g_rec.play_idx 			= 0;
    g_rec.loop_enabled    	= 0;
    g_rec.blocked 			= 0;
}

bool Recorder_IsRecording(void)
{
    return (g_rec.state == REC_STATE_RECORD);
}

bool Recorder_IsPlaying(void)
{
    return (g_rec.state == REC_STATE_PLAY);
}

// ---------- RECORD CONTROL ----------
void Recorder_StartRecord(void)
{
    g_rec.state           = REC_STATE_RECORD;
    g_rec.seg_count       = 0;
    g_rec.rec_initialized = 0;   // ให้ไป init ใน Recorder_Update
    g_rec.loop_enabled    = 0;   // เริ่ม Record ใหม่ => ปิด loop
    printf("[REC] START RECORD\n");
}


void Recorder_StopRecord(void)
{
    if (g_rec.state == REC_STATE_RECORD) {
        g_rec.state = REC_STATE_IDLE;
        printf("[REC] STOP RECORD, seg_count=%u\n", (unsigned)g_rec.seg_count);
        for (uint8_t i = 0; i < g_rec.seg_count; ++i) {
            printf("  seg[%u]: dir=%d, ticks=%ld\n",
                   (unsigned)i,
                   (int)g_rec.segs[i].dir,
                   (long)g_rec.segs[i].ticks);
        }
    }
}

// ---------- PLAY CONTROL ----------

void Recorder_StartPlay(void)
{
    if (g_rec.seg_count == 0) {
        printf("[REC] PLAY START FAILED: no segments\n");
        return;
    }
    g_rec.state           = REC_STATE_PLAY;
    g_rec.play_idx        = 0;
    g_rec.play_initialized = 0;  // ให้ไป init ด้วย tick แรก
    printf("[REC] PLAY START (%u segments)\n", (unsigned)g_rec.seg_count);
}

void Recorder_StopPlay(void)
{
    if (g_rec.state == REC_STATE_PLAY) {
        g_rec.state = REC_STATE_IDLE;
        // safety: หยุดหุ่นยนต์
        Robot_ApplyTwist(0.0f, 0.0f);
        printf("[REC] PLAY STOP\n");
    }
}

// ---------- RECORD UPDATE (เรียกใน loop) ----------

void Recorder_Update(int32_t current_ticks)
{
    if (g_rec.state != REC_STATE_RECORD) {
        return;
    }

    // ครั้งแรก: แค่จดค่าไว้ก่อน ยังไม่สร้าง segment
    if (!g_rec.rec_initialized) {
        g_rec.last_ticks = current_ticks;
        g_rec.rec_initialized = 1;
        return;
    }

    int32_t diff = current_ticks - g_rec.last_ticks;
    g_rec.last_ticks = current_ticks;

    if (diff == 0) {
        return;
    }

    int8_t dir   = (diff > 0) ? +1 : -1;
    int32_t step = (diff > 0) ? diff : -diff;   // abs(diff)

    if (g_rec.seg_count == 0) {
        // segment แรก
        g_rec.segs[0].dir   = dir;
        g_rec.segs[0].ticks = step;
        g_rec.seg_count     = 1;
        return;
    }

    RecSegment_t *cur = &g_rec.segs[g_rec.seg_count - 1];

    if (cur->dir == dir) {
        // ทิศเดิม → รวม tick ต่อไป
        cur->ticks += step;
    } else {
        // ทิศเปลี่ยน → สร้าง segment ใหม่
        if (g_rec.seg_count < REC_MAX_SEGMENTS) {
            g_rec.segs[g_rec.seg_count].dir   = dir;
            g_rec.segs[g_rec.seg_count].ticks = step;
            g_rec.seg_count++;
        } else {
            printf("[REC] segment overflow, stop record.\n");
            g_rec.state = REC_STATE_IDLE;
        }
    }
}

void Recorder_HandleCmd(int8_t cmd)
{
    switch (cmd) {

    case 1: // toggle record start/stop
        if (Recorder_IsRecording()) {
            // กำลังอัดอยู่ → หยุดอัด
            Recorder_StopRecord();
        } else {
            // ถ้ากำลังเล่นอยู่ ให้หยุดก่อน
            if (Recorder_IsPlaying()) {
                Recorder_StopPlay();
            }
            g_rec.loop_enabled = 0;   // เริ่ม record ใหม่ → ไม่วน
            Recorder_StartRecord();
        }
        break;

    case 2: // เริ่ม PLAY แบบวนลูป
        if (g_rec.seg_count == 0) {
            printf("[REC] cmd=2 (play) but no segments, ignore\n");
        } else {
            g_rec.loop_enabled = 1;   // เปิดโหมดวนลูป
            Recorder_StartPlay();
        }
        break;

    case 0:   // stop play
    case -1:  // อีกค่าเผื่ออนาคต
        g_rec.loop_enabled = 0;
        Recorder_StopPlay();
        printf("[REC] cmd=%d -> STOP PLAY & CLEAR LOOP\n", (int)cmd);
        break;

    default:
        printf("[REC] unknown cmd=%d\n", (int)cmd);
        break;
    }
}

bool Recorder_IsBlocked(void)
{
    return (g_rec.blocked != 0);
}

void Recorder_SetBlocked(bool blocked)
{
    g_rec.blocked = blocked ? 1 : 0;
}


// ---------- PLAY STEP (เรียกใน loop) ----------

int Recorder_PlayStep(int32_t current_ticks, float dt_s)
{
    (void)dt_s; // ตอนนี้ยังไม่ใช้ dt_s แต่เผื่อใช้ในอนาคต

    // ถ้าโดน block → หยุดนิ่ง รอจนกว่าจะ clear
    if (g_rec.blocked) {
        Robot_ApplyTwist(0.0f, 0.0f);   // หยุดล้อไว้
        return 0;                       // ยังไม่จบ play แค่ pause
    }

    if (g_rec.state != REC_STATE_PLAY) {
        return 0;
    }

    if (g_rec.play_idx >= g_rec.seg_count) {
        // เล่นจบทุก segment แล้ว
        Robot_ApplyTwist(0.0f, 0.0f);

        if (g_rec.loop_enabled) {
            // วนลูปใหม่
            printf("[REC] LOOP RESTART\n");
            g_rec.play_idx         = 0;
            g_rec.play_initialized = 0;
            return 0;  // ยังเล่น (แต่เริ่ม loop รอบใหม่)
        } else {
            // เล่นครั้งเดียว → จบ
            g_rec.state = REC_STATE_IDLE;
            printf("[REC] PLAY DONE\n");
            return 1;
        }
    }


    RecSegment_t *seg = &g_rec.segs[g_rec.play_idx];

    // init จุดเริ่มนับของ segment นี้
    if (!g_rec.play_initialized) {
        g_rec.play_start_ticks = current_ticks;
        g_rec.play_initialized = 1;
        printf("[REC] PLAY seg[%u]: dir=%d, ticks=%ld\n",
               (unsigned)g_rec.play_idx,
               (int)seg->dir,
               (long)seg->ticks);
    }

    int32_t traveled    = current_ticks - g_rec.play_start_ticks;
    int32_t traveled_abs = (traveled >= 0) ? traveled : -traveled;

    // สั่งความเร็วตามทิศของ segment
    float v = (seg->dir > 0) ? REC_PLAY_V_MPS : -REC_PLAY_V_MPS;
    Robot_ApplyTwist(v, 0.0f);   // ให้ Robot_ApplyTwist แปลงเป็น tps ตามระบบเดิม

     // *** DEBUG: ดูว่าขณะ PLAY เราสั่ง v อะไรอยู่ และ encoder วิ่งไปเท่าไหร่แล้ว ***
     printf("[REC] seg[%u]: traveled=%ld / %ld ticks, v=%.3f m/s\n",
            (unsigned)g_rec.play_idx,
            (long)traveled_abs,
            (long)seg->ticks,
            (double)v);

    if (traveled_abs >= seg->ticks) {
        // segment นี้ครบระยะแล้ว → ข้ามไปอันถัดไป
        g_rec.play_idx++;
        g_rec.play_initialized = 0;  // ให้ re-init start_ticks ใน segment ถัดไป
    }

    return 0;  // ยังเล่นอยู่
}
