/*
 * recorder.h
 *
 *  Created on: Dec 23, 2025
 *      Author: cg
 */

#ifndef CONTROL_RECORDER_H_
#define CONTROL_RECORDER_H_

#include <stdint.h>
#include <stdbool.h>

// 1 segment = ทิศ + จำนวน tick
typedef struct {
    int8_t  dir;    // +1 = เดินหน้า, -1 = ถอยหลัง
    int32_t ticks;  // จำนวน tick ใน segment นี้ (abs)
} RecSegment_t;

#define REC_MAX_SEGMENTS  16

typedef enum {
    REC_STATE_IDLE = 0,
    REC_STATE_RECORD,
    REC_STATE_PLAY,
} RecState_t;

// init ต้องถูกเรียกตอน boot
void Recorder_Init(void);

// === ฝั่งสั่งโหมด (เรียกจาก Robot_HandleRecordCommand) ===
void Recorder_StartRecord(void);      // เริ่ม record (ล้างของเก่า)
void Recorder_StopRecord(void);       // จบ record
void Recorder_StartPlay(void);        // เริ่ม play ตามที่ record ไว้
void Recorder_StopPlay(void);         // หยุด play

bool Recorder_IsRecording(void);
bool Recorder_IsPlaying(void);

// === ฝั่ง control loop ===
// เรียกทุกรอบพร้อมกับ multi_ticks ของล้ออ้างอิง (เช่น drive_enc[0].multi_ticks)
void Recorder_Update(int32_t current_ticks);

void Recorder_HandleCmd(int8_t cmd);

bool Recorder_IsBlocked(void);
void Recorder_SetBlocked(bool blocked);


// เรียกทุกรอบตอน PLAY เพื่อให้มัน set Robot_ApplyTwist() ให้เอง
// return:
//   0 = ยังเล่นอยู่
//   1 = เล่นจบแล้ว (มันจะหยุดให้แล้ว)
int Recorder_PlayStep(int32_t current_ticks, float dt_s);

#endif /* CONTROL_RECORDER_H_ */
