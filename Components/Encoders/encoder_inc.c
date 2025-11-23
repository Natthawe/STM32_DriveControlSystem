/*
 * encoder_inc.c
 *
 *  Created on: Nov 23, 2025
 *      Author: cg
 */

#include "Encoders/encoder_inc.h"
#include <stdio.h>

// counts ต่อรอบของ encoder ล้อ
#define WHEEL_COUNTS_PER_REV   1440U

// map: wheel_enc[0..3] -> TIM1,3,4,8
DriveEnc_t drive_enc[DRIVE_NUM] = {
    { &htim1, "DRV_FR", WHEEL_COUNTS_PER_REV, +1, 0, 0 },  // Front Right  drive
    { &htim3, "DRV_RR", WHEEL_COUNTS_PER_REV, +1, 0, 0 },  // Rear  Right  drive
    { &htim4, "DRV_RL", WHEEL_COUNTS_PER_REV, -1, 0, 0 },  // Rear  Left   drive
    { &htim8, "DRV_FL", WHEEL_COUNTS_PER_REV, -1, 0, 0 },  // Front Left  drive
};

void DriveEnc_InitAll(void)
{
    for (uint32_t i = 0; i < DRIVE_NUM; ++i) {
    	DriveEnc_t *w = &drive_enc[i];

        // start encoder mode ทุกช่องของ timer
        HAL_TIM_Encoder_Start(w->htim, TIM_CHANNEL_ALL);
        __HAL_TIM_SET_COUNTER(w->htim, 0);

        w->last_raw    = 0;
        w->multi_ticks = 0;

        HAL_Delay(5);
    }
}

void DriveEnc_UpdateAll(void)
{
    for (uint32_t i = 0; i < DRIVE_NUM; ++i) {
    	DriveEnc_t *w = &drive_enc[i];

        uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(w->htim);

        // ใช้ int16_t เพื่อ handle wrap-around 0..65535
        int16_t diff = (int16_t)(now - w->last_raw);
        w->last_raw = now;

        // คูณ sign เพื่อให้ทิศทางตรงตาม convention ล้อ
        w->multi_ticks += (int32_t)diff * (int32_t)w->sign;
    }
}

int32_t DriveEnc_GetTicks(uint8_t idx)
{
    if (idx >= DRIVE_NUM) return 0;
    return drive_enc[idx].multi_ticks;
}

float DriveEnc_GetRevs(uint8_t idx)
{
    if (idx >= DRIVE_NUM) return 0.0f;

    return (float)drive_enc[idx].multi_ticks /
           (float)drive_enc[idx].counts_per_rev;
}

void DriveEnc_Debug_Print_One(uint8_t idx)
{
    if (idx >= DRIVE_NUM) {
        printf("DRV[%d] : index out of range\r\n", idx);
        return;
    }

    DriveEnc_t *w = &drive_enc[idx];

    printf("DRV[%d] %s : ticks=%ld, rev=%.4f\r\n",
           idx,
           w->name,
           (long)w->multi_ticks,
           (float)w->multi_ticks / (float)w->counts_per_rev);
}

void DriveEnc_Debug_Print_All(void)
{
    for (uint8_t i = 0; i < DRIVE_NUM; ++i) {
        DriveEnc_Debug_Print_One(i);
    }
}
