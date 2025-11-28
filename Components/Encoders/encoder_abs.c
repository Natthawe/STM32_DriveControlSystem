/*
 * encoder_abs.c
 *
 *  Created on: Nov 22, 2025
 *      Author: cg
 */

#include "Encoders/encoder_abs.h"
#include <stdio.h>

// ==== CS pin mapping ====

#define ENC2_CS_GPIO_Port   GPIOB
#define ENC2_CS_Pin         GPIO_PIN_4

#define ENC3_CS_GPIO_Port   ENC3_CS_Pin_GPIO_Port
#define ENC3_CS_Pin         ENC3_CS_Pin_Pin

#define ENC4_CS_GPIO_Port   GPIOE
#define ENC4_CS_Pin         ENC4_CS_Pin_Pin

#define ENC5_CS_GPIO_Port   GPIOF
#define ENC5_CS_Pin         ENC5_CS_Pin_Pin

// ความละเอียด encoder 10-bit
#define ENC_RESOLUTION_TICKS   1024.0f

// ตาราง mapping encoder -> SPI / CS
static const SteerEnc_t enc_if[ABS_ENCODER_COUNT] = {
    { &hspi2, ENC2_CS_GPIO_Port, ENC2_CS_Pin, "ENC2/SPI2" },    //FR    PD3=SCK,     PC2=MISO,   PB4=CS
    { &hspi3, ENC3_CS_GPIO_Port, ENC3_CS_Pin, "ENC3/SPI3" },    //RR    PC10=SCK,    PC11=MISO,  PA4=CS
    { &hspi4, ENC4_CS_GPIO_Port, ENC4_CS_Pin, "ENC4/SPI4" },    //RL    PE2=SCK,     PE13=MISO,  PE4=CS
    { &hspi5, ENC5_CS_GPIO_Port, ENC5_CS_Pin, "ENC5/SPI5" },    //FL    PF7=SCK,     PF8=MISO,   PF9=CS
};

void ENC_StartALL(void)
{
    // ตั้ง CS ของ encoder ทุกตัวเป็น HIGH (inactive)
    HAL_GPIO_WritePin(ENC2_CS_GPIO_Port, ENC2_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ENC3_CS_GPIO_Port, ENC3_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ENC4_CS_GPIO_Port, ENC4_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ENC5_CS_GPIO_Port, ENC5_CS_Pin, GPIO_PIN_SET);
}

uint16_t ENC_ReadRaw_ByIndex(int idx)
{
    if (idx < 0 || idx >= ABS_ENCODER_COUNT) {
        return 0xFFFF;
    }

    const SteerEnc_t *e = &enc_if[idx];

    uint8_t rx[2] = {0xFF, 0xFF};

    // CS low
    HAL_GPIO_WritePin(e->cs_port, e->cs_pin, GPIO_PIN_RESET);

    // small delay (guard time)
    for (volatile int i = 0; i < 100; i++) {
        __NOP();
    }

    HAL_StatusTypeDef st = HAL_SPI_Receive(e->hspi, rx, 2, 10);

    // CS high
    HAL_GPIO_WritePin(e->cs_port, e->cs_pin, GPIO_PIN_SET);

    if (st != HAL_OK) {
        printf("%s SPI ERR status=%d\r\n", e->name, st);
        return 0xFFFF;
    }

    // byte1: MSB, byte0: LSB (10-bit valid)
    uint16_t raw   = ((uint16_t)rx[1] << 8) | rx[0];
    uint16_t ticks = raw & 0x03FF;   // 10-bit (0..1023)

    return ticks;
}

float ENC_TicksToDeg(uint16_t ticks)
{
    if (ticks == 0xFFFF) return -1.0f;
    return (float)ticks * 360.0f / ENC_RESOLUTION_TICKS;
}

// from -> current, to -> target (zero_offset)
int16_t ENC10_Diff(uint16_t from, uint16_t to)
{
    int16_t diff = (int16_t)to - (int16_t)from;  // ~[-1023..+1023]

    if (diff > 512) {
        diff -= 1024;  // หมุนย้อนกลับดีกว่า
    } else if (diff < -512) {
        diff += 1024;
    }
    // ตอนนี้ diff อยู่ในช่วง [-512, +511]
    return diff;
}

void Read_ENC_INDEX(void)
{
	uint16_t t[4];
	float    d[4];

	for (int i = 0; i < 4; ++i)
	{
	    t[i] = ENC_ReadRaw_ByIndex(i);
	    d[i] = ENC_TicksToDeg(t[i]);
	}

	printf("ENC2: %4u (%.2f deg) | ENC3: %4u (%.2f deg) | ENC4: %4u (%.2f deg) | ENC5: %4u (%.2f deg)\r\n",
			  t[0], d[0], t[1], d[1], t[2], d[2], t[3], d[3]);
}

// อ่านแบบทีละตัว (return เป็นองศา)
float ENC_ReadDeg_ByIndex(int idx)
{
    uint16_t t = ENC_ReadRaw_ByIndex(idx);
    return ENC_TicksToDeg(t);
}

// print ทีละตัว
void ENC_Debug_Print_One(int idx)
{
    if (idx < 0 || idx >= ABS_ENCODER_COUNT) {
        printf("ABS[%d] : index out of range\r\n", idx);
        return;
    }

    uint16_t t = ENC_ReadRaw_ByIndex(idx);
    float    d = ENC_TicksToDeg(t);

    printf("ABS[%d] %s : %4u (%.2f deg)\r\n",
           idx, enc_if[idx].name, t, d);
}
