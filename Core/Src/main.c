/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <sys/unistd.h>
#include <stdint.h>
#include <math.h>
#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include <string.h>
#include <stdbool.h>
#include "Motors/motor.h"
#include "Encoders/encoder_abs.h"
#include "Encoders/encoder_inc.h"
#include "Control/pid_ctrl.h"
#include "Control/drive_control.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DRIVE_DUTY  		0.7f
#define STEER_DUTY  		0.6f

#define WHEEL_COUNTS_PER_REV   1440U

// ==== Geometry ล้อ ====
#define WHEEL_DIAMETER_M       0.37f
#define WHEEL_COUNTS_PER_REV_F 1440.0f
#define WHEEL_CIRCUM_M         (3.1415926f * WHEEL_DIAMETER_M)
#define TICKS_PER_METER        (WHEEL_COUNTS_PER_REV_F / WHEEL_CIRCUM_M)
// ~ 1440 / (π*0.37) ≈ 1239 ticks/m

// ====== Steer PID (MOTOR 2,4,6,8) ======
#define STEER_NUM   (sizeof(steer_axes)/sizeof(steer_axes[0]))

#define STEER_DEADBAND_TICKS   5.0f       	// ถ้าอยู่ในระยะนี้ถือว่าตรงแล้ว ไม่ต้องหมุน
#define STEER_MIN_DUTY         0.6f       	// duty ต่ำสุด กันอาการสั่นเบา ๆ แล้วไม่ขยับ

#define STEER_JOG_DUTY_DEFAULT   0.7f     	// duty เวลา jog
#define STEER_JOG_MS_DEFAULT     100U     	// หมุนครั้งละ x ms

#define STEER_TICKS_MAX      		1024U
#define STEER_MAX_DEG        		40.0f   		// ไม่เกิน 40 องศา
#define ENC_TICKS_PER_REV    		1024.0f
#define ENC_TICKS_PER_DEG    		(ENC_TICKS_PER_REV / 360.0f)  // ≈ 2.8444 tick/deg
#define STEER_RAMP_DEG_PER_SEC  	40.0f    // deg/s

static float g_cmd_target_deg     = 0.0f; // มุมที่ ROS/serial สั่งมา
static float g_current_target_deg = 0.0f; // มุมที่ ramp แล้ว (ใช้จริง)


// ====== Drive PID (MOTOR 1,3,5,7) ======
#define DRIVE_MAX_TICKS_PER_SEC   1000.0f   // ปรับตามความเร็วจริงของล้อ
#define DRIVE_MIN_DUTY            0.4f      // duty ต่ำสุดกันไม่หมุน

// ---- สำหรับ ramp ความเร็ว drive ----
float g_cmd_dir_sign        = 0.0f;  // -1,0,+1 จากคำสั่งล่าสุด
float g_cmd_speed_norm      = 0.0f;  // 0..1 จาก drive_pct
float g_current_speed_norm  = 0.0f;  // 0..1 ที่ถูก ramp แล้ว

// เป้า "ความเร็วล้อ" รวมทุกล้อ (ticks/sec) จากคำสั่ง (ROS/serial)
float g_cmd_target_tps     = 0.0f;  // target_tps_common จากคำสั่ง
float g_current_target_tps = 0.0f;  // ที่ ramp แล้ว ใช้จริงใน PID

// กำหนดอัตราเร่ง/อัตราหน่วง (หน่วย: สัดส่วนต่อวินาที)
// เช่น จาก 0 -> 100% ใช้เวลาประมาณ 1 วินาที
#define SPEED_RAMP_UP_NORM_PER_SEC    1.0f   // เร่งขึ้น ค่าน้อยออกตัวนุ่ม
#define SPEED_RAMP_DOWN_NORM_PER_SEC  1.0f   // เบรค ค่าน้อยเบรคช้า

// ====== การ map จาก linear.x / angular.z ======
#define MAX_CMD_LINEAR   1.0f   // [m/s] linear.x สูงสุดที่คาดว่าจะส่งมา
#define MAX_CMD_ANGULAR  0.7f   // [rad/s] angular.z สูงสุดที่คาดว่าจะส่งมา
#define LIN_DEADZONE     0.02f  // deadzone สำหรับ linear (normalized)
#define ANG_DEADZONE     0.02f  // deadzone สำหรับ angular (normalized)

// ====== UDP control ======
static struct udp_pcb *g_udp_ctrl_pcb = NULL;

//typedef struct {
//	TIM_HandleTypeDef *htim;
//	uint32_t channel;
//	GPIO_TypeDef *portR;
//	uint16_t pinR;
//	GPIO_TypeDef *portL;
//	uint16_t pinL;
//} Motor_t;
//
//typedef enum {
//	MOTOR_DIR_BRAKE = 0,
//	MOTOR_DIR_FWD = 1,
//	MOTOR_DIR_REV = -1
//} MotorDir_t;

typedef struct {
    const char *name;
    int         enc_index;      // index ใน enc_if[]  (0..3)
    uint16_t    zero_offset;    // tick เมื่อล้อตั้งตรง
    uint8_t     motor_idx;      // motor index 1..8
    int8_t      enc_dir;        // +1 / -1 สำหรับแกนนี้
    uint16_t    target_ticks;   // tick เป้าหมาย (0..1023)
    float       duty_base;      // duty ขั้นต่ำต่อแกน
    PID_t       pid;
    float       last_duty;      // duty ล่าสุดที่ส่งให้มอเตอร์ (ไว้ debug)
} SteerAxis_t;




typedef enum {
    RUN_MODE_STEER_CALIB = 0,   // โหมด JOG: Calibrate ตั้ง  zero_offset
    RUN_MODE_STEER_PID          // โหมด PID: รักษามุมให้ตรง zero_offset
} RunMode_t;

static RunMode_t g_steer_mode = RUN_MODE_STEER_PID;

static const uint8_t steer_motor_ids[4] = { 2, 4, 6, 8 };

static void SteerMotor_Jog(uint8_t axis, MotorDir_t dir, float duty)
{
    if (axis >= 4) return;
    uint8_t motor_idx = steer_motor_ids[axis];
    Motor_set(motor_idx, dir, duty);
}

typedef enum {
    ROBOT_CMD_NONE = 0,
    ROBOT_CMD_FORWARD,        // เดินหน้า
    ROBOT_CMD_BACKWARD,       // ถอยหลัง
    ROBOT_CMD_TURN_LEFT,      // หมุนเลี้ยวซ้ายอยู่กับที่
    ROBOT_CMD_TURN_RIGHT,     // หมุนเลี้ยวขวาอยู่กับที่
    ROBOT_CMD_STOP,           // หยุด

    ROBOT_CMD_FWD_LEFT,       // เดินหน้า + เลี้ยวซ้าย
    ROBOT_CMD_FWD_RIGHT,      // เดินหน้า + เลี้ยวขวา
    ROBOT_CMD_BACK_LEFT,      // ถอยหลัง + เลี้ยวซ้าย
    ROBOT_CMD_BACK_RIGHT      // ถอยหลัง + เลี้ยวขวา
} RobotCmd_t;


RobotCmd_t g_robot_cmd = ROBOT_CMD_STOP;

void Robot_ApplyCommand_WithDuty(RobotCmd_t cmd, float drive_duty, float steer_duty)
{
    // clamp duty
    if (drive_duty < 0.0f) drive_duty = 0.0f;
    if (drive_duty > 1.0f) drive_duty = 1.0f;
    if (steer_duty < 0.0f) steer_duty = 0.0f;
    if (steer_duty > 1.0f) steer_duty = 1.0f;

    switch (cmd) {

    case ROBOT_CMD_FORWARD:
        Motors_Drive_All(MOTOR_DIR_FWD, drive_duty);
        Motors_Steer_All(MOTOR_DIR_BRAKE, 0.0f);
        break;

    case ROBOT_CMD_BACKWARD:
        Motors_Drive_All(MOTOR_DIR_REV, drive_duty);
        Motors_Steer_All(MOTOR_DIR_BRAKE, 0.0f);
        break;

    case ROBOT_CMD_TURN_LEFT:
        Motors_Drive_All(MOTOR_DIR_BRAKE, 0.0f);
        Motors_Steer_All(MOTOR_DIR_REV, steer_duty);
        break;

    case ROBOT_CMD_TURN_RIGHT:
        Motors_Drive_All(MOTOR_DIR_BRAKE, 0.0f);
        Motors_Steer_All(MOTOR_DIR_FWD, steer_duty);
        break;

    case ROBOT_CMD_FWD_LEFT:
        Motors_Drive_All(MOTOR_DIR_FWD, drive_duty);
        Motors_Steer_All(MOTOR_DIR_REV, steer_duty);
        break;

    case ROBOT_CMD_FWD_RIGHT:
        Motors_Drive_All(MOTOR_DIR_FWD, drive_duty);
        Motors_Steer_All(MOTOR_DIR_FWD, steer_duty);
        break;

    case ROBOT_CMD_BACK_LEFT:
        Motors_Drive_All(MOTOR_DIR_REV, drive_duty);
        Motors_Steer_All(MOTOR_DIR_REV, steer_duty);
        break;

    case ROBOT_CMD_BACK_RIGHT:
        Motors_Drive_All(MOTOR_DIR_REV, drive_duty);
        Motors_Steer_All(MOTOR_DIR_FWD, steer_duty);
        break;

    case ROBOT_CMD_STOP:
    default:
        Motors_Drive_All(MOTOR_DIR_BRAKE, 0.0f);
        Motors_Steer_All(MOTOR_DIR_BRAKE, 0.0f);
        break;
    }
}

RobotCmd_t Robot_CmdFromChar(uint8_t c)
{
    switch (c) {
    case 'i': case 'I': return ROBOT_CMD_FORWARD;
    case ',':           return ROBOT_CMD_BACKWARD;

    case 'j': case 'J': return ROBOT_CMD_TURN_LEFT;
    case 'l': case 'L': return ROBOT_CMD_TURN_RIGHT;
    case 'k': case 'K': return ROBOT_CMD_STOP;

    case 'u': case 'U': return ROBOT_CMD_FWD_LEFT;
    case 'o': case 'O': return ROBOT_CMD_FWD_RIGHT;

    case 'm': case 'M': return ROBOT_CMD_BACK_LEFT;
    case '.':           return ROBOT_CMD_BACK_RIGHT;

    case 'w': case 'W': return ROBOT_CMD_FORWARD;
    case 's': case 'S': return ROBOT_CMD_BACKWARD;
    case 'a': case 'A': return ROBOT_CMD_TURN_LEFT;
    case 'd': case 'D': return ROBOT_CMD_TURN_RIGHT;
    case 'x': case 'X': return ROBOT_CMD_STOP;

    default:
        return ROBOT_CMD_NONE;
    }
}


static SteerAxis_t steer_axes[] = {
	//  name, enc_idx, zero,  motor, enc_dir target, duty_base, PID
	{ "STEER_FR", 0, 489, 2, -1, 489, 0.7f,
//	  { .kp=0.01f, .ki=0.03f, .kd=0.0f,
	  { .kp=0.007f, .ki=0.0f, .kd=0.0f,
		.integrator=0, .prev_error=0,
		.out_min=-1.0f, .out_max=1.0f } },	// ENC2 -> Motor2 (front-right steer)

	{ "STEER_RR", 1, 641, 4, -1, 641,  0.64f,
//	  { .kp=0.006f, .ki=0.011f, .kd=0.0f,
	  { .kp=0.006f, .ki=0.0f, .kd=0.0f,
		.integrator=0, .prev_error=0,
		.out_min=-1.0f, .out_max=1.0f } },	// ENC3 -> Motor4 (rear-right steer)

	{ "STEER_RL", 2,  33, 6, -1,  33,  0.64f,
//	  { .kp=0.006f, .ki=0.01f, .kd=0.0f,
	  { .kp=0.006f, .ki=0.0f, .kd=0.0f,
		.integrator=0, .prev_error=0,
		.out_min=-1.0f, .out_max=1.0f } },	// ENC4 -> Motor6 (rear-left steer)

	{ "STEER_FL", 3,  5,  8,  -1,  5,  0.7f,
//	  { .kp=0.0074f, .ki=0.02f, .kd=0.0f,
	  { .kp=0.007f, .ki=0.0f, .kd=0.0f,
		.integrator=0, .prev_error=0,
		.out_min=-1.0f, .out_max=1.0f } },	// ENC5 -> Motor8 (front-left steer)
};

// for ROS
static void Steer_UpdateAll(float dt_s)
{
    if (dt_s <= 0.0f) dt_s = 0.001f;

    static uint32_t dbg_cnt = 0;
    dbg_cnt++;

    // ถ้า |cmd - current| > 0.5° แปลว่ายัง ramp อยู่
    bool steering_ramping = (fabsf(g_cmd_target_deg - g_current_target_deg) > 0.5f);

    for (uint32_t i = 0; i < STEER_NUM; ++i) {
        SteerAxis_t *ax = &steer_axes[i];

        uint16_t ticks = ENC_ReadRaw_ByIndex(ax->enc_index);
        if (ticks == 0xFFFF) {
            Motor_set(ax->motor_idx, MOTOR_DIR_BRAKE, 0.0f);
            ax->last_duty = 0.0f;
            continue;
        }

        int16_t err_ticks = ENC10_Diff(ticks, ax->target_ticks);
        err_ticks = (int16_t)(err_ticks * ax->enc_dir);
        float err_f = (float)err_ticks;

        bool in_deadband = (err_f > -STEER_DEADBAND_TICKS &&
                            err_f <  STEER_DEADBAND_TICKS);

        // ถ้าไม่ ramp แล้ว และอยู่ใน deadband -> hold เฉย ๆ
        if (!steering_ramping && in_deadband) {
            Motor_set(ax->motor_idx, MOTOR_DIR_BRAKE, 0.0f);
            ax->last_duty = 0.0f;
            continue;
        }

        // ----- P control ล้วน ๆ -----
        // u = Kp * error  (หน่วยเดียวกับ out_min/out_max = -1..+1)
        float u = ax->pid.kp * err_f;

        // clamp ตามขอบเขต PID เดิม
        if (u > ax->pid.out_max) u = ax->pid.out_max;
        if (u < ax->pid.out_min) u = ax->pid.out_min;

        MotorDir_t dir;
        float duty_raw;

        if (u >= 0.0f) {
            dir      = MOTOR_DIR_FWD;
            duty_raw =  u;
        } else {
            dir      = MOTOR_DIR_REV;
            duty_raw = -u;
        }

        // base duty + clamp
        float duty = duty_raw;
        if (duty < ax->duty_base) duty = ax->duty_base;
        if (duty > 1.0f)          duty = 1.0f;

        ax->last_duty = duty;
        Motor_set(ax->motor_idx, dir, duty);

        // debug ถ้าอยากดู
        /*
        if (dbg_cnt % 20 == 0) {
            float err_deg = err_f * 360.0f / 1024.0f;
            printf("[%s] err=%4d (%.2f deg), u=%.3f, duty=%.2f, dir=%d (ramp=%d)\r\n",
                   ax->name,
                   (int)err_ticks,
                   err_deg,
                   u,
                   duty,
                   (int)dir,
                   (int)steering_ramping);
        }
        */
    }
}

static uint16_t wrap_ticks(int32_t t)
{
    while (t < 0)                 			t += STEER_TICKS_MAX;
    while (t >= (int32_t)STEER_TICKS_MAX)	t -= STEER_TICKS_MAX;
    return (uint16_t)t;
}

// angle_deg: มุมเลี้ยวเท่ากันทุกล้อ (+ ซ้าย, - ขวา)
static void Steer_SetTargetAngleDeg(float angle_deg)
{
    // clamp มุมไม่เกิน ±40°
    if (angle_deg >  STEER_MAX_DEG) angle_deg =  STEER_MAX_DEG;
    if (angle_deg < -STEER_MAX_DEG) angle_deg = -STEER_MAX_DEG;

    float delta_ticks_f = angle_deg * ENC_TICKS_PER_DEG;

    for (uint32_t i = 0; i < STEER_NUM; ++i) {
        int32_t base  = (int32_t)steer_axes[i].zero_offset;
        int32_t delta = (int32_t)lrintf(delta_ticks_f);   // ปัดเป็น int
        steer_axes[i].target_ticks = wrap_ticks(base + delta);
    }
}

// อัปเดตมุมเป้าหมายแบบ ramp จาก g_cmd_target_deg -> g_current_target_deg
static void Steer_UpdateTargetWithRamp(float dt_s)
{
    if (dt_s <= 0.0f) dt_s = 0.001f;

    // ต่างของมุม (deg)
    float delta = g_cmd_target_deg - g_current_target_deg;

    // limit การเปลี่ยนแปลงต่อรอบ (deg)
    float max_step = STEER_RAMP_DEG_PER_SEC * dt_s;

    if (delta >  max_step) delta =  max_step;
    if (delta < -max_step) delta = -max_step;

    g_current_target_deg += delta;

    // deadzone เล็ก ๆ รอบ 0 กันค้างค่าจิ๋ว ๆ
    if (fabsf(g_current_target_deg) < 0.05f && fabsf(g_cmd_target_deg) < 0.05f) {
        g_current_target_deg = 0.0f;
    }

    // เซ็ต target_ticks ของทุกล้อจากมุมที่ ramp แล้ว
    Steer_SetTargetAngleDeg(g_current_target_deg);
}


// เริ่มต้น: ให้ target = zero ทั้งหมด
static void Steer_InitTargetsToZero(void)
{
    g_cmd_target_deg     = 0.0f;
    g_current_target_deg = 0.0f;
    Steer_SetTargetAngleDeg(0.0f);
}

// === Helper: Print current steer zeros as steer_axes[] config ===
static void Steer_PrintCurrentZeroConfig(void)
{
    uint16_t ticks[STEER_NUM];
    float    degs[STEER_NUM];

    // อ่าน encoder ทุกแกน
    for (uint32_t i = 0; i < STEER_NUM; ++i) {
        uint16_t t = ENC_ReadRaw_ByIndex(steer_axes[i].enc_index);
        ticks[i] = t;
        degs[i]  = ENC_TicksToDeg(t);
    }

    printf("\r\n==== STEER ZERO CAL RESULT ====\r\n");
    for (uint32_t i = 0; i < STEER_NUM; ++i) {
        printf("[ZCAL] %s: tick=%4u (%.2f deg)\r\n",
               steer_axes[i].name,
               (unsigned)ticks[i],
               degs[i]);
    }

    printf("\r\n/* Paste config below into your code (steer_axes[]): */\r\n");
    printf("static SteerAxis_t steer_axes[] = {\r\n");

    for (uint32_t i = 0; i < STEER_NUM; ++i) {
        SteerAxis_t *ax = &steer_axes[i];
        uint16_t z = ticks[i];

        printf("    { \"%s\", %d, %u, %u, %d, %u, %.2ff,\r\n"
               "      { .kp=%.6ff, .ki=%.6ff, .kd=%.6ff,\r\n"
               "        .integrator=0, .prev_error=0,\r\n"
               "        .out_min=%.1ff, .out_max=%.1ff } },\r\n",
               ax->name,
               ax->enc_index,
               (unsigned)z,        // zero_offset ใหม่
               ax->motor_idx,
               (int)ax->enc_dir,
               (unsigned)z,        // target_ticks เริ่มต้น = zero_offset
               ax->duty_base,
               ax->pid.kp,
               ax->pid.ki,
               ax->pid.kd,
               ax->pid.out_min,
               ax->pid.out_max);
    }

    printf("};\r\n\n");
}


static void PrintModeHelp(RunMode_t mode)
{
    printf("\r\n=============================\r\n");
    if (mode == RUN_MODE_STEER_CALIB) {
        printf(" RUN MODE: STEER CALIB (JOG)\r\n");
        printf("  1-4 : select steer axis (1..4)\r\n");
        printf("  a   : turn LEFT  (REV)\r\n");
        printf("  d   : turn RIGHT (FWD)\r\n");
        printf("  s   : stop motor\r\n");
        printf("  z   : read encoder tick for current axis\r\n");
        printf("  p   : switch to PID mode\r\n");
    } else {
        printf(" RUN MODE: STEER PID (from ROS /cmd_vel)\r\n");
        printf("  - Drive: from serial packet [mode, drive_pct, steer_pct]\r\n");
        printf("  - Steer: PID to target angle (max 40 deg)\r\n");
        printf("  c : switch to CALIB mode\r\n");
    }
    printf("=============================\r\n");
}

void Steer_JogCalib_HandleUart(void)
{
    static uint8_t current_axis = 0;              // 0..3 -> STEER1..4

    const float    jog_duty = STEER_JOG_DUTY_DEFAULT;
    const uint32_t JOG_MS   = STEER_JOG_MS_DEFAULT;

    uint8_t ch;

    if (HAL_UART_Receive(&huart3, &ch, 1, 10) != HAL_OK) {
        return;
    }

    switch (ch)
    {
    case '1':
    case '2':
    case '3':
    case '4':
        current_axis = (uint8_t)(ch - '1');  // '1'->0, '2'->1 ...
        printf("Selected STEER axis %d\r\n", (int)(current_axis + 1));
        break;

    case 'a':
    case 'A':
        SteerMotor_Jog(current_axis, MOTOR_DIR_REV, jog_duty);
        HAL_Delay(JOG_MS);
        SteerMotor_Jog(current_axis, MOTOR_DIR_BRAKE, 0.0f);
        printf("Axis %d: JOG LEFT  (%.0f ms, duty=%.2f)\r\n",
               (int)(current_axis + 1),
               (float)JOG_MS,
               jog_duty);
        break;

    case 'd':
    case 'D':
        SteerMotor_Jog(current_axis, MOTOR_DIR_FWD, jog_duty);
        HAL_Delay(JOG_MS);
        SteerMotor_Jog(current_axis, MOTOR_DIR_BRAKE, 0.0f);
        printf("Axis %d: JOG RIGHT (%.0f ms, duty=%.2f)\r\n",
               (int)(current_axis + 1),
               (float)JOG_MS,
               jog_duty);
        break;

    case 's':
    case 'S':
        SteerMotor_Jog(current_axis, MOTOR_DIR_BRAKE, 0.0f);
        printf("Axis %d: STOP\r\n", (int)(current_axis + 1));
        break;

    case 'z':
    case 'Z':
    {
        if (ch == 'z') {
            		// อ่านเฉพาะแกนปัจจุบัน (ตาม current_axis)
            uint16_t ticks = ENC_ReadRaw_ByIndex(current_axis);
            float    deg   = ENC_TicksToDeg(ticks);
            printf("CALIB axis %d : zero_ticks = %u (%.2f deg)\r\n",
                   (int)(current_axis + 1),
                   (unsigned)ticks,
                   deg);
        } else {
            // 'Z' = อ่านทุกแกนแล้วพิมพ์โค้ด steer_axes[] ครบทั้ง 4 ล้อ
            Steer_PrintCurrentZeroConfig();
        }
//        uint16_t ticks = ENC_ReadRaw_ByIndex(current_axis);
//        float    deg   = ENC_TicksToDeg(ticks);
//        printf("CALIB axis %d : zero_ticks = %u (%.2f deg)\r\n",
//               (int)(current_axis + 1),
//               (unsigned)ticks,
//               deg);
        break;
    }

    // ---- สลับโหมดกลางทางจาก CALIB -> PID / CALIB เอง ----
    case 'c':
    case 'C':
        g_steer_mode = RUN_MODE_STEER_CALIB;
        printf("\r\n[MODE] Stay in CALIB mode\r\n");
        PrintModeHelp(g_steer_mode);
        break;

    case 'p':
    case 'P':
        g_steer_mode = RUN_MODE_STEER_PID;
        printf("\r\n[MODE] Switch to PID mode\r\n");
        PrintModeHelp(g_steer_mode);
        break;

    default:
        break;
    }
}

static uint32_t g_last_cmd_ms = 0;
#define CMD_TIMEOUT_MS   500U   // ms

static void Robot_ApplyTwist(float linear_x, float angular_z)
{
    g_last_cmd_ms = HAL_GetTick();

    // --- 1) จำกัด linear_x ไม่ให้เกิน MAX_CMD_LINEAR ---
    if (linear_x >  MAX_CMD_LINEAR) linear_x =  MAX_CMD_LINEAR;
    if (linear_x < -MAX_CMD_LINEAR) linear_x = -MAX_CMD_LINEAR;

    // ใช้ lin_norm เพื่อ deadzone
    float lin_norm = linear_x / MAX_CMD_LINEAR;   // -1..+1
    if (lin_norm >  1.0f) lin_norm =  1.0f;
    if (lin_norm < -1.0f) lin_norm = -1.0f;

    if (fabsf(lin_norm) < LIN_DEADZONE) {
        lin_norm = 0.0f;
    }

    // linear_x_cmd หลัง deadzone แล้ว (m/s)
    float linear_x_cmd = lin_norm * MAX_CMD_LINEAR;

    // --- 2) map เป็น target_tps_common ด้วย geometry ---
    // TICKS_PER_METER ~ 1239 ticks/m
    float target_tps_cmd = linear_x_cmd * TICKS_PER_METER;

    // เก็บเป็นคำสั่งหลักในหน่วย tps
    g_cmd_target_tps = target_tps_cmd;

    // เก็บ dir + speed_norm ไว้ debug ถ้าอยากใช้
    if (lin_norm > 0.0f)      g_cmd_dir_sign = +1.0f;
    else if (lin_norm < 0.0f) g_cmd_dir_sign = -1.0f;
    else                      g_cmd_dir_sign = 0.0f;
    g_cmd_speed_norm = fabsf(lin_norm);   // 0..1

    // --- 3) มุมเลี้ยวจาก angular_z ---
    float ang_norm = angular_z / MAX_CMD_ANGULAR;  // -1..+1
    if (ang_norm >  1.0f) ang_norm =  1.0f;
    if (ang_norm < -1.0f) ang_norm = -1.0f;
    if (fabsf(ang_norm) < ANG_DEADZONE) ang_norm = 0.0f;

    // map -> มุมเลี้ยว +-STEER_MAX_DEG
    float target_deg = ang_norm * STEER_MAX_DEG;
//    Steer_SetTargetAngleDeg(target_deg);
    g_cmd_target_deg = target_deg;   // ไม่เรียก Steer_SetTargetAngleDeg ตรง ๆ แล้ว

//    printf("Twist: lin=%.2f m/s (tgt=%.0f tps), ang=%.2f rad/s -> dir=%.0f, speed_norm=%.2f, deg=%.1f\r\n",
//           linear_x_cmd, target_tps_cmd, angular_z,
//           g_cmd_dir_sign, g_cmd_speed_norm, target_deg);
}

// callback ตอนมี UDP packet เข้ามาที่ port control
static void udp_ctrl_recv(void *arg, struct udp_pcb *upcb,
                          struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    if (p == NULL) {
        return;
    }

    if (p->len >= 8) {
        float linear_x  = 0.0f;
        float angular_z = 0.0f;

        // payload: 0..3 = linear_x (float32), 4..7 = angular_z (float32)
        memcpy(&linear_x,  p->payload,           4);
        memcpy(&angular_z, (uint8_t*)p->payload + 4, 4);

        Robot_ApplyTwist(linear_x, angular_z);
    }

    pbuf_free(p);
}

// สร้าง UDP listener
static void UDP_Ctrl_Init(void)
{
    g_udp_ctrl_pcb = udp_new();
    if (g_udp_ctrl_pcb == NULL) {
        printf("udp_new() failed\r\n");
        return;
    }

    // ใช้ port 6000 (ปรับได้ตามใจ แต่ฝั่ง ROS ต้องส่งมาที่ port เดียวกัน)
    err_t err = udp_bind(g_udp_ctrl_pcb, IP_ADDR_ANY, 6000);
    if (err != ERR_OK) {
        printf("udp_bind() failed: %d\r\n", err);
        udp_remove(g_udp_ctrl_pcb);
        g_udp_ctrl_pcb = NULL;
        return;
    }

    udp_recv(g_udp_ctrl_pcb, udp_ctrl_recv, NULL);
    printf("UDP control listening on port 6000\r\n");
}


#define CTRL_PERIOD_MS   10U   // control loop ทุก 10 ms (100 Hz)

void Drive_Control_And_Test(uint32_t now_ms)
{
    static uint32_t last_ctrl_ms  = 0;
    static uint32_t last_debug_ms = 0;
    static int32_t  prev_ticks[DRIVE_NUM] = {0};

    uint32_t diff_ms = now_ms - last_ctrl_ms;
    if (diff_ms < CTRL_PERIOD_MS) {
        return;   // ยังไม่ถึงเวลา control รอบใหม่
    }
    last_ctrl_ms = now_ms;

    float dt_s = diff_ms / 1000.0f;
    if (dt_s <= 0.0f) dt_s = 0.001f;
    if (dt_s > 0.1f)  dt_s = 0.1f;

    // อ่าน encoder ทุกล้อ
    DriveEnc_UpdateAll();

    if (g_drive_mode == RUN_MODE_DRIVE_PID) {
        // ===== โหมดปกติ: ใช้ PID ตาม target_tps (common) =====
//        Drive_UpdateTargetsWithRamp(dt_s);  	// ramp g_current_target_tps -> g_cmd_target_tps
    	Drive_UpdateTargetsWithRamp(dt_s, &g_cmd_target_tps, &g_current_target_tps);
        Drive_UpdateAll(dt_s);              	// ใช้ target_tps ต่อ-ล้อ + duty_base + PID

        // เฉพาะตอน RUN_MODE_STEER_PID เท่านั้นที่ให้ PID ทำงาน เผื่อใช้โหมด CALIB
        if (g_steer_mode == RUN_MODE_STEER_PID) {
            Steer_UpdateTargetWithRamp(dt_s);   // ramp angle
            Steer_UpdateAll(dt_s);              // PID ตาม target_ticks
        }

//        Steer_UpdateTargetWithRamp(dt_s);    	// ramp angle
//        Steer_UpdateAll(dt_s);					// PID ตาม target_ticks ที่ถูก ramp แล้ว

        // เช็ค timeout จาก UDP/serial
        if (g_last_cmd_ms != 0U) {
            uint32_t dt_cmd = now_ms - g_last_cmd_ms;
            if (dt_cmd > CMD_TIMEOUT_MS) {
                // reset คำสั่งความเร็วในหน่วย tps
                g_cmd_target_tps     = 0.0f;
                g_current_target_tps = 0.0f;

                // reset ตัวแปรแบบเดิม (เผื่อใช้ที่อื่น)
                g_cmd_dir_sign       = 0.0f;
                g_cmd_speed_norm     = 0.0f;
                g_current_speed_norm = 0.0f;

                Drive_StopAll();
                g_cmd_target_deg     = 0.0f;
                g_current_target_deg = 0.0f;
                Steer_SetTargetAngleDeg(0.0f);
                g_last_cmd_ms = 0U;
                // printf("[TIMEOUT] stop & steer zero\n");
            }
        }

    } else {
        // ===== โหมด TEST: ขับทีละล้อหรือทุกล้อแบบ open-loop =====
        for (uint32_t i = 0; i < DRIVE_NUM; ++i) {
            DriveAxis_t *d = &drive_axes[i];

            uint8_t selected =
                (g_test_drive_idx < 0) ||          // -1 = ALL wheels
                (i == (uint32_t)g_test_drive_idx); // index ที่เลือก

            if (selected && g_test_duty > 0.0f) {
                MotorDir_t dir = (g_test_dir > 0) ? MOTOR_DIR_FWD : MOTOR_DIR_REV;
                Motor_set(d->motor_idx, dir, g_test_duty);
            } else {
                Motor_set(d->motor_idx, MOTOR_DIR_BRAKE, 0.0f);
            }
        }
    }

    // ===== DEBUG tps ทุก ๆ ~100ms =====
    uint32_t dbg_diff = now_ms - last_debug_ms;
    if (dbg_diff >= 100) {
        last_debug_ms = now_ms;

        float tps[DRIVE_NUM];
        float dbg_dt_s = dbg_diff / 1000.0f;
        if (dbg_dt_s <= 0.0f) dbg_dt_s = 0.001f;

        for (uint32_t i = 0; i < DRIVE_NUM; ++i) {
            int32_t now_ticks  = drive_enc[i].multi_ticks;
            int32_t diff_ticks = now_ticks - prev_ticks[i];
            prev_ticks[i]      = now_ticks;
            tps[i] = diff_ticks / dbg_dt_s;
        }

        if (g_drive_mode == RUN_MODE_DRIVE_CALIB)
        {
            if (g_test_drive_idx < 0)
            {
                printf("[TEST] wheel=ALL duty=%.2f dir=%d | tps=[%.0f,%.0f,%.0f,%.0f]\r\n",
                       g_test_duty,
                       (int)g_test_dir,
                       tps[0], tps[1], tps[2], tps[3]);
            } else {
                printf("[TEST] wheel=%d duty=%.2f dir=%d | tps=[%.0f,%.0f,%.0f,%.0f]\r\n",
                       (int)(g_test_drive_idx + 1),
                       g_test_duty,
                       (int)g_test_dir,
                       tps[0], tps[1], tps[2], tps[3]);
            }
        }
        // ใน NORMAL mode ตอนนี้เรามี debug จาก Drive_UpdateAll() อยู่แล้ว
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */

  Motor_PWM_StartALL();
  ENC_StartALL();
  DriveEnc_InitAll();
  Drive_InitAll();
  Steer_InitTargetsToZero();
  UDP_Ctrl_Init();

  printf("\r\n=== Boot OK ===\r\n");

  g_drive_mode = RUN_MODE_DRIVE_PID;
  g_steer_mode   = RUN_MODE_STEER_PID;
  PrintModeHelp(g_steer_mode);

//  uint32_t last_tick = HAL_GetTick();

//  Motor_set(1, MOTOR_DIR_FWD, 0.7f);
//  HAL_Delay(3000);
//  Motor_set(1, MOTOR_DIR_BRAKE, 0.0f);
//  HAL_Delay(1000);

  //	-------- check ticks --------
//  // ตั้งให้ล้อ 1,3,5,7 วิ่งไปข้างหน้าด้วย duty 0.5
//  Motor_set(1, MOTOR_DIR_FWD, 0.5f);
//  Motor_set(3, MOTOR_DIR_FWD, 0.5f);
//  Motor_set(5, MOTOR_DIR_FWD, 0.5f);
//  Motor_set(7, MOTOR_DIR_FWD, 0.5f);

//  printf("DRV_FR ticks = %ld\r\n", drive_enc[0].multi_ticks);
//  uint32_t last_ms = HAL_GetTick();
//  int32_t prev_ticks[4] = {
//      drive_enc[0].multi_ticks,
//      drive_enc[1].multi_ticks,
//      drive_enc[2].multi_ticks,
//      drive_enc[3].multi_ticks
//  };

//  uint8_t buf[3];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    // รับ UDP จาก ROS

	  MX_LWIP_Process();

	  uint32_t now_ms = HAL_GetTick();

	    // ควบคุมล้อทั้งหมด (PID / CALIB ขึ้นกับ g_drive_mode และ g_steer_mode)
	  Drive_Control_And_Test(now_ms);

	  if (g_steer_mode == RUN_MODE_STEER_CALIB)
	  {
	      Steer_JogCalib_HandleUart();
	  }
//	  --------- TEST ---------
//	  uint32_t now_ms = HAL_GetTick();

//	  Drive_Control_And_Test(now_ms);	// คุมล้อ + โหมด CALIB/PID

//	  Process_UART_TestDrive();	// เปลี่ยนโหมด NORMAL/TEST + เลือกล้อ + duty

//	  --------- Jog ---------
//	  Steer_JogCalib_HandleUart();


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20404768;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_10BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi3.Init.DataSize = SPI_DATASIZE_10BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi4.Init.DataSize = SPI_DATASIZE_10BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi5.Init.DataSize = SPI_DATASIZE_10BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 7;
  hspi5.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi5.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 43199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 43199;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 43199;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 43199;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ENC4_CS_Pin_Pin|MOTOR2_L_Pin|MOTOR3_L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, MOTOR4_R_Pin|MOTOR4_L_Pin|ENC5_CS_Pin_Pin|MOTOR3_R_Pin
                          |MOTOR1_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENC3_CS_Pin_GPIO_Port, ENC3_CS_Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|ENC2_CS_Pin_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, MOTOR2_R_Pin|MOTOR1_L_Pin|MOTOR8_R_Pin|MOTOR5_R_Pin
                          |MOTOR5_L_Pin|MOTOR6_R_Pin|MOTOR6_L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, MOTOR7_L_Pin|MOTOR7_R_Pin|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR8_L_GPIO_Port, MOTOR8_L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ENC4_CS_Pin_Pin MOTOR2_L_Pin MOTOR3_L_Pin */
  GPIO_InitStruct.Pin = ENC4_CS_Pin_Pin|MOTOR2_L_Pin|MOTOR3_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR4_R_Pin MOTOR4_L_Pin ENC5_CS_Pin_Pin MOTOR3_R_Pin
                           MOTOR1_R_Pin */
  GPIO_InitStruct.Pin = MOTOR4_R_Pin|MOTOR4_L_Pin|ENC5_CS_Pin_Pin|MOTOR3_R_Pin
                          |MOTOR1_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC3_CS_Pin_Pin */
  GPIO_InitStruct.Pin = ENC3_CS_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENC3_CS_Pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin ENC2_CS_Pin_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|ENC2_CS_Pin_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR2_R_Pin MOTOR1_L_Pin MOTOR8_R_Pin MOTOR5_R_Pin
                           MOTOR5_L_Pin MOTOR6_R_Pin MOTOR6_L_Pin */
  GPIO_InitStruct.Pin = MOTOR2_R_Pin|MOTOR1_L_Pin|MOTOR8_R_Pin|MOTOR5_R_Pin
                          |MOTOR5_L_Pin|MOTOR6_R_Pin|MOTOR6_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR7_L_Pin MOTOR7_R_Pin PG6 */
  GPIO_InitStruct.Pin = MOTOR7_L_Pin|MOTOR7_R_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR8_L_Pin */
  GPIO_InitStruct.Pin = MOTOR8_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR8_L_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
