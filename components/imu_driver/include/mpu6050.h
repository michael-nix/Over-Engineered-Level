#pragma once

#include "driver/gpio.h"
#include "esp_bit_defs.h"

#define MPU6050_SCL_PIN      GPIO_NUM_14
#define MPU6050_SDA_PIN      GPIO_NUM_13
#define MPU6050_INT_PIN      GPIO_NUM_15
#define MPU6050_ADDRESS      0x68
#define MPU6050_SCL_SPEED_HZ 400000
#define MPU6050_INTR_TYPE    GPIO_INTR_POSEDGE
#define MPU6050_PULLDOWN_EN  GPIO_PULLDOWN_ENABLE
#define MPU6050_PULLUP_EN    GPIO_PULLUP_DISABLE

/*

Defines the registers and bitfields for configuring and using the MPU6050 6-axis
IMU, as per the nearby located datasheet, `components\imu_driver\docs\MPU-6050
Register Map and Descriptions.pdf`.

*/

// Register 25 - Sample Rate Divider
// Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
// Gyroscope Output Rate = 1 kHz when DLPF (Config (0x1A), DLPF_CFG[2:0]) is
// enabled.
#define SMPRT_DIV_REG  0x19
#define SMPLRT_DIV_VAL 0

// Register 26 - Configuration
#define CONFIG_REG   0x1A
#define DLPF_CFG_VAL 1

// Register 27 - Gyroscope Configuration
#define GYRO_CONFIG_REG 0x1B
#define FS_SEL_VAL      1
#define FS_SEL_BIT      FS_SEL_VAL << 3

// Register 28 - Accelerometer Configuration
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_HPF_VAL    0
#define AFS_SEL_VAL      0
#define AFS_SEL_BIT      AFS_SEL_VAL << 3

// Register 35 - FIFO Enable
#define FIFO_EN_REG       0x23
#define XG_FIFO_EN_BIT    BIT6
#define YG_FIFO_EN_BIT    BIT5
#define ZG_FIFO_EN_BIT    BIT4
#define ACCEL_FIFO_EN_BIT BIT3

// Register 55 - INT Pin Configuration
#define INT_PIN_CFG_REG  0x37
#define INT_LEVEL_BIT    BIT7
#define INT_OPEN_BIT     BIT6
#define LATCH_INT_EN_BIT BIT5

// Register 56 - Interrupt Enable
#define INT_ENABLE_REG    0x38
#define FIFO_OFLOW_EN_BIT BIT4

// Register 104 - Signal Path Reset
#define SIGNAL_PATH_RESET_REG 0x68
#define GYRO_RESET_BIT        BIT2
#define ACCEL_RESET_BIT       BIT1
#define TEMP_RESET_BIT        BIT0

// Register 106 - User Control:
#define USER_CTRL_REG      0x6A
#define FIFO_EN_BIT        BIT6
#define I2C_MST_EN_BIT     BIT5
#define I2C_IF_DIS_BIT     BIT4
#define FIFO_RESET_BIT     BIT2
#define I2C_MST_RESET_BIT  BIT1
#define SIG_COND_RESET_BIT BIT0

// Register 107 - Power Management 1
#define PWR_MGMT_1_REG   0x6B
#define DEVICE_RESET_BIT BIT7
#define SLEEP_BIT        BIT6
#define CYCLE_BIT        BIT5
#define TEMP_DIS_BIT     BIT3
#define CLKSEL_VAL       1

// Register 114 and 115 - FIFO Count Registers
#define FIFO_COUNT_H_REG 0x72
#define FIFO_COUNT_L_REG 0x73

// Register 116 - FIFO Read Write
#define FIFO_R_W_REG 0x74
