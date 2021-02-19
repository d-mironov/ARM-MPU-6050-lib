#include <inttypes.h>
#include "stm32f10x.h"

#ifndef _MPU_H
#define _MPU_H

#define MPU_ADDR_DEFAULT 0x68
#define MPU_ADDR_SCND 0x69

#define MPU_ACCEL_X_REG 		0x3B
#define MPU_ACCEL_Y_REG 		0x3D
#define MPU_ACCEL_Z_REG 		0x3F
#define MPU_GYRO_X_REG 			0x43
#define MPU_GYRO_Y_REG 			0x45
#define MPU_GYRO_Z_REG 			0x47
#define MPU_TEMP_REG 			0x41
#define MPU_GYRO_CONFIG_REG		0x1B
#define MPU_ACCEL_CONFIG_REG	0x1C
#define MPU_TEMP_CONFIG_REG

#define NUM_CALIBRATIONS 1000

static I2C_TypeDef *I2C_MPU;

typedef enum {
	MPU_Gyro_Range_250 = 0x00,
	MPU_Gyro_Range_500 = (1<<3),
	MPU_Gyro_Range_1000 = (1<<4),
	MPU_Gyro_Range_2000 = ((1<<4) | (1<<3))
}MPU_GyroRange;

typedef enum {
	MPU_Accel_Range_2G = 0x00,
	MPU_Accel_Range_4G = (1<<3),
	MPU_Accel_Range_8G = (1<<4),
	MPU_Accel_Range_16G = ((1<<4) | (1<<3))
}MPU_AccelRange;

static MPU_GyroRange gyroRange;
static MPU_AccelRange accelRange;
static uint8_t MPU_address;

int MPU_init(I2C_TypeDef *I2Cx, uint8_t address);
void MPU_setAddress(uint8_t address);
void MPU_gyroConfig(MPU_GyroRange range);
void MPU_accelConfig(MPU_AccelRange range);
uint8_t MPU_calibrate(void);

uint16_t MPU_read(uint8_t addr);
void MPU_write(uint8_t address, uint8_t data);

int MPU_accel_offset[3];
int MPU_gyro_offset[3];

float MPU_accelX(void);
float MPU_accelY(void);
float MPU_accelZ(void);

float MPU_tempC(void);
float MPU_tempF(void);
float MPU_tempK(void);

float MPU_gyroX(void);
float MPU_gyroY(void);
float MPU_gyroZ(void);

int16_t MPU_gyroXraw(void);
int16_t MPU_gyroYraw(void);
int16_t MPU_gyroZraw(void);

int16_t MPU_accelXraw(void);
int16_t MPU_accelYraw(void);
int16_t MPU_accelZraw(void);

float MPU_get_accelRange(void);
float MPU_get_gyroRange(void);

#endif

