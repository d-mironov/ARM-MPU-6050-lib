/**
 * |=====================| STM32F10xx MPU-6050/GY-521 Library |=====================|
 * |																				|
 * |	@author: SL7															    |
 * |	@version: v1.0																|
 * |																				|
 * |	Changelog:																	|
 * |	-- v1.0:																	|
 * |		|__ Initial commit														|
 * |																				|
 * |	TODO:																		|
 * |	- [ ] More Boards															|
 * |	- [ ] make more stable														|
 * |    - [ ] add ERROR Codes														|
 * | 	- [ ] make better delay														|
 * |    - [ ] enhance calibration													|
 * |    - [ ] add more configs														|
 * |    - [ ] add Self-Test routine													|
 * |																				|
 * |================================================================================|
 */

#include "mpu.h"

#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
#include <stdio.h>


#include "../delay.h"

/**
 * Init function to set the Address of the MPU
 * and to set the I2Cx which the MPU is connected to
 * Also initializes the Delay and the MPU
 * @param I2Cx I2C port
 * @param address MPU Address
 */
int MPU_init(I2C_TypeDef *I2Cx, uint8_t address) {
	if (I2C_MPU == NULL) {
		I2C_MPU = I2Cx;
	}
	DelayInit();
	MPU_address = address;
	MPU_write(0x6B, 0x00);
	DelayMs(4);
	return 1;
}

/**
 * Read function to read from a specific register in the MPU
 * @param addr Address of the data you need
 */
uint16_t MPU_read(uint8_t addr) {
	uint16_t data;
	uint32_t temp;
	I2C_MPU->CR1 |= I2C_CR1_ACK;
	I2C_MPU->CR1 |= I2C_CR1_START;
	while (!(I2C_MPU->SR1 & I2C_SR1_SB));
	I2C_MPU->DR = MPU_address<<1;
	while (!(I2C_MPU->SR1 & I2C_SR1_ADDR));
	temp = I2C_MPU->SR2;
	I2C_MPU->DR = addr;
	I2C_MPU->CR1 |= I2C_CR1_START;
	while(!(I2C_MPU->SR1 & I2C_SR1_SB));
	I2C_MPU->DR = (MPU_address<<1|1);
	while(!(I2C_MPU->SR1 & I2C_SR1_ADDR));
	temp = I2C_MPU->SR2;
	while(!(I2C_MPU->SR1 & I2C_SR1_RXNE));
	I2C_MPU->CR1 &= ~I2C_CR1_ACK;
	I2C_MPU->CR1 |= I2C_CR1_STOP;
	data = I2C_MPU->DR;
	while(!(I2C_MPU->SR1 & I2C_SR1_RXNE));
	data <<= 8;
	data |= I2C_MPU->DR;
	DelayMs(2);
	return data;
}

/**
 * Write to a specific register on the MPU
 * @param addr addres to write to
 * @param data data to write
 */
void MPU_write(uint8_t address, uint8_t data) {
	uint32_t temp;

	I2C_MPU->CR1 |= I2C_CR1_START;					// Generate Start condition

	while(!(I2C_MPU->SR1 & I2C_FLAG_SB));			// Wait until the Start condition is sent

	I2C_MPU->DR = MPU_address<<1;							// Send device over to the Date register
	while(!(I2C_MPU->SR1 & I2C_SR1_ADDR));			// Wait until a Slave responds to the address
	temp = I2C_MPU->SR2;

	I2C_MPU->DR = address;								// Send internal register to write to
	while (!(I2C_MPU->SR1 & I2C_SR1_TXE));			// Wait until the Transmit register is empty and thus the data is transmitted

	I2C_MPU->DR = data; 							// Put in the data you want to write to the address
	while (!(I2C_MPU->SR1 & I2C_SR1_TXE));

	I2C_MPU->CR1 |= I2C_CR1_STOP;
}

/**
 * Calibrate the MPU offsets
 * DONT TOUCH THE DEVICE UNTIL CALIBRATION HAS FINISHED
 * Takes NUM_CALIBRATIONS readings and gets the average of them
 */
uint8_t MPU_calibrate(void) {
	int i;
	for (int j = 0; j < 3; j++) {
		MPU_accel_offset[j] = 0;
		MPU_gyro_offset[j] = 0;
	}
	//printf("MPU 6050 Calibration");
	for (i = 0; i < NUM_CALIBRATIONS; i++) {
		if (i % 50 == 0) {
			//printf(".");
		}
		MPU_accel_offset[0] += MPU_accelXraw();
		MPU_accel_offset[1] += MPU_accelYraw();
		MPU_accel_offset[2] += MPU_accelZraw();
		MPU_gyro_offset[0] += MPU_gyroXraw();
		MPU_gyro_offset[1] += MPU_gyroYraw();
		MPU_gyro_offset[2] += MPU_gyroZraw();
	}
	//printf("\r\n");
	for (int j = 0; j < 3; j++) {
		MPU_accel_offset[j] /= NUM_CALIBRATIONS;
		MPU_gyro_offset[j] /= NUM_CALIBRATIONS;
	}
	//printf("Offset: %d | %d | %d \r\n", MPU_gyro_offset[0], MPU_gyro_offset[1], MPU_gyro_offset[2]);
	DelayMs(2);
	return 1;
}

/**
 * Configure the range of the gyro
 * @param range gyro range
 */
void MPU_gyroConfig(MPU_GyroRange range) {
	gyroRange = range;
	MPU_write(0x1B, gyroRange);
	DelayMs(2);
}

/**
 * Configure the Accelerometer range
 * @param range accelerometer range
 */
void MPU_accelConfig(MPU_AccelRange range) {
	accelRange = range;
	MPU_write(0x1C, accelRange);
	DelayMs(2);
}

/**
 * Read the calculated Accelerometer value on X-axis
 */
float MPU_accelX(void) {
	int16_t accelX;
	accelX = MPU_read(MPU_ACCEL_X_REG);
	accelX -= MPU_accel_offset[0];
	float accelX_calc;
	accelX_calc = accelX / (MPU_get_accelRange());
	DelayMs(2);
	return accelX_calc;
}
/**
 * Read the calculated Accelerometer value on Y-axis
 */
float MPU_accelY(void) {
	int16_t accelY;
	accelY = MPU_read(MPU_ACCEL_Y_REG);
	float accelY_calc;
	accelY_calc = accelY / (MPU_get_accelRange());
	DelayMs(2);
	return accelY_calc;
}
/**
 * Read the calculated Accelerometer value on Z-axis
 */
float MPU_accelZ(void) {
	int16_t accelZ;
	accelZ = MPU_read(MPU_ACCEL_Z_REG);
	float accelZ_calc;
	accelZ_calc = accelZ / (MPU_get_accelRange());
	DelayMs(2);
	return accelZ_calc;
}
/**
 * Read the Temperature sensor in degrees Celcius
 */
float MPU_tempC(void) {
	int16_t temp;
	temp = MPU_read(MPU_TEMP_REG);
	float tempC;
	tempC = temp / 340 + 36.53;
	DelayMs(2);
	return tempC;
}
/**
 * Read the Temperature sensor in degrees Fahrenheit
 */
float MPU_tempF(void) {
	float temp = MPU_tempC();
	float tempF;
	tempF = temp * 9 / 5 + 32;
	DelayMs(2);
	return tempF;
}
/**
 * Read the Temperature sensor in Kelvin
 */
float MPU_tempK(void) {
	float temp = MPU_tempC();
	float tempK;
	tempK = temp + 273.15;
	DelayMs(2);
	return tempK;
}
/**
 * Read calculated Gyroscope values on X-axis
 */
float MPU_gyroX(void) {
	int16_t gyroX;
	gyroX = MPU_read(MPU_GYRO_X_REG);
	gyroX -= MPU_gyro_offset[0];
	float gyroX_calc;
	gyroX_calc = gyroX / (MPU_get_gyroRange());
	DelayMs(2);
	return gyroX_calc;
}
/**
 * Read calculated Gyroscope values on Y-axis
 */
float MPU_gyroY(void) {
	int16_t gyroY;
	gyroY = MPU_read(MPU_GYRO_Y_REG);
	gyroY -= MPU_gyro_offset[1];
	float gyroY_calc;
	gyroY_calc = gyroY / (MPU_get_gyroRange());
	DelayMs(2);
	return gyroY_calc;
}
/**
 * Read calculated Gyroscope values on Z-axis
 */
float MPU_gyroZ(void) {
	int16_t gyroZ;
	gyroZ = MPU_read(MPU_GYRO_Z_REG);
	gyroZ -= MPU_gyro_offset[2];
	float gyroZ_calc;
	gyroZ_calc = gyroZ / (MPU_get_gyroRange());
	DelayMs(2);
	return gyroZ_calc;
}
/**
 * Read Raw values of the Accelerometer on the X-axis
 */
int16_t MPU_accelXraw(void) {
	int16_t accelX;
	accelX = MPU_read(MPU_ACCEL_X_REG);
	DelayMs(2);
	return accelX;
}
/**
 * Read Raw values of the Accelerometer on the Y-axis
 */
int16_t MPU_accelYraw(void) {
	int16_t accelY;
	accelY = MPU_read(MPU_ACCEL_Y_REG);
	DelayMs(2);
	return accelY;
}
/**
 * Read Raw values of the Accelerometer on the Z-axis
 */
int16_t MPU_accelZraw(void) {
	int16_t accelZ;
	accelZ = MPU_read(MPU_ACCEL_Z_REG);
	DelayMs(2);
	return accelZ;
}
/**
 * Read Raw values of the Gyroscope on the X-axis
 */
int16_t MPU_gyroXraw(void) {
	int16_t gyroX;
	gyroX = MPU_read(MPU_GYRO_X_REG);
	DelayMs(2);
	return gyroX;
}
/**
 * Read Raw values of the Gyroscope on the Y-axis
 */
int16_t MPU_gyroYraw(void) {
	int16_t gyroY;
	gyroY = MPU_read(MPU_GYRO_Y_REG);
	DelayMs(2);
	return gyroY;
}
/**
 * Read Raw values of the Gyroscope on the Z-axis
 */
int16_t MPU_gyroZraw(void) {
	int16_t gyroZ;
	gyroZ = MPU_read(MPU_GYRO_Z_REG);
	DelayMs(2);
	return gyroZ;
}
/**
 * Get the Accelerometer range value
 */
float MPU_get_accelRange(void) {
	switch (accelRange) {
	case MPU_Accel_Range_2G:
		return 16384.0f;
	case MPU_Accel_Range_4G:
		return 8192.0f;
	case MPU_Accel_Range_8G:
		return 4096.0f;
	case MPU_Accel_Range_16G:
		return 2048.0f;
	default:
		return 2048.0f;
	}
}
/**
 * Get the Gyroscope range value
 */
float MPU_get_gyroRange(void) {
	switch (gyroRange) {
	case MPU_Gyro_Range_250:
		return 131;
	case MPU_Gyro_Range_500:
		return 65.5;
	case MPU_Gyro_Range_1000:
		return 32.8;
	case MPU_Gyro_Range_2000:
		return 16.4;
	default:
		return 16.4;
	}
}


