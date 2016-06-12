#include "stm32f4xx_hal.h"
#include <imu.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#define LIS3MDL_ADDR	0x1E
#define LSM6DS0_ADDR	0x6b



static int lis3mdl_WriteReg(uint8_t reg, int8_t val)
{
	return HAL_I2C_Mem_Write(&hi2c1, LIS3MDL_ADDR, reg, 1, (uint8_t*)&val, sizeof(val), 100);
}

static int lis3mdl_ReadReg(uint8_t reg, int8_t *val)
{
	return HAL_I2C_Mem_Read(&hi2c1, LIS3MDL_ADDR, reg, 1, (uint8_t*)val, sizeof(*val), 100);
}

static int lis3mdl_ReadReg16(uint8_t reg, int16_t *val)
{
	return HAL_I2C_Mem_Read(&hi2c1, LIS3MDL_ADDR, reg, 1, (uint8_t*)val, sizeof(*val), 100);
}


static int lis3mdl_Initialize(uint32_t freq)
{

	/* Verify ID */
	int8_t tmp1;
	lis3mdl_ReadReg(LIS3MDL_WHO_AM_I, &tmp1);
	printf("LIS3MDL ID 0x%x\r\n", tmp1);

	/* OM = 11 (ultra-high-performance mode for X and Y); DO = 100 (10 Hz ODR) */
	lis3mdl_WriteReg(LIS3MDL_CTRL_REG1, 0x70);
	/* FS = 00 (+/- 4 gauss full scale) */
	lis3mdl_WriteReg(LIS3MDL_CTRL_REG2, 0x00);

	/* MD = 00 (continuous-conversion mode) */
	lis3mdl_WriteReg(LIS3MDL_CTRL_REG3, 0x00);

	/* OMZ = 11 (ultra-high-performance mode for Z) */
	lis3mdl_WriteReg(LIS3MDL_CTRL_REG4, 0x0C);
	return 0;
}

static int lis3mdl_ReadValues(IMUData_t *data)
{
	lis3mdl_ReadReg16(LIS3MDL_OUT_X_L, &data->mag.x);
	lis3mdl_ReadReg16(LIS3MDL_OUT_Y_L, &data->mag.y);
	lis3mdl_ReadReg16(LIS3MDL_OUT_Z_L, &data->mag.z);
	return 0;
}


static int lsm6ds0_WriteReg(uint8_t reg, int8_t val)
{
	return HAL_I2C_Mem_Write(&hi2c1, LSM6DS0_ADDR, reg, 1, (uint8_t*)&val, sizeof(val), 100);
}

static int lsm6ds0_ReadReg(uint8_t reg, int8_t *val)
{
	return HAL_I2C_Mem_Read(&hi2c1, LSM6DS0_ADDR, reg, 1, (uint8_t*)val, sizeof(*val), 100);
}

static int lsm6ds0_ReadReg16(uint8_t reg, int16_t *val)
{
	return HAL_I2C_Mem_Read(&hi2c1, LSM6DS0_ADDR, reg, 1, (uint8_t*)val, sizeof(*val), 100);
}

static int lsm6ds0_ReadValues(IMUData_t *data)
{
	lsm6ds0_ReadReg16(LSM6DS0_XG_OUT_X_L_G | (1 << 7), &data->gyro.x);
	lsm6ds0_ReadReg16(LSM6DS0_XG_OUT_Y_L_G | (1 << 7), &data->gyro.y);
	lsm6ds0_ReadReg16(LSM6DS0_XG_OUT_Z_L_G | (1 << 7), &data->gyro.z);

	lsm6ds0_ReadReg16(LSM6DS0_XG_OUT_X_L_XL | (1 << 7), &data->acc.x);
	lsm6ds0_ReadReg16(LSM6DS0_XG_OUT_Y_L_XL | (1 << 7), &data->acc.y);
	lsm6ds0_ReadReg16(LSM6DS0_XG_OUT_Z_L_XL | (1 << 7), &data->acc.z);

	return 0;
}

static int lsm6ds0_Initialize(uint32_t freq)
{

	/* gyro init */
	int8_t tmp1;

	/* Verify ID */
	lsm6ds0_ReadReg(LSM6DS0_XG_WHO_AM_I_ADDR, &tmp1);
	printf("LSM6D0 ID 0x%x\r\n", tmp1);
	lsm6ds0_ReadReg(LSM6DS0_XG_CTRL_REG1_G, &tmp1);
	/* Output Data Rate selection */
	tmp1 &= ~(LSM6DS0_G_ODR_MASK);
	tmp1 |= LSM6DS0_G_ODR_119HZ;
	/* Full scale selection */
	tmp1 &= ~(LSM6DS0_G_FS_MASK);
	tmp1 |= LSM6DS0_G_FS_2000;
	lsm6ds0_WriteReg(LSM6DS0_XG_CTRL_REG1_G, tmp1);

	lsm6ds0_ReadReg(LSM6DS0_XG_CTRL_REG4, &tmp1);
	/* Enable X axis selection */
	tmp1 &= ~(LSM6DS0_G_XEN_MASK);
	tmp1 |= LSM6DS0_G_XEN_ENABLE;
	/* Enable Y axis selection */
	tmp1 &= ~(LSM6DS0_G_YEN_MASK);
	tmp1 |= LSM6DS0_G_YEN_ENABLE;
	/* Enable Z axis selection */
	tmp1 &= ~(LSM6DS0_G_ZEN_MASK);
	tmp1 |= LSM6DS0_G_ZEN_ENABLE;
	lsm6ds0_WriteReg(LSM6DS0_XG_CTRL_REG4,tmp1);

	/******************************/
	/***** Accelerometer init *****/

	lsm6ds0_ReadReg(LSM6DS0_XG_CTRL_REG6_XL, &tmp1);
	/* Output Data Rate selection */
	tmp1 &= ~(LSM6DS0_XL_ODR_MASK);
	tmp1 |= LSM6DS0_XL_ODR_119HZ;

	/* Full scale selection */
	tmp1 &= ~(LSM6DS0_XL_FS_MASK);
	tmp1 |= LSM6DS0_XL_FS_2G;
	lsm6ds0_WriteReg(LSM6DS0_XG_CTRL_REG6_XL,tmp1);

	lsm6ds0_ReadReg(LSM6DS0_XG_CTRL_REG5_XL, &tmp1);
	/* Enable X axis selection */
	tmp1 &= ~(LSM6DS0_XL_XEN_MASK);
	tmp1 |= LSM6DS0_XL_XEN_ENABLE;

	/* Enable Y axis selection */
	tmp1 &= ~(LSM6DS0_XL_YEN_MASK);
	tmp1 |= LSM6DS0_XL_YEN_ENABLE;

	/* Enable Z axis selection */
	tmp1 &= ~(LSM6DS0_XL_ZEN_MASK);
	tmp1 |= LSM6DS0_XL_ZEN_ENABLE;
	lsm6ds0_WriteReg(LSM6DS0_XG_CTRL_REG5_XL,tmp1);
	return 0;
}

static int IMU_Initialize(uint32_t freq)
{
	lis3mdl_Initialize(freq);
	lsm6ds0_Initialize(freq);

	return 0;
}

void IMUTaskThread(void *arg)
{

	/* Initialize the IMUs */
	IMU_Initialize(100);

	/* Start the IMUs */

	/* Forever */
	while (1) {
		IMUData_t data;

		/* Wait for data ready from compass */
		if (pdTRUE == xSemaphoreTake(&drdyCompass, portMAX_DELAY)) {

			/* Fetch data */
			data.time = xTaskGetTickCount();
			lsm6ds0_ReadValues(&data);
			lis3mdl_ReadValues(&data);

			/* queue data */
			xQueueSend(&qImuToFusion, &data, 10);
		}
	}

}


