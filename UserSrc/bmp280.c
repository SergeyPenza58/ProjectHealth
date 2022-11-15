/*
 * bmp280.c
 *
 *  Created on: 13 нояб. 2022 г.
 *      Author: gsa
 */

#include "bmp280.h"

//---------------------- Define ----------------------------
/*Slave address 0x76 (SDO PIN connected to GND)*/
/*Address need shift left to 1 byte before calling the HAL Function */
#define BMP280_I2C_ADDRESS			(0x76 << 1)

/* timeout value for I2C operation read/write*/
#define READ_TIMEOUT 				(100)

/* Compensation start address register T1*/
#define COMPENSATION_ADDRESS_T1		(0x88)

/* Pressure start address register*/
#define PRESSURE_ADDRESS 			(0xF7)

/*  minimum output ADC value temperature*/
#define BMP280_ADC_T_MIN                  (0x00000)

/* maximum 20-bit output ADC value without over sampling  temperature*/
#define BMP280_ADC_T_MAX                  (0xFFFF0)

/* minimum output ADC value pressure */
#define BMP280_ADC_P_MIN                  (0x00000)

/*maximum 20-bit output cADC value without over sampling pressure*/
#define BMP280_ADC_P_MAX                  (0xFFFF0)

//----------- Function prototype ------------------

uint8_t BMP280_CheckValue_TemperaturePressure(int32_t rTemperature,
		int32_t rPressure);
void BMP280_GetTemperature(BMP280 *bmp280);
void BMP280_GetPressure(BMP280 *bmp280);

/**
 * @brief BMP280 Initialization
 *
 * @param hi2c: I2C handle pointer
 *
 * @param BMP280 *bmp280 - pointer to struct @BMP280
 *
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef BMP280_Init(I2C_HandleTypeDef *hi2c, BMP280 *bmp280)
{

	// Compensation parameters
	// 3 for temperature
	// 9 for pressure
	// all is 2 bytes
	// (3 + 9)*2 = 24
	uint8_t rxBuf[24];

	// First parameter is T1
	uint8_t startRegister = COMPENSATION_ADDRESS_T1;

	HAL_StatusTypeDef result = HAL_I2C_Master_Transmit(hi2c, BMP280_I2C_ADDRESS,
			&startRegister, 1, READ_TIMEOUT);

	if (result == HAL_OK)
	{

		// Starting address was successfully tx
		// Read pressure and temperature
		// Sensor increment address auto
		result = HAL_I2C_Master_Receive(hi2c, BMP280_I2C_ADDRESS, rxBuf, 24,
		READ_TIMEOUT);

		if (result == HAL_OK)
		{

			bmp280->dig_T1 = (uint16_t) ((uint16_t) rxBuf[0]
					| ((uint16_t) rxBuf[1] << 8));
			bmp280->dig_T2 = (int16_t) ((uint16_t) rxBuf[2]
					| ((uint16_t) rxBuf[3] << 8));
			bmp280->dig_T3 = (int16_t) ((uint16_t) rxBuf[4]
					| ((uint16_t) rxBuf[5] << 8));

			bmp280->dig_P1 = (uint16_t) ((uint16_t) rxBuf[6]
					| ((uint16_t) rxBuf[7] << 8));
			bmp280->dig_P2 = (int16_t) ((uint16_t) rxBuf[8]
					| ((uint16_t) rxBuf[9] << 8));
			bmp280->dig_P3 = (int16_t) ((uint16_t) rxBuf[10]
					| ((uint16_t) rxBuf[11] << 8));
			bmp280->dig_P4 = (int16_t) ((uint16_t) rxBuf[12]
					| ((uint16_t) rxBuf[13] << 8));
			bmp280->dig_P5 = (int16_t) ((uint16_t) rxBuf[14]
					| ((uint16_t) rxBuf[15] << 8));
			bmp280->dig_P6 = (int16_t) ((uint16_t) rxBuf[16]
					| ((uint16_t) rxBuf[17] << 8));
			bmp280->dig_P7 = (int16_t) ((uint16_t) rxBuf[18]
					| ((uint16_t) rxBuf[19] << 8));
			bmp280->dig_P8 = (int16_t) ((uint16_t) rxBuf[20]
					| ((uint16_t) rxBuf[21] << 8));
			bmp280->dig_P9 = (int16_t) ((uint16_t) rxBuf[22]
					| ((uint16_t) rxBuf[23] << 8));

			bmp280->isInit = 1;
			return result;
		}
	}

	bmp280->isInit = 0;
	return result;
}

/**
 * @brief Do exchange with sensor and update value temperature and pressure
 *
 * @param hi2c: I2C handle pointer
 *
 * @param BMP280 *bmp280 - pointer to struct @BMP280
 *
 *
 *
 * @retval HAL_StatusTypeDef "HAL_OK" if exchange with sensor is OK
 */
HAL_StatusTypeDef BMP280_GetNewValue(I2C_HandleTypeDef *hi2c, BMP280 *bmp280)
{
	uint8_t rxBuf[6]; 					// 3 bytes of pressure and 3 temperature
	uint8_t startRegister = PRESSURE_ADDRESS;

	HAL_StatusTypeDef result;

	result = HAL_I2C_Master_Transmit(hi2c, BMP280_I2C_ADDRESS, &startRegister,
			1, READ_TIMEOUT);

	if (result == HAL_OK)
	{
		// Starting address was successfully tx
		// Read pressure and temperature
		// Sensor increment address auto
		result = HAL_I2C_Master_Receive(hi2c, BMP280_I2C_ADDRESS, rxBuf, 6,
		READ_TIMEOUT);

		if (result == HAL_OK)
		{
			// Pressure is 20 bit value
			bmp280->pressure_ADC = (int32_t) ((((uint32_t) (rxBuf[0])) << 12)
					| (((uint32_t) (rxBuf[1])) << 4)
					| ((uint32_t) rxBuf[2] >> 4));

			bmp280->temperature_ADC = (int32_t) ((((uint32_t) (rxBuf[3])) << 12)
					| (((uint32_t) (rxBuf[4])) << 4)
					| ((uint32_t) rxBuf[5] >> 4));

			if (BMP280_CheckValue_TemperaturePressure(bmp280->temperature_ADC,
					bmp280->pressure_ADC) == 1)
			{
				BMP280_GetTemperature(bmp280);
				BMP280_GetPressure(bmp280);

				return HAL_OK;
			}

		}
	}

	bmp280->temperature_C = 0;
	bmp280->pressure_mm = 0;

	return result;
}

/**
 * @brief Check the read value temperature and pressure to over sampling
 *
 *
 * @param int32_t rTemperature - read value temperature from the sensor
 *
 * @paramint32_t rPressure - read value pressure from the sensor
 *
 * @retval uint8_t "1" if data is correct
 */
uint8_t BMP280_CheckValue_TemperaturePressure(int32_t rTemperature,
		int32_t rPressure)
{

	if ((rTemperature >= BMP280_ADC_T_MIN) && (rTemperature <= BMP280_ADC_T_MAX)
			&& (rPressure >= BMP280_ADC_P_MIN)
			&& (rPressure <= BMP280_ADC_P_MAX))
	{
		// Read value is correct
		return 1;
	}

	return 0;
}

/**
 * @brief Convert read ADC value to real temperature
 * @note   Value will be saved in @temperature_C
 *
 *  @param BMP280 *bmp280 - pointer to struct @BMP280
 *
 * @retval none
 */
void BMP280_GetTemperature(BMP280 *bmp280)
{
	int32_t var1, var2;

	// Let's convert the ADC code for real temperature in accordance with the compensation,
	// see section 3.11.3, technical description.
	var1 = ((((bmp280->temperature_ADC / 8) - ((int32_t) bmp280->dig_T1 << 1)))
			* ((int32_t) bmp280->dig_T2)) / 2048;

	var2 = (((((bmp280->temperature_ADC / 16) - ((int32_t) bmp280->dig_T1))
			* ((bmp280->temperature_ADC / 16) - ((int32_t) bmp280->dig_T1)))
			/ 4096) * ((int32_t) bmp280->dig_T3)) / 16384;

	// Real temperature

	bmp280->t_fine = var1 + var2;
	bmp280->temperature_C = ((int16_t) (bmp280->t_fine) / 512.0);
}

/**
 * @brief Convert read ADC value to real pressure
 * @note   Value will be saved in @pressure_Pa
 *
 *  @param BMP280 *bmp280 - pointer to struct @BMP280
 *
 * @retval none
 */
void BMP280_GetPressure(BMP280 *bmp280)
{
	int64_t var1, var2, p;

// Let's convert the ADC code for real pressure in accordance with the compensation,
// see section 3.11.3, technical description.

	var1 = ((int64_t) bmp280->t_fine) - 128000;

	var2 = var1 * var1 * (int64_t) bmp280->dig_P6;

	var2 = var2 + ((var1 * (int64_t) bmp280->dig_P5) << 17);

	var2 = var2 + (((int64_t) bmp280->dig_P4) << 35);

	var1 = ((var1 * var1 * (int64_t) bmp280->dig_P3) >> 8)
			+ ((var1 * (int64_t) bmp280->dig_P2) << 12);

	var1 = (((((int64_t) 1) << 47) + var1)) * ((int64_t) bmp280->dig_P1) >> 33;

	if (var1 == 0)
	{
		return; // avoid exception caused by division by zero
	}

	p = 1048576 - bmp280->pressure_ADC;

	p = (((p << 31) - var2) * 3125) / var1;

	var1 = (((int64_t) bmp280->dig_P9) * (p >> 13) * (p >> 13)) >> 25;

	var2 = (((int64_t) bmp280->dig_P8) * p) >> 19;

	p = ((p + var1 + var2) >> 8) + (((int32_t) bmp280->dig_P7) << 4);

	// 1 Pa = 0.0075006375541921 mmHg
	bmp280->pressure_mm = (int16_t) (p / 256.0 * 0.0075);

}
