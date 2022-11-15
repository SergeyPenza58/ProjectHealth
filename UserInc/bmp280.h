/*
 * bmp280.h
 *
 *  Created on: 13 нояб. 2022 г.
 *      Author: gsa
 */

#ifndef BMP280_H_
#define BMP280_H_

#include "main.h"

typedef struct
{
	uint8_t isInit; 	/*<Store flag that sensor is init*/
	uint16_t dig_T1; 	/*< Compensation parameter T1 for temperature defined in production time*/
	int16_t dig_T2;		/*< Compensation parameter T2 for temperature defined in production time*/
	int16_t dig_T3;		/*< Compensation parameter T3 for temperature defined in production time*/

	uint16_t dig_P1;		/*< Compensation parameter P1 for pressure defined in production time*/
	int16_t dig_P2;		/*< Compensation parameter P2 for pressure defined in production time*/
	int16_t dig_P3;		/*< Compensation parameter P3 for pressure defined in production time*/
	int16_t dig_P4;		/*< Compensation parameter P4 for pressure defined in production time*/
	int16_t dig_P5;		/*< Compensation parameter P5 for pressure defined in production time*/
	int16_t dig_P6;		/*< Compensation parameter P6 for pressure defined in production time*/
	int16_t dig_P7;		/*< Compensation parameter P7 for pressure defined in production time*/
	int16_t dig_P8;		/*< Compensation parameter P8 for pressure defined in production time*/
	int16_t dig_P9;		/*< Compensation parameter P9 for pressure defined in production time*/

	int16_t temperature_C; 	/*<temperature value in degree C*/
	int16_t pressure_mm;		/*<pressure value in mHg*/

	int32_t temperature_ADC; /*<temperature ADC read code*/
	int32_t pressure_ADC;	 /*<pressure ADC read code*/

	int32_t t_fine;			 /*< carries fine temperature as*/
}BMP280;

HAL_StatusTypeDef BMP280_Init(I2C_HandleTypeDef *hi2c, BMP280 *bmp280);
HAL_StatusTypeDef BMP280_GetNewValue(I2C_HandleTypeDef* hi2c, BMP280 *bmp280);


#endif /* BMP280_H_ */
