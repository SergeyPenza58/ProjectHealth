/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp280.h"
#include "lcd5110.h"
#include <math.h>
#include "usbd_cdc_if.h"
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

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

// define the temperature status
typedef enum
{
	tmp_none = 0,	// Temperature is not defined
	tmp_norma,  	// Temperature is normal
	tmp_warningL,	// Temperature is warning below zero
	tmp_warningH,	// Temperature is warning above zero
	tmp_alarmL,		// Temperature is alarm below zero
	tmp_alarmH,		// Temperature is alarm above zero
} TemperatureStatus;

// define the pressure status
typedef enum
{
	prs_none = 0,	// Pressure is not defined
	prs_norma,  	// Pressure is normal
	prs_warningL,	// Pressure is warning low
	prs_warningH,	// Pressure is warning high
	prs_alarmL,		// Pressure is alarm low
	prs_alarmH,		// Pressure is alarm high
} PressureStatus;

// define the Altitude status
typedef enum
{
	alt_none = 0,	// Altitude is not defined
	alt_norma,  	// Altitude is normal
	alt_warning,	// Altitude is warning
	alt_alarm,		// Altitude is alarm

} AltitudeStatus;

// define the buzzer status
typedef enum
{
	buzzer_off = 0,		//buzzer in off mode
	buzzer_Warning,  	// buzzer in warning mode
	buzzer_Alram,		// buzzer in alarm mode

} BuzzerStatus;

const uint8_t bmp280_cntErr_max = 10; /*<Store the max counter value for error exchange with bmp280*/

const float temper_Warning_L = -30; /*<Store the warning value of temperature below zero*/
const float temper_Alaram_L = -40; /*<Store the warning value of temperature below zero*/

const float temper_Warning_H = 40; /*<Store the warning value of temperature above zero*/
const float temper_Alaram_H = 45; /*<Store the warning value of temperature above zero*/

const float pressure_Warning_H = 760; /*<Store the warning value of pressure high*/
const float pressure_Alaram_H = 770; /*<Store the warning value of pressure high*/

const float pressure_Warning_L = 650; /*<Store the warning value of pressure low*/
const float pressure_Alaram_L = 640; /*<Store the alarm value of pressure low*/

const float altitude_Warning = 3000; /*<Store the warning value of altitude*/
const float altitude_Alaram = 3500; /*<Store the alarm value of altitude*/

uint8_t time_1msIsPassed;
BuzzerStatus buzzerStatus;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
TemperatureStatus Temperature_DoAnalyze(float temperature);
PressureStatus Pressure_DoAnalyze(float pressure);
AltitudeStatus Altitude_DoAnalyze(float altitude);

float Altitude_GetValue(float pressure, float temperature_C);
float Oxygen_GetValue(float pressure);

void Buzzer_Warning_TurnON();
void Buzzer_Alarm_TurnOn();
void Buzzer_TurnOff();

void Terminal_SendData(int16_t temperature, uint16_t pressure,
		uint16_t altitude, uint16_t oxygen);
void LCD_SendData(int16_t temperature, uint16_t pressure, uint16_t altitude,
		uint16_t oxygen, BuzzerStatus buzzerStatus);
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/*
	 * htim1 - PWM signal to buzzer.
	 * hi2c1 - communicate with BMP280 sensor.
	 * hspi1 - communicate with display PCD8544
	 *
	 */
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	// Do delay before init peripheral
	HAL_Delay(2000);
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */

	uint8_t bmp280_cntErr = 0;
	uint8_t time_cnt1ms;
	uint8_t time_cnt100ms;

	TemperatureStatus tmpStatus = tmp_none;
	PressureStatus prsStatus = prs_none;
	AltitudeStatus altStatus = alt_none;

	unsigned char str[] = "Hello User! ";
	float altitude_m; /*<altitude value in meters >*/
	float oxygen; /*< oxygen mmHg Art.*/

	BMP280 myBMP280;

	if (BMP280_Init(&hi2c1, &myBMP280) != HAL_OK)
	{
		// Sensor is error
		// Led Turn ON
		bmp280_cntErr = bmp280_cntErr_max;

		// Turn ON LED,
		HAL_GPIO_WritePin(LED_TEST_GPIO_Port, LED_TEST_Pin, GPIO_PIN_SET);
	}

	LcdInit();
	LcdClear();

	LcdGotoXYFont(1, 1);
	LcdStr(FONT_1X, str);

	LcdUpdate();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		if (time_1msIsPassed == 1)
		{
			time_1msIsPassed = 0;

			// 1 ms is passed

			if (++time_cnt1ms >= 100)
			{
				time_cnt1ms = 0;

				// 100 ms is passed

				if (++time_cnt100ms >= 10)
				{
					time_cnt100ms = 0;

					// 1 second is passed

					if (BMP280_GetNewValue(&hi2c1, &myBMP280) == HAL_OK)
					{
						// Reset counter of errors
						bmp280_cntErr = 0;

						// Change LED state, Exchange with sensor is OK
						HAL_GPIO_TogglePin(LED_TEST_GPIO_Port, LED_TEST_Pin);

						altitude_m = Altitude_GetValue(myBMP280.pressure_mm,
								myBMP280.temperature_C);

						oxygen = Oxygen_GetValue(myBMP280.pressure_mm);

						tmpStatus = Temperature_DoAnalyze(
								myBMP280.temperature_C);
						prsStatus = Pressure_DoAnalyze(myBMP280.pressure_mm);

						altStatus = Altitude_DoAnalyze(altitude_m);

						if ((tmpStatus > tmp_norma) || (prsStatus > prs_norma)
								|| (altStatus > alt_norma))
						{
							if ((tmpStatus >= tmp_alarmL)
									|| (prsStatus >= prs_alarmL)
									|| (altStatus >= alt_alarm))
							{
								// we have a Alarm situation
								Buzzer_Alarm_TurnOn();
								
							}
							else
							{
								// we have a Warning situation
								Buzzer_Warning_TurnON();
							}
						}
						else
						{
							// Value temperature and pressure is normal
							Buzzer_TurnOff();
						}

					}
					else
					{
						if (++bmp280_cntErr >= bmp280_cntErr_max)
						{
							bmp280_cntErr = bmp280_cntErr_max;
							// Turn ON LED, Exchange with sensor is ERROR
							HAL_GPIO_WritePin(LED_TEST_GPIO_Port,
							LED_TEST_Pin, GPIO_PIN_SET);

							// Do reinit communication with sensor
							BMP280_Init(&hi2c1, &myBMP280);

							// Turn off buzzer, we don't know value
							Buzzer_TurnOff();
						}
					}

					Terminal_SendData(myBMP280.temperature_C,
							myBMP280.pressure_mm, altitude_m, oxygen);
					LCD_SendData(myBMP280.temperature_C, myBMP280.pressure_mm,
							altitude_m, oxygen, buzzerStatus);

				}
			}
		}
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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit =
	{ 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

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

	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };
	TIM_OC_InitTypeDef sConfigOC =
	{ 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig =
	{ 0 };

	/* USER CODE BEGIN TIM1_Init 1 */
	/*
	 * Fckltim 72 MHz
	 *
	 * Fckl buzzer = 4 kHz
	 *
	 *  Period = 72 MHz / 4kHz = 18000
	 */
	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 18000;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_TEST_GPIO_Port, LED_TEST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
	LCD_CE_Pin | LCD_DC_Pin | LCD_Reset_Pin | SPI1_CS_DISP_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_TEST_Pin */
	GPIO_InitStruct.Pin = LED_TEST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_TEST_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_CE_Pin LCD_DC_Pin LCD_Reset_Pin SPI1_CS_DISP_Pin */
	GPIO_InitStruct.Pin = LCD_CE_Pin | LCD_DC_Pin | LCD_Reset_Pin
			| SPI1_CS_DISP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
 * @brief  Perform analyze value of temperature
 *
 * @param float temperature - current value of temperature
 *
 * @retval @TemperatureStatus
 */
TemperatureStatus Temperature_DoAnalyze(float temperature)
{
	if (temperature > 0)
	{
		if (temperature >= temper_Alaram_H)
		{
			// Alarm high temperature
			return tmp_alarmH;
		}
		else
		{
			if (temperature >= temper_Warning_H)
			{
				// warning high temperature
				return tmp_warningH;
			}
		}
	}
	else
	{
		if (temperature <= temper_Alaram_L)
		{
			// Alarm low temperature
			return tmp_alarmL;
		}
		else
		{
			if (temperature <= temper_Warning_L)
			{
				// warning  low temperature

				return tmp_warningL;
			}
		}
	}

	// Temperature is normal

	return tmp_norma;
}

/**
 * @brief  Perform analyze value of pressure
 *
 * @param float pressure - current value of pressure
 *
 * @retval @PressureStatus
 */
PressureStatus Pressure_DoAnalyze(float pressure)
{
	if (pressure > 0)
	{
		if (pressure >= pressure_Alaram_H)
		{
			// Alarm high pressure
			return prs_alarmH;
		}
		else
		{
			if (pressure >= pressure_Warning_H)
			{
				// warning high pressure
				return prs_warningH;
			}
		}
	}
	else
	{
		if (pressure <= pressure_Alaram_L)
		{
			// Alarm low pressure
			return prs_alarmL;
		}
		else
		{
			if (pressure <= pressure_Warning_L)
			{
				// warning  low pressure

				return prs_warningL;
			}
		}
	}

	// pressure is normal

	return tmp_norma;
}

/**
 * @brief  Perform analyze value of Altitude
 *
 * @param float altitude - current value of pressure
 *
 * @retval @AltitudeStatus
 */
AltitudeStatus Altitude_DoAnalyze(float altitude)
{
	if (altitude >= altitude_Alaram)
	{
		return alt_alarm;
	}
	else
	{
		if (altitude >= altitude_Warning)
		{
			return alt_warning;
		}
	}

	return alt_norma;
}
/**
 * @brief  Calculate Altitude according to pressure and temperature
 *
 * @param float pressure - current pressure (mmHg)
 *
 * @param float temperature_C - current temperature (C)
 *
 * @retval Altitude float value
 */
float Altitude_GetValue(float pressure, float temperature_C)
{
	// P= P0*e^((-M * g * h)/ (R * T)
	// T - absolute air temperature
	// R - universal gas constant, R = 8.31 J/mol K;
	// h - height, m
	// g - free fall acceleration, g = 9.81 m/s²;
	// M - molar mass of dry air, M = 0.029 kg/mol;
	// P0 - pressure at sea level [Pa];  1 mmHg = 133.322 Pa 760 mm =1 01324.72
	// P - current @pressure_Pa

	float coef1 = (-0.029 * 9.81) / (8.31 * (temperature_C + 273.15));

	float myln = log(pressure / 760.0);

	return (myln / coef1);
}

/**
 * @brief  Calculate Oxygen  according to pressure
 *
 * @param float pressure - current pressure (mmHg Art)
 *
 *
 * @retval Oxygen float value mmHg Art
 */
float Oxygen_GetValue(float pressure)
{
	// oxygen is at any height 20.9%
	return (pressure * 20.9 / 100.0);
}

/**
 * @brief  Turn on buzzer warning signal
 * @retval None
 */
void Buzzer_Warning_TurnON()
{
	//Check the buzzer status
	if (buzzerStatus != buzzer_Warning)
	{
		buzzerStatus = buzzer_Warning;
		/*
		 * Fckltim 72 MHz
		 *
		 * Fckl buzzer = 4 kHz
		 *
		 *  Period = 72 MHz / 4kHz = 18000
		 */
		// Set duty 20% for PWM signal on buzzer  and 4 kHz
		// duty cycle determines the loudness. 50% duty cycle 100% volume
		TIM1->ARR = 18000;
		TIM1->CCR1 = TIM1->ARR - (uint32_t) ((float) TIM1->ARR * 0.2);

		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	}
}

/**
 * @brief Turn on buzzer Alarm signal
 * @retval None
 */
void Buzzer_Alarm_TurnOn()
{
	// Check the buzzer status
	if (buzzerStatus != buzzer_Alram)
	{
		buzzerStatus = buzzer_Alram;
		/*
		 * Fckltim 72 MHz
		 *
		 * Fckl buzzer = 6 kHz
		 *
		 *  Period = 72 MHz / 6kHz = 12000
		 */
		// Set duty 40% for PWM signal on buzzer and 6 kHz
		// duty cycle determines the loudness. 50% duty cycle 100% volume
		//
		TIM1->ARR = 12000;
		TIM1->CCR1 = TIM1->ARR - (uint32_t) ((float) TIM1->ARR * 0.4);

		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	}
}

/**
 * @brief Turn off buzzer
 * @retval None
 */
void Buzzer_TurnOff()
{
	buzzerStatus = buzzer_off;
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	TIM1->CCR1 = 0;

}

/**
 * @brief Transmint data from usb
 *
 * @param int16_t temperature - current value of temperature
 *
 * @param uint16_t pressure - current value of pressure
 *
 * @param uint16_t altitude - current value of altitude
 *
 * @param uint16_t oxygen - current value of oxygen
 *
 * @retval None
 */
void Terminal_SendData(int16_t temperature, uint16_t pressure,
		uint16_t altitude, uint16_t oxygen)
{
	uint8_t tempData[6];

	// Tx temperature value
	uint8_t tempName[] = "Temperature is : ";

	CDC_Transmit_FS(tempName, sizeof(tempName));
	sprintf(tempData, "%d", temperature);
	tempData[5] = '\n';
	CDC_Transmit_FS(tempData, sizeof(tempData));

	// Tx pressure value
	sprintf(tempName, "%c", "Pressure is:");
	CDC_Transmit_FS(tempName, sizeof(tempName));
	sprintf(tempData, "%d", pressure);
	tempData[5] = '\n';
	CDC_Transmit_FS(tempData, sizeof(tempData));

	// Tx altitude value
	sprintf(tempName, "%c", "Altitude is:");
	CDC_Transmit_FS(tempName, sizeof(tempName));
	sprintf(tempData, "%d", altitude);
	tempData[5] = '\n';
	CDC_Transmit_FS(tempData, sizeof(tempData));

	// Tx oxygen value
	sprintf(tempName, "%c", "Oxygen is:");
	CDC_Transmit_FS(tempName, sizeof(tempName));
	sprintf(tempData, "%d", oxygen);
	tempData[5] = '\n';
	CDC_Transmit_FS(tempData, sizeof(tempData));

}

/**
 * @brief Transmint data from LCD
 *
 * @param int16_t temperature - current value of temperature
 *
 * @param uint16_t pressure - current value of pressure
 *
 * @param uint16_t altitude - current value of altitude
 *
 * @param uint16_t oxygen - current value of oxygen
 *
 * @param BuzzerStatus buzzerStatus - buzzer status
 *
 * @retval None
 */
void LCD_SendData(int16_t temperature, uint16_t pressure, uint16_t altitude,
		uint16_t oxygen, BuzzerStatus buzzerStatus)
{
	unsigned char str[16];

	LcdClear();

	// Print Buzzer values on LCD

	if (buzzerStatus == buzzer_Warning)
	{
		LcdGotoXYFont(0, 0);
		LcdStr(FONT_1X, (unsigned char*) "  WARNING!!!");

	}
	else
	{
		if (buzzerStatus == buzzer_Alram)
		{
			LcdGotoXYFont(0, 0);
			LcdStr(FONT_1X, (unsigned char*) "  ALARM!!!");
		}
	}

	// Print Temperature values on LCD
	LcdGotoXYFont(0, 1);
	LcdStr(FONT_1X, (unsigned char*) "Temperat");

	LcdGotoXYFont(10, 1);
	sprintf(str, "%d", temperature);
	LcdStr(FONT_1X, str);

	// Print Pressure values on LCD
	LcdGotoXYFont(0, 2);
	LcdStr(FONT_1X, (unsigned char*) "Pressure");

	LcdGotoXYFont(10, 2);
	sprintf(str, "%d", pressure);
	LcdStr(FONT_1X, str);

	// Print Altitude values on LCD
	LcdGotoXYFont(0, 3);
	LcdStr(FONT_1X, (unsigned char*) "Altitude");

	LcdGotoXYFont(10, 3);
	sprintf(str, "%d", altitude);
	LcdStr(FONT_1X, str);

	// Print Oxygen values on LCD
	LcdGotoXYFont(0, 4);
	LcdStr(FONT_1X, (unsigned char*) "Oxygen");

	LcdGotoXYFont(10, 4);
	sprintf(str, "%d", oxygen);
	LcdStr(FONT_1X, str);

	LcdUpdate();
}
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
