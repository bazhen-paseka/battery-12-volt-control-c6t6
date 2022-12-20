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
#include "i2c.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

	#include "stdio.h"
	#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
	#define 	MY_DEBUG
	#define 	UART_DEBUG		&huart1
	#define 	SOFT_VERSION	1101

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	uint8_t alarma = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

	void UartDebug(char* _text) ;
	void StmSleep(void) 		;
	void StmStop(void) 			;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	char DataChar[100];
	int soft_version_arr_int[3];
	soft_version_arr_int[0] = ((SOFT_VERSION) / 1000) %10 ;
	soft_version_arr_int[1] = ((SOFT_VERSION) /   10) %100 ;
	soft_version_arr_int[2] = ((SOFT_VERSION)       ) %10 ;

	sprintf(DataChar,"\r\n\r\n\tBattery 12 Volt control v%d.%02d.%d " ,
	soft_version_arr_int[0] , soft_version_arr_int[1] , soft_version_arr_int[2] );
	UartDebug(DataChar) ;

	#define DATE_as_int_str 	(__DATE__)
	#define TIME_as_int_str 	(__TIME__)
	sprintf(DataChar,"\r\n\tBuild: %s. Time: %s." , DATE_as_int_str , TIME_as_int_str ) ;
	UartDebug(DataChar) ;

	sprintf(DataChar,"\r\n\tFor debug: UART1-115200/8-N-1\r\n" );
	UartDebug(DataChar) ;
	int counter = 0;
	//		ADC1_Init(&hadc1, ADC_CHANNEL) ;
	//		uint32_t adc1_init_value;
	//		sprintf(DataChar,"ADC for blink: "); UartDebug(DataChar) ;
	//		adc1_init_value = ADC1_GetValue( &hadc1, ADC_CHANNEL ) ;
	//		adc1_init_value = (1000 * adc1_init_value) / ADC_COEFFICIENT;
	//		sprintf(DataChar, "%lu.%02luV, ", adc1_init_value/100, adc1_init_value%100 ); UartDebug(DataChar) ;
	//		sprintf(DataChar,"\r\n"); UartDebug(DataChar) ;

	RTC_TimeTypeDef TimeSt = { 0 } ;
	HAL_RTC_GetTime(&hrtc, &TimeSt, RTC_FORMAT_BIN);
	sprintf(DataChar,"RTC Time: %02d:%02d:%02d \r\n",TimeSt.Hours, TimeSt.Minutes, TimeSt.Seconds );
	UartDebug(DataChar);
	alarma = 1 ;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	sprintf(DataChar,"%d ", counter++ ); UartDebug(DataChar);
	//HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
	HAL_Delay(1000);
	//HAL_IWDG_Refresh(&hiwdg);

	if (alarma == 1) {
		//HAL_IWDG_Refresh(&hiwdg);
		RTC_TimeTypeDef TimeSt = { 0 } ;
		HAL_RTC_GetTime(&hrtc, &TimeSt, RTC_FORMAT_BIN);
		//sprintf(DataChar,"RTC  time: %02d:%02d:%02d\r\n",TimeSt.Hours, TimeSt.Minutes, TimeSt.Seconds ); UartDebug(DataChar) ;
		RTC_AlarmTypeDef AlarmSt = {0};
		AlarmSt.Alarm = 0;
		AlarmSt.AlarmTime.Hours   = TimeSt.Hours 		;
		AlarmSt.AlarmTime.Minutes = TimeSt.Minutes + 0	;
		AlarmSt.AlarmTime.Seconds = TimeSt.Seconds + 5	;
		sprintf(DataChar,"set alarm: %02d:%02d:%02d ",AlarmSt.AlarmTime.Hours, AlarmSt.AlarmTime.Minutes, AlarmSt.AlarmTime.Seconds ); UartDebug(DataChar) ;
		HAL_StatusTypeDef alarm_status= HAL_RTC_SetAlarm_IT(&hrtc, &AlarmSt, RTC_FORMAT_BIN);
		sprintf(DataChar," (status: %d) \r\n", alarm_status); UartDebug(DataChar);
		alarma = 0;
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void UartDebug(char* _text) {
#ifdef MY_DEBUG
	HAL_UART_Transmit(UART_DEBUG, (uint8_t*)_text, strlen(_text), 100);
#endif
} //**************************************************************************

void StmSleep(void) {
#ifdef STOP_PRINT
	sprintf(DataChar, "sleep.. "); UartDebug(DataChar) ;
#endif
	#ifdef MASTER
		HAL_IWDG_Refresh(&hiwdg);
	#endif
    HAL_SuspendTick();
	HAL_PWR_EnterSLEEPMode(	PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI );	//	Sec in Sec
	// -> Sleep MODE <- //
	SystemClock_Config();
	HAL_ResumeTick();
#ifdef STOP_PRINT
	sprintf(DataChar, "^ "); UartDebug(DataChar) ;
#endif
} //**************************************************************************

void StmStop(void) {
#ifdef STOP_PRINT
	sprintf(DataChar, "Stop mode...\r\n\r\n"); UartDebug(DataChar) ;
#endif
#ifdef MASTER
	HAL_IWDG_Refresh(&hiwdg);
#endif
    HAL_SuspendTick();
	HAL_PWR_EnterSTOPMode(	PWR_LOWPOWERREGULATOR_ON,	PWR_STOPENTRY_WFI  );
	// -> STOP MODE <- //
    SystemClock_Config();
	HAL_ResumeTick();
#ifdef STOP_PRINT
	sprintf(DataChar, "WakeUp.\r\n"); UartDebug(DataChar) ;
#endif
} //**************************************************************************

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {
	char		_text[40]	= { 0 } ;
	sprintf(_text," AlarmA: ") ;
	HAL_UART_Transmit(&huart1, (uint8_t*)_text, strlen(_text), 100);

	RTC_TimeTypeDef TimeSt = { 0 } ;
	HAL_RTC_GetTime(hrtc, &TimeSt, RTC_FORMAT_BIN);

	sprintf(_text,"%02d:%02d:%02d\r\n",TimeSt.Hours, TimeSt.Minutes, TimeSt.Seconds );
	UartDebug(_text);
	alarma = 1;
} //**************************************************************************

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
