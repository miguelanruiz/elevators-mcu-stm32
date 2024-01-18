/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMER_FINISHED 0
#define SIZE_QUEUE 10
#define SIZE_QUEUE_KEYBOARD 10
#define SIZE_PAGE_ASCENSOR 16
#define SIZE_PAGE_DOORS 2
#define SIZE_LOCKED_BUTTONS 2
#define SIZE_DATA_REPORT (SIZE_PAGE_ASCENSOR*2)+SIZE_PAGE_DOORS
#define DEFAULT_BASE_FLOOR 3
#define DEFAULT_START_MCU_FLOOR 3
#define CONST_TIME_MOVEMENT 3000
#define CONST_DOORS_MOVEMENT 8000
#define G_P 0x107   /* x^8 + x^2 + x + 1 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
volatile uint16_t counter = 0;
volatile uint8_t firstAscensorReport[SIZE_PAGE_ASCENSOR];
volatile uint8_t secondAscensorReport[SIZE_PAGE_ASCENSOR];
volatile uint8_t lockedKeyboardButtons[SIZE_LOCKED_BUTTONS];
volatile uint8_t globalDoors[SIZE_PAGE_DOORS];
volatile uint8_t dataReport[SIZE_DATA_REPORT];

enum{
	T0 = 0 ,
	T1,
	T2,
	T3,
	T4,
	T5,
	T6,
	TDOORS_ONE,
	TDOORS_TWO,
	TPRINT,
	TIMERS
};

volatile uint32_t Timers[TIMERS];

typedef enum{
	DETENIDO = 0,
	SUBIENDO = 1,
	BAJANDO = 2,
	ABRIR_PUERTAS = 3,
	CERRAR_PUERTAS = 4
}ASCENSOR;

typedef enum{
	PRINTING = 0,
	WAIT_INFO = 1,
	ERROR_UNIT = 2
}PRINTER_MACHINE;

volatile ASCENSOR firstAscensor = DETENIDO;
volatile ASCENSOR secondAscensor = DETENIDO;
volatile PRINTER_MACHINE printerUnit = PRINTING;

volatile uint8_t currentFloor = DEFAULT_START_MCU_FLOOR;
volatile uint8_t queueFloor[SIZE_QUEUE];
volatile uint8_t firstQueueSelection[SIZE_QUEUE_KEYBOARD];
volatile uint8_t tempFloor = DEFAULT_BASE_FLOOR;

volatile uint8_t currentFloor_second = DEFAULT_START_MCU_FLOOR;
volatile uint8_t queueFloor_second[SIZE_QUEUE];
volatile uint8_t secondQueueSelection[SIZE_QUEUE_KEYBOARD];
volatile uint8_t tempFloor_second = DEFAULT_BASE_FLOOR;

//uint8_t text[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void AscensorUnit();
void AscensorUnit_second();
void PrinterUnit();
uint8_t crc8(uint8_t *message, uint32_t length);
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
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  //HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  for(int i = 0; i < SIZE_QUEUE; i++){
	  queueFloor[i] = DEFAULT_BASE_FLOOR;
  }
  for(int i = 0; i < SIZE_PAGE_ASCENSOR; i++){
	  firstAscensorReport[i] = 0;
  }
  firstAscensorReport[0]='A';
  firstAscensorReport[1]='1';
  firstAscensorReport[2]='S';
  firstAscensorReport[3]='0';
  firstAscensorReport[4]='B';
  firstAscensorReport[5]='0';

  for(int i = 0; i < SIZE_QUEUE; i++){
  	  queueFloor_second[i] = DEFAULT_BASE_FLOOR;
  }
  for(int i = 0; i < SIZE_PAGE_ASCENSOR; i++){
	  secondAscensorReport[i] = 0;
  }
  secondAscensorReport[0]='A';
  secondAscensorReport[1]='2';
  secondAscensorReport[2]='S';
  secondAscensorReport[3]='0';
  secondAscensorReport[4]='B';
  secondAscensorReport[5]='0';

  globalDoors[0] = '0';
  globalDoors[1] = '0';

  lockedKeyboardButtons[0] = '0';
  lockedKeyboardButtons[1] = '4';

  Timers[TPRINT] = 500;
  Timers[TDOORS_ONE] = 0;
  Timers[TDOORS_TWO] = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 /* if ( CDC_Transmit_FS(&counter, sizeof(char)*2)!=USBD_OK ) {
		HAL_GPIO_WritePin(led_GPIO_Port, led_Pin,GPIO_PIN_RESET);
		HAL_Delay(1000);
	}
	  else{
		  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin,GPIO_PIN_SET);
		  HAL_Delay(500);
	  }*/
	  //AscensorUnit();
	  AscensorUnit();
	  AscensorUnit_second();
	  PrinterUnit();
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 35999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : led_Pin */
  GPIO_InitStruct.Pin = led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CHA_Pin CHB_Pin */
  GPIO_InitStruct.Pin = CHA_Pin|CHB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  //UNUSED(htim);
	if(htim->Instance == TIM2){
		if(counter ++ == 1000){
			counter = 0;
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		}
		for(int i = 0; i < TIMERS; i++){
			if(Timers[i] != TIMER_FINISHED)
				Timers[i]--;
		}
	}
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
}
void CDC_ReceiveCallBack(uint8_t *buf, uint32_t len){
	//sprintf(buffer, "Error magnitud\n");
	if(buf[0] == 'A' && buf[1] == '1'){
		if(buf[3] == '1' || buf[5] == '1'){
			if(buf[8] != crc8(buf,8))
				return;
			if(currentFloor == buf[7]-47){
				firstAscensor = ABRIR_PUERTAS;
				return;
			}
			if(currentFloor_second == buf[7]-47){
				secondAscensor = ABRIR_PUERTAS;
				return;
			}
			if(abs(currentFloor-buf[7]-47) < abs(currentFloor_second-buf[7]-47)){
				if(firstAscensor == SUBIENDO && buf[7]-47 > currentFloor && buf[3] == '1'){
						queueFloor[buf[7]-48] = buf[7]-47;
						firstAscensor = ABRIR_PUERTAS;
				}
				else{
						queueFloor_second[buf[7]-48] = buf[7]-47;
						secondAscensor = ABRIR_PUERTAS;
				}
			}
			else{
				if(secondAscensor == BAJANDO && buf[7]-47 < currentFloor_second && buf[5] == '1'){
						queueFloor_second[buf[7]-48] = buf[7]-47;
						secondAscensor = ABRIR_PUERTAS;
				}
				else{
						queueFloor[buf[7]-48] = buf[7]-47;
						firstAscensor = ABRIR_PUERTAS;
				}
			}
		}
	}
	if(buf[0] == 'T' && buf[1] == 'F' && buf[2] == 'S' && buf[3] == '1'){
		if(buf[14] != crc8(buf+4,10)){
			printerUnit = ERROR_UNIT;
			for(int i = 6; i < SIZE_PAGE_ASCENSOR; i++){
				firstAscensorReport[i]='1';
			}
			return;
		}
		for(int i = 0; i < SIZE_QUEUE_KEYBOARD; i++){
			if(buf[i+4] == '1' && queueFloor[i] == DEFAULT_BASE_FLOOR){
				queueFloor[i] = i+1;
			}
		}
	}
	if(buf[0] == 'T' && buf[1] == 'F' && buf[2] == 'S' && buf[3] == '2'){
		if(buf[14] != crc8(buf+4,10)){
			printerUnit = ERROR_UNIT;
			for(int i = 6; i < SIZE_PAGE_ASCENSOR; i++){
				secondAscensorReport[i]='1';
			}
			return;
		}
		for(int i = 0; i < SIZE_QUEUE_KEYBOARD; i++){
			if(buf[i+4] == '1' && queueFloor[i] == DEFAULT_BASE_FLOOR){
				queueFloor_second[i] = i+1;
			}
		}
	}
	//CDC_Transmit_FS(buf, len);
	//CDC_Transmit_FS(constantBuffer, (uint32_t)4);
}

void AscensorUnit(){
	switch(firstAscensor){
	case DETENIDO:{
		for(int i = 0; i < SIZE_QUEUE; i++){
			if(queueFloor[i] > tempFloor){
				tempFloor = queueFloor[i];
			}
		}
		if(tempFloor > currentFloor){
			firstAscensor = SUBIENDO;
			Timers[T0] = CONST_TIME_MOVEMENT;
			return;
		}
		for(int i = 0; i < SIZE_QUEUE; i++){
			if(queueFloor[i] < tempFloor){
				tempFloor = queueFloor[i];
			}
		}
		if(tempFloor < currentFloor){
			firstAscensor = BAJANDO;
			Timers[T1] = CONST_TIME_MOVEMENT;
			return;
		}
		firstAscensorReport[3]='0';
		firstAscensorReport[5]='0';
		for(int i = 6; i < SIZE_PAGE_ASCENSOR; i++){
			firstAscensorReport[i]='0';
		}
		firstAscensorReport[currentFloor+5]='1';
		break;
	}
	case SUBIENDO:{
		if(Timers[T0] == TIMER_FINISHED){
			currentFloor++;
			firstAscensorReport[3]='1';
			firstAscensorReport[5]='0';
			for(int i = 6; i < SIZE_PAGE_ASCENSOR; i++){
				firstAscensorReport[i]='0';
			}
			firstAscensorReport[currentFloor+5]='1';
			//CDC_Transmit_FS(firstAscensorReport, SIZE_PAGE_ASCENSOR); --->
			if(currentFloor == tempFloor){
				for(int i = 0; i < SIZE_QUEUE; i++){
					if(queueFloor[i] == currentFloor){
						queueFloor[i] = DEFAULT_BASE_FLOOR;
					}
				}
				firstAscensor = ABRIR_PUERTAS;
				//CDC_Transmit_FS(constantBufferUp, 7);
				return;
			}
			else{
				for(int i = 0; i < SIZE_QUEUE; i++){
					if(queueFloor[i] == currentFloor){
						firstAscensor = ABRIR_PUERTAS;
						queueFloor[i] = DEFAULT_BASE_FLOOR;
						//CDC_Transmit_FS(constantBufferUp, 7);
					}
				}
			}
			//CDC_Transmit_FS(buf, 5);
			Timers[T0] = CONST_TIME_MOVEMENT;
		}
		break;
	}
	case BAJANDO:{
		if(Timers[T1] == TIMER_FINISHED){
			currentFloor--;
			firstAscensorReport[3]='0';
			firstAscensorReport[5]='1';
			for(int i = 6; i < SIZE_PAGE_ASCENSOR; i++){
				firstAscensorReport[i]='0';
			}
			firstAscensorReport[currentFloor+5]='1';
			//CDC_Transmit_FS(firstAscensorReport, SIZE_PAGE_ASCENSOR); --->
			if(currentFloor == tempFloor){
				for(int i = 0; i < SIZE_QUEUE; i++){
					if(queueFloor[i] == currentFloor){
						queueFloor[i] = DEFAULT_BASE_FLOOR;
					}
				}
				firstAscensor = ABRIR_PUERTAS;
				//CDC_Transmit_FS(constantBufferDown, 7);
				return;
			}
			else{
				for(int i = 0; i < SIZE_QUEUE; i++){
					if(queueFloor[i] == currentFloor){
						firstAscensor = ABRIR_PUERTAS;
						queueFloor[i] = DEFAULT_BASE_FLOOR;
						//CDC_Transmit_FS(constantBufferDown, 7);
					}
				}
			}
			//CDC_Transmit_FS(buf, 5);
			Timers[T1] = CONST_TIME_MOVEMENT;
		}
		break;
	}
	case ABRIR_PUERTAS:{
		globalDoors[0] = '1';
		if(Timers[TDOORS_ONE] == TIMER_FINISHED){
			firstAscensor = CERRAR_PUERTAS;
			Timers[TDOORS_ONE] = CONST_TIME_MOVEMENT;
		}
		break;
	}
	case CERRAR_PUERTAS:{
		if(Timers[TDOORS_ONE] == TIMER_FINISHED){
			firstAscensor = DETENIDO;
			globalDoors[0] = '0';
		}
		break;
	}
	}
}

void AscensorUnit_second(){
	switch(secondAscensor){
	case DETENIDO:{
		for(int i = 0; i < SIZE_QUEUE; i++){
			if(queueFloor_second[i] > tempFloor_second){
				tempFloor_second = queueFloor_second[i];
			}
		}
		if(tempFloor_second > currentFloor_second){
			secondAscensor = SUBIENDO;
			Timers[T2] = CONST_TIME_MOVEMENT;
			return;
		}
		for(int i = 0; i < SIZE_QUEUE; i++){
			if(queueFloor_second[i] < tempFloor_second){
				tempFloor_second = queueFloor_second[i];
			}
		}
		if(tempFloor_second < currentFloor_second){
			secondAscensor = BAJANDO;
			Timers[T3] = CONST_TIME_MOVEMENT;
			return;
		}
		secondAscensorReport[3]='0';
		secondAscensorReport[5]='0';
		for(int i = 6; i < SIZE_PAGE_ASCENSOR; i++){
			secondAscensorReport[i]='0';
		}
		secondAscensorReport[currentFloor_second+5]='1';
		break;
	}
	case SUBIENDO:{
		if(Timers[T2] == TIMER_FINISHED){
			currentFloor_second++;
			secondAscensorReport[3]='1';
			secondAscensorReport[5]='0';
			for(int i = 6; i < SIZE_PAGE_ASCENSOR; i++){
				secondAscensorReport[i]='0';
			}
			secondAscensorReport[currentFloor_second+5]='1';
			//CDC_Transmit_FS(secondAscensorReport, SIZE_PAGE_ASCENSOR); --->
			if(currentFloor_second == tempFloor_second){
				for(int i = 0; i < SIZE_QUEUE; i++){
					if(queueFloor_second[i] == currentFloor_second){
						queueFloor_second[i] = DEFAULT_BASE_FLOOR;
					}
				}
				secondAscensor = ABRIR_PUERTAS;
				//CDC_Transmit_FS(constantBufferUp, 7);
				return;
			}
			else{
				for(int i = 0; i < SIZE_QUEUE; i++){
					if(queueFloor_second[i] == currentFloor_second){
						secondAscensor = ABRIR_PUERTAS;
						queueFloor_second[i] = DEFAULT_BASE_FLOOR;
						//CDC_Transmit_FS(constantBufferUp, 7);
					}
				}
			}
			//CDC_Transmit_FS(buf, 5);
			Timers[T2] = CONST_TIME_MOVEMENT;
		}
		break;
	}
	case BAJANDO:{
		if(Timers[T3] == TIMER_FINISHED){
			currentFloor_second--;
			secondAscensorReport[3]='0';
			secondAscensorReport[5]='1';
			for(int i = 6; i < SIZE_PAGE_ASCENSOR; i++){
				secondAscensorReport[i]='0';
			}
			secondAscensorReport[currentFloor_second+5]='1';
			//CDC_Transmit_FS(secondAscensorReport, SIZE_PAGE_ASCENSOR); --->
			if(currentFloor_second == tempFloor_second){
				for(int i = 0; i < SIZE_QUEUE; i++){
					if(queueFloor_second[i] == currentFloor_second){
						queueFloor_second[i] = DEFAULT_BASE_FLOOR;
					}
				}
				secondAscensor = ABRIR_PUERTAS;
				//CDC_Transmit_FS(constantBufferDown, 7);
				return;
			}
			else{
				for(int i = 0; i < SIZE_QUEUE; i++){
					if(queueFloor_second[i] == currentFloor_second){
						secondAscensor = ABRIR_PUERTAS;
						queueFloor_second[i] = DEFAULT_BASE_FLOOR;
						//CDC_Transmit_FS(constantBufferDown, 7);
					}
				}
			}
			//CDC_Transmit_FS(buf, 5);
			Timers[T3] = CONST_TIME_MOVEMENT;
		}
		break;
	}
	case ABRIR_PUERTAS:{
		globalDoors[1] = '1';
		if(Timers[TDOORS_TWO] == TIMER_FINISHED){
			secondAscensor = CERRAR_PUERTAS;
			Timers[TDOORS_TWO] = CONST_TIME_MOVEMENT;
		}
		break;
	}
	case CERRAR_PUERTAS:{
		if(Timers[TDOORS_TWO] == TIMER_FINISHED){
			secondAscensor = DETENIDO;
			globalDoors[1] = '0';
		}
		break;
	}
	}
}

uint8_t crc8(uint8_t *message, uint32_t length)
{
	uint8_t i, j, crc = 0;
    for (i = 0; i < length; i++)
    {
    	crc ^= message[i];
    	for (j = 0; j < 8; j++)
    	{
    		if (crc & 1)
    			crc ^= G_P;
    		crc >>= 1;
    	}
    }
    return crc;
}

void PrinterUnit(){
	switch(printerUnit){
	case PRINTING:{
		for(int i = 0; i < SIZE_PAGE_ASCENSOR; i++)
			dataReport[i] = firstAscensorReport[i];
		for(int i = SIZE_PAGE_ASCENSOR; i < SIZE_PAGE_ASCENSOR*2; i++)
			dataReport[i] = secondAscensorReport[i-SIZE_PAGE_ASCENSOR];
		for(int i = SIZE_PAGE_ASCENSOR*2; i < (SIZE_DATA_REPORT); i++)
			dataReport[i] = globalDoors[i-SIZE_PAGE_ASCENSOR*2];
		CDC_Transmit_FS(dataReport, SIZE_DATA_REPORT);
		Timers[TPRINT] = 500;
		printerUnit = WAIT_INFO;
		break;
	}
	case WAIT_INFO:{
		if(Timers[TPRINT] == TIMER_FINISHED){
			printerUnit = PRINTING;
		}
		break;
	}
	case ERROR_UNIT:{
		if(Timers[TPRINT] == TIMER_FINISHED){
			CDC_Transmit_FS(dataReport, SIZE_DATA_REPORT);
			printerUnit = WAIT_INFO;
		}
	}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of ERROR_UNIT occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
