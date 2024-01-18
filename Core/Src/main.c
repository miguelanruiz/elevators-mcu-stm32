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
  * @author         : Miguel Angel Ruiz Torres
  * @email          : miguelangelrtorresco@gmail.com
  * @github         : https://github.com/miguelanruiz
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
/**
  * @brief Constant indicating the timer has finished its count.
  */
#define TIMER_FINISHED 0

/**
  * @brief Total number of elevators being managed.
  */
#define NUM_ELEVATORS 2

/**
  * @brief Size of the queue that holds floor requests for each elevator.
  */
#define SIZE_QUEUE 10

/**
  * @brief Size of the keyboard queue, matching the size of the main queue.
  */
#define SIZE_QUEUE_KEYBOARD SIZE_QUEUE

/**
  * @brief Size of the command chain array for each elevator.
  */
#define SIZE_PAGE_ASCENSOR 16

/**
  * @brief Size of the array that represents the status of the doors.
  */
#define SIZE_PAGE_DOORS NUM_ELEVATORS

/**
  * @brief Total size of the data report, combining command chains and door status.
  */
#define SIZE_DATA_REPORT (SIZE_PAGE_ASCENSOR * NUM_ELEVATORS + SIZE_PAGE_DOORS)

/**
  * @brief Placeholder value when no floor is selected.
  */
#define NO_FLOOR_SELECTED 3

/**
  * @brief The starting floor for the MCU upon initialization.
  */
#define DEFAULT_START_MCU_FLOOR 3

/**
  * @brief Constant time for elevator movement between floors.
  */
#define CONST_TIME_MOVEMENT 3000

/**
  * @brief Constant time for doors' opening and closing.
  */
#define CONST_DOORS_MOVEMENT 8000

/**
  * @brief Interval at which USB data transmission occurs.
  */
#define USB_TRANSMISSION_INTERVAL 500

/**
  * @brief Elevator ID for the first elevator.
  */
#define ELEVATOR_ID_A '1'

/**
  * @brief Elevator ID for the second elevator.
  */
#define ELEVATOR_ID_B '2'

/**
  * @brief Request identifier for elevator commands.
  */
#define ELEVATOR_REQUEST 'X'

/**
  * @brief Index where floor buttons begin in the elevator command chain.
  */
#define FLOOR_BUTTONS_START_INDEX 6

/**
  * @brief Index for the 'going up' bit in the command chain.
  */
#define BIT_UP 3

/**
  * @brief Index for the 'going down' bit in the command chain.
  */
#define BIT_DOWN 5

/**
  * @brief Polynomial used for CRC8 checksum calculations.
  */
#define G_P 0x107 // x^8 + x^2 + x + 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* Enums for various states and statuses used in the application */

/**
 * @brief Timer identifiers for various timing functions.
 */
enum {
    T0 = 0,
    T1,
    T2,
    T3,
    T4,
    T5,
    TRANSMISSION_TIMER,
    TIMERS
};

/**
 * @brief States in which an elevator can be.
 */
typedef enum {
    STOPPED = 0,
    GOING_UP,
    GOING_DOWN,
    OPENING_DOORS,
    CLOSING_DOORS
} ELEVATOR_STATE;

/**
 * @brief States of USB communication.
 */
typedef enum {
    SENDING = 0,
    AWAITING,
    COM_ERROR
} USB_COMM_STATE;

/**
 * @brief Flag statuses for enabling or disabling features.
 */
typedef enum {
    _BIT_DISABLE = '0',
    _BIT_ENABLE = '1'
} FLAG_STATUS;

/* Variables for tracking the state and operation of elevators and communication */

/**
 * @brief Current USB communication state.
 */
volatile USB_COMM_STATE commState = SENDING;

/**
 * @brief Elevator structure representing the state and control of an elevator.
 */
typedef struct {
    volatile ELEVATOR_STATE state;            ///< Current state of the elevator
    volatile uint8_t position;                ///< Current floor position of the elevator
    volatile uint8_t next;                    ///< Next target floor for the elevator
    volatile uint8_t queue[SIZE_QUEUE];       ///< Queue of floor requests
    volatile uint8_t commandChain[SIZE_PAGE_ASCENSOR]; ///< Command chain for USB communication
    uint8_t timerUp;                          ///< Timer index for moving up
    uint8_t timerDown;                        ///< Timer index for moving down
    uint8_t timerDoors;                       ///< Timer index for doors operation
} Elevator;

/**
 * @brief Array of elevator structures to manage multiple elevators.
 */
Elevator elevators[NUM_ELEVATORS];

/**
 * @brief Array of timers for various timing operations in the application.
 */
volatile uint32_t TIM_BANK[TIMERS];

/* Variables for door status and USB communication */

/**
 * @brief Status of the doors for each elevator, where each index represents an elevator.
 */
volatile uint8_t lockedDoors[SIZE_PAGE_DOORS];

/**
 * @brief Buffer to hold the combined data report for USB transmission.
 */
volatile uint8_t dataReport[SIZE_DATA_REPORT];

/**
 * @brief Counter used for timing operations.
 */
volatile uint16_t TIME_STEP = 0;

/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
/* Function prototypes for initializing and running elevator operations */
void InitCommandChain(volatile uint8_t *commandChain, char elevatorID);
void InitElevator(Elevator* e, char elevatorID, uint8_t timerUpIdx, uint8_t timerDownIdx, uint8_t timerDoorsIdx);
void RunElevator(Elevator* e);
void UpdateTargetFloor(Elevator* e);
void UpdateElevatorCommandChain(Elevator* e, ELEVATOR_STATE movingState);
void CheckAndOpenDoors(Elevator* e);
void BuildDataReport(uint8_t *firstReport, uint8_t *secondReport, uint8_t *doorsStatus, uint8_t *combinedReport);
int GetClosestElevator(int floorRequested);
void AttemptUSBReset();
void HandleUSBCommunication();
void HandleElevatorRequest(uint8_t *buf);
void HandleFloorButtonState(uint8_t *buf, int elevatorIndex);
void SetErrorState(int elevatorIndex);
uint8_t CRC_8(uint8_t *message, uint32_t length);
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

  InitElevator(&elevators[0], ELEVATOR_ID_A, T0, T1, T2);
  InitElevator(&elevators[1], ELEVATOR_ID_B, T3, T4, T5);
  
  lockedDoors[0] = _BIT_DISABLE;
  lockedDoors[1] = _BIT_DISABLE;

  TIM_BANK[TRANSMISSION_TIMER] = USB_TRANSMISSION_INTERVAL;

  // Doors timer
  TIM_BANK[T2] = 0;
  TIM_BANK[T5] = 0;

  while (1)
  {
    /* USER CODE BEGIN 3 */
	  runElevator(&elevators[0]);
	  runElevator(&elevators[1]);
	  HandleUSBCommunication();
  }
  return 0;
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
  __HAL_RCC_GPIOC_CLK__BIT_ENABLE();
  __HAL_RCC_GPIOD_CLK__BIT_ENABLE();
  __HAL_RCC_GPIOA_CLK__BIT_ENABLE();

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

/**
  * @brief Modifies the command chain for a specified elevator.
  * @param commandChain: A pointer to the array where the elevator command chain is stored.
  *                      The default command chain 'A1S0B0' indicates:
  *                      'A': Prefix for elevator command chain,
  *                      '1' or '2': Elevator ID,
  *                      'S0': Not going up,
  *                      'B0': Not going down.
  * @param elevatorID: The elevator identifier, '1' or '2', to be set in the command chain.
  * @retval None
  */
void InitCommandChain(volatile uint8_t *commandChain, char elevatorID) {
    if (commandChain != NULL) {
        commandChain[0] = 'A';
        commandChain[1] = elevatorID;
        commandChain[2] = 'S';
        commandChain[3] = _BIT_DISABLE;
        commandChain[4] = 'B';
        commandChain[5] = _BIT_DISABLE;
    }
}

/**
 * @brief Initialize an elevator structure with default values and specified timers.
 * @param e Pointer to the elevator structure to initialize.
 * @param elevatorID Identifier for the elevator ('1' or '2').
 * @param timerUpIdx Index of the timer for going up.
 * @param timerDownIdx Index of the timer for going down.
 * @param timerDoorsIdx Index of the timer for doors operation.
 */
void InitElevator(Elevator* e, char elevatorID, uint8_t timerUpIdx, uint8_t timerDownIdx, uint8_t timerDoorsIdx) {
    if (e != NULL) {
        e->state = STOPPED;
        e->position = DEFAULT_START_MCU_FLOOR;
        for (int i = 0; i < SIZE_QUEUE; i++) {
            e->queue[i] = NO_FLOOR_SELECTED;
        }
        InitElevatorCommandChain(e->commandChain, elevatorID);
        e->timerUp = timerUpIdx;
        e->timerDown = timerDownIdx;
        e->timerDoors = timerDoorsIdx;
    }
}

/**
 * @brief Callback function for TIM2 periodic elapsed events.
 * @param htim Pointer to the TIM_HandleTypeDef structure.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  //UNUSED(htim);
	if(htim->Instance == TIM2){
		if(TIME_STEP ++ == SECOND){
			TIME_STEP = 0;
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		}
		for(int i = 0; i < TIMERS; i++){
			if(TIM_BANK[i] != TIMER_FINISHED)
				TIM_BANK[i]--;
		}
	}
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
}

/**
 * @brief Handles elevator request messages.
 * @param buf Buffer containing the elevator request message.
 */
void HandleElevatorRequest(uint8_t *buf) {
    if (buf[BIT_UP] == _BIT_ENABLE || buf[BIT_DOWN] == _BIT_ENABLE) {
        int floorRequested = buf[7] - 47;
        int closestElevatorIndex = GetClosestElevator(floorRequested);

        // Assign the floor to the closest elevator
        elevators[closestElevatorIndex].queue[floorRequested - 1] = floorRequested;
        elevators[closestElevatorIndex].state = OPENING_DOORS;
    }
}

/**
 * @brief Returns the index of the closest elevator to the requested floor.
 * @param floorRequested The floor number that has been requested.
 * @return Index of the closest elevator.
 */
int GetClosestElevator(int floorRequested) {
    int distanceToFirstElevator = abs(elevators[0].position - floorRequested);
    int distanceToSecondElevator = abs(elevators[1].position - floorRequested);

    if (distanceToFirstElevator < distanceToSecondElevator) {
        return 0;
    } else {
        return 1;
    }
}

/**
 * @brief Handles the state of floor buttons for a specific elevator.
 * @param buf Buffer containing the state of the floor buttons.
 * @param elevatorIndex Index of the elevator.
 */
void HandleFloorButtonState(uint8_t *buf, int elevatorIndex) {
    if (buf[14] != CRC_8(buf + 4, 10)) {
        SetErrorState(elevatorIndex);
        return;
    }

    for (int i = 0; i < SIZE_QUEUE_KEYBOARD; i++) {
        if (buf[i + 4] == '1' && elevators[elevatorIndex].queue[i] == NO_FLOOR_SELECTED) {
            elevators[elevatorIndex].queue[i] = i + 1;
        }
    }
}

/**
 * @brief Sets the error state for a specific elevator.
 * @param elevatorIndex Index of the elevator to set the error state.
 */
void SetErrorState(int elevatorIndex) {
    commState = COM_ERROR;
    for (int i = 6; i < SIZE_PAGE_ASCENSOR; i++) {
        elevators[elevatorIndex].commandChain[i] = _BIT_ENABLE;
    }
}

/**
 * @brief Callback function to handle data received over USB.
 * @param buf Buffer containing received data.
 * @param len Length of the received data.
 */
void CDC_ReceiveCallBack(uint8_t *buf, uint32_t len) {
    if (buf[0] == 'A' && buf[1] == ELEVATOR_REQUEST) {
        if (buf[8] != CRC_8(buf, 8)) return;
        HandleElevatorRequest(buf);
    } else if (buf[0] == 'T' && buf[1] == 'F' && buf[2] == 'S') {
        if (buf[3] == ELEVATOR_ID_A) {
            HandleFloorButtonState(buf, 0);
        } else if (buf[3] == ELEVATOR_ID_B) {
            HandleFloorButtonState(buf, 1);
        }
    }
}

/**
 * @brief Runs the elevator operation logic based on its current state.
 * @param e Pointer to the elevator structure.
 */
void RunElevator(Elevator* e) {
    if (e == NULL) {
        return;
    }

    switch (e->state) {
        case STOPPED: {
            // Logic to determine the next target floor and transition to moving state
            UpdateTargetFloor(e);
            break;
        }
        case GOING_UP: {
            if (TIM_BANK[e->timerUp] == TIMER_FINISHED) {
                e->position++;
                UpdateElevatorCommandChain(e, GOING_UP);
                CheckAndOpenDoors(e);
                TIM_BANK[e->timerUp] = CONST_TIME_MOVEMENT;
            }
            break;
        }
        case GOING_DOWN: {
            if (TIM_BANK[e->timerDown] == TIMER_FINISHED) {
                e->position--;
                UpdateElevatorCommandChain(e, GOING_DOWN);
                CheckAndOpenDoors(e);
                TIM_BANK[e->timerDown] = CONST_TIME_MOVEMENT;
            }
            break;
        }
        case OPENING_DOORS: {
            if (TIM_BANK[e->timerDoors] == TIMER_FINISHED) {
                e->state = CLOSING_DOORS;
                lockedDoors[e->position] = _BIT_ENABLE;
                TIM_BANK[e->timerDoors] = CONST_DOORS_MOVEMENT;
            }
            break;
        }
        case CLOSING_DOORS: {
            if (TIM_BANK[e->timerDoors] == TIMER_FINISHED) {
                e->state = STOPPED;
                lockedDoors[e->position] = _BIT_DISABLE;
            }
            break;
        }
    }
}

/**
 * @brief Updates the target floor for the elevator based on its queue.
 * @param e Pointer to the elevator structure.
 */
void UpdateTargetFloor(Elevator* e) {
    int highestPriorityFloor = -1;
    for (int i = 0; i < SIZE_QUEUE; ++i) {
        if (e->queue[i] != NO_FLOOR_SELECTED &&
            (highestPriorityFloor == -1 || e->queue[i] > highestPriorityFloor)) {
            highestPriorityFloor = e->queue[i];
        }
    }

    if (highestPriorityFloor != -1) {
        if (highestPriorityFloor > e->position) {
            e->state = GOING_UP;
            TIM_BANK[e->timerUp] = CONST_TIME_MOVEMENT;
        } else if (highestPriorityFloor < e->position) {
            e->state = GOING_DOWN;
            TIM_BANK[e->timerDown] = CONST_TIME_MOVEMENT;
        }
    }
}

/**
 * @brief Updates the elevator's command chain based on its movement state.
 * @param e Pointer to the elevator structure.
 * @param movingState The current moving state of the elevator (GOING_UP, GOING_DOWN).
 */
void UpdateElevatorCommandChain(Elevator* e, ELEVATOR_STATE movingState) {
    memset(e->commandChain, _BIT_DISABLE, SIZE_PAGE_ASCENSOR);
    e->commandChain[0] = 'A';
    e->commandChain[1] = movingState == GOING_UP ? ELEVATOR_ID_A : ELEVATOR_ID_B;
    e->commandChain[2] = 'S';
    e->commandChain[BIT_UP] = movingState == GOING_UP ? _BIT_ENABLE : _BIT_DISABLE;
    e->commandChain[4] = 'B';
    e->commandChain[BIT_DOWN] = movingState == GOING_DOWN ? _BIT_ENABLE : _BIT_DISABLE;
    e->commandChain[FLOOR_BUTTONS_START_INDEX + e->position - 1] = _BIT_ENABLE;
}

/**
 * @brief Checks and opens the doors if the elevator has reached a requested floor.
 * @param e Pointer to the elevator structure.
 */
void CheckAndOpenDoors(Elevator* e) {
    for (int i = 0; i < SIZE_QUEUE; ++i) {
        if (e->queue[i] == e->position) {
            e->state = OPENING_DOORS;
            TIM_BANK[e->timerDoors] = CONST_DOORS_MOVEMENT;
            e->queue[i] = NO_FLOOR_SELECTED;  // Clear the request as it's being served
            break;
        }
    }
}

/**
 * @brief Computes the CRC-8 checksum for a given message.
 * @param message Pointer to the message data.
 * @param length Length of the message.
 * @return Computed CRC-8 checksum.
 */
uint8_t CRC_8(uint8_t *message, uint32_t length)
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

/**
 * @brief Handles the overall USB communication logic. STM to PC communication.
 */
void HandleUSBCommunication() {
    switch (commState) {
        case SENDING: {
            // Build the data report to send
            BuildDataReport(elevators[0].commandChain, elevators[1].commandChain, lockedDoors, dataReport);
            // Send the data report over USB
            CDC_Transmit_FS(dataReport, SIZE_DATA_REPORT);
            // Set timer for next transmission
            TIM_BANK[TRANSMISSION_TIMER] = USB_TRANSMISSION_INTERVAL;
            // Transition to waiting state
            commState = AWAITING;
            break;
        }
        case AWAITING: {
            // Check if it's time to send another report
            if (TIM_BANK[TRANSMISSION_TIMER] == TIMER_FINISHED) {
                commState = SENDING;
            }
            break;
        }
        case COM_ERROR: {
            // Handle error state if needed, maybe try to reset USB interface
            if (TIM_BANK[TRANSMISSION_TIMER] == TIMER_FINISHED) {
                AttemptUSBReset();
                commState = AWAITING;
            }
            break;
        }
    }
}

/**
 * @brief Builds the data report for USB transmission.
 * @param firstReport Pointer to the first elevator's report data.
 * @param secondReport Pointer to the second elevator's report data.
 * @param doorsStatus Pointer to the doors' status data.
 * @param combinedReport Pointer to the buffer where the combined report will be stored.
 */
void BuildDataReport(uint8_t *firstReport, uint8_t *secondReport, uint8_t *doorsStatus, uint8_t *combinedReport) {
    memcpy(combinedReport, firstReport, SIZE_PAGE_ASCENSOR);
    memcpy(combinedReport + SIZE_PAGE_ASCENSOR, secondReport, SIZE_PAGE_ASCENSOR);
    memcpy(combinedReport + (2 * SIZE_PAGE_ASCENSOR), doorsStatus, SIZE_PAGE_DOORS);
}

/**
 * @brief Attempts to reset the USB interface in case of errors.
 */
void AttemptUSBReset() {
    // Logic to reset USB if possible
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of COM_ERROR_UNIT occurrence.
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
