/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define duoji_ok 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

MOTOR_send MOTOR_M8010_SEND;
MOTOR_recv MOTOR_M8010_RECV;
uint8_t action_ok = 0;
uint8_t key_pressed = 0;

uint16_t ADV[128] = {0}; // ����CCD���??
uint8_t CCD_Zhongzhi = 0, CCD_Yuzhi = 0;
// uint8_t SBUS_Data_Low[17] = {0};
// uint8_t SBUS_Data_High[17] = {0};
// uint8_t SBUS_Transmit_Flag = 0;
// uint8_t SBUS_H_L = 0;

uint8_t atest = 10;
uint16_t fff = 0;
uint8_t bin = 0;

uint8_t fangmiao = 0;
uint8_t qumiao = 0;
uint8_t chongzhi = 1;
uint8_t flag_ad7606 = 0;
int16_t ad7606_value[4] = {0};
uint8_t SBUS_done = 0;
uint8_t jishu_zhuanxiang = 0;
uint8_t staus_dipan = 0;
uint8_t fa_qiu_zhuan_xiang_start = 0;
uint8_t faqiuduoji = 0;
uint8_t fa_qiu_done = 0;
uint8_t quqiu_duoji = 0;
uint8_t ball_pos_;
uint8_t yuntai_finish = 0;
double init_pos;
double need_pos;
double set_T;
double a = 0;
double t_now = 3;
uint8_t pos_change_flag = 0;
float rotate_angle[12] = {53.0, 50.0, 52.0, 45.0, 25.0, 15.0, /**/ -22.6, -20.67, -12.57, 2.036, 10.246, 26.81}; //;6:3.750399827957153322,6.2553963661193847����2500������10000//5��6.0343108177185058����һ��//4��5.140189647674560544����һ��//3��3.5250959396362304����һ��//2��2.6181278228759765����һ��//1��7.8846794366836547E-1����һ��
int16_t back_rotate_speed[12] = {11000, 14000, 13000, 13000, 13000, 13000, /*�Գ�*/ 11000, 11000, 11000 + 1000, 11000, 11000, 11000};
int16_t shoot_speed_right[12] = {3000 + 1000, 3200 + 600, 2800 + 600, 2700 + 300, 2700 + 300, 2700 + 300, /**/ 2500 + 600, 2500 + 500, 3200, 2500, 2500, 2500};
int16_t shoot_speed_left[12] = {1800 + 1000, 1500 + 600, 1600 + 600, 1500 + 300, 1500 + 300, 1500 + 300, /**/ 2500 + 600, 2500 + 500, 3200, 2500, 2500, 2500};
// int16_t shoot_speed_right[6] = {2000,1700,1800,1700,1500,1100};
float yuntai_init_pos = 0;
uint8_t jishu = 0;
int16_t quqiu_duty = 3000;

uint8_t action_us[4] = {0};
// IMU�������

raw_t raw = {0};
uint8_t decode_succ = 0;
uint8_t rx_buf[10];

imu imu_data;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for M3508control */
osThreadId_t M3508controlHandle;
const osThreadAttr_t M3508control_attributes = {
    .name = "M3508control",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for myTest */
osThreadId_t myTestHandle;
const osThreadAttr_t myTest_attributes = {
    .name = "myTest",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for M8010control */
osThreadId_t M8010controlHandle;
const osThreadAttr_t M8010control_attributes = {
    .name = "M8010control",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for DS5160_duoji */
osThreadId_t DS5160_duojiHandle;
const osThreadAttr_t DS5160_duoji_attributes = {
    .name = "DS5160_duoji",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for shoot */
osThreadId_t shootHandle;
const osThreadAttr_t shoot_attributes = {
    .name = "shoot",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for transmit_zero */
osThreadId_t transmit_zeroHandle;
const osThreadAttr_t transmit_zero_attributes = {
    .name = "transmit_zero",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for chuansongdai */
osThreadId_t chuansongdaiHandle;
const osThreadAttr_t chuansongdai_attributes = {
    .name = "chuansongdai",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for CCD */
osThreadId_t CCDHandle;
const osThreadAttr_t CCD_attributes = {
    .name = "CCD",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for GM6020 */
osThreadId_t GM6020Handle;
const osThreadAttr_t GM6020_attributes = {
    .name = "GM6020",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for jiguang */
osThreadId_t jiguangHandle;
const osThreadAttr_t jiguang_attributes = {
    .name = "jiguang",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for IMU */
osThreadId_t IMUHandle;
const osThreadAttr_t IMU_attributes = {
    .name = "IMU",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* USER CODE BEGIN PV */
uint16_t error_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_UART7_Init(void);
static void MX_UART8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_UART4_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM8_Init(void);
void StartDefaultTask(void *argument);
void M3508controlTask(void *argument);
void myTestTask(void *argument);
void M8010controlTask(void *argument);
void DS5160(void *argument);
void shoot_task(void *argument);
void transmit_zero_(void *argument);
void chuansongdai_Task(void *argument);
void CCDTask(void *argument);
void GM6020Task(void *argument);
void jiguang_task(void *argument);
void IMU_TASK(void *argument);

/* USER CODE BEGIN PFP */

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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM5_Init();
  MX_UART4_Init();
  MX_ADC1_Init();
  MX_SPI4_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle =
      osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of M3508control */
  M3508controlHandle =
      osThreadNew(M3508controlTask, NULL, &M3508control_attributes);

  /* creation of myTest */
  myTestHandle = osThreadNew(myTestTask, NULL, &myTest_attributes);

  /* creation of M8010control */
  M8010controlHandle =
      osThreadNew(M8010controlTask, NULL, &M8010control_attributes);

  /* creation of DS5160_duoji */
  DS5160_duojiHandle = osThreadNew(DS5160, NULL, &DS5160_duoji_attributes);

  /* creation of shoot */
  shootHandle = osThreadNew(shoot_task, NULL, &shoot_attributes);

  /* creation of transmit_zero */
  transmit_zeroHandle =
      osThreadNew(transmit_zero_, NULL, &transmit_zero_attributes);

  /* creation of chuansongdai */
  chuansongdaiHandle =
      osThreadNew(chuansongdai_Task, NULL, &chuansongdai_attributes);

  /* creation of CCD */
  CCDHandle = osThreadNew(CCDTask, NULL, &CCD_attributes);

  /* creation of GM6020 */
  GM6020Handle = osThreadNew(GM6020Task, NULL, &GM6020_attributes);

  /* creation of jiguang */
  jiguangHandle = osThreadNew(jiguang_task, NULL, &jiguang_attributes);

  /* creation of IMU */
  IMUHandle = osThreadNew(IMU_TASK, NULL, &IMU_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data
   * Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef can_filter_st;
  can_filter_st.FilterActivation = ENABLE;
  can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter_st.FilterIdHigh = 0x0000;
  can_filter_st.FilterIdLow = 0x0000;
  can_filter_st.FilterMaskIdHigh = 0x0000;
  can_filter_st.FilterMaskIdLow = 0x0000;
  can_filter_st.FilterBank = 0;
  can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
  HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END CAN1_Init 2 */
}

/**
 * @brief CAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 4;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  CAN_FilterTypeDef can_filter_st;
  can_filter_st.FilterActivation = ENABLE;
  can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter_st.FilterIdHigh = 0x0000;
  can_filter_st.FilterIdLow = 0x0000;
  can_filter_st.FilterMaskIdHigh = 0x0000;
  can_filter_st.FilterMaskIdLow = 0x0000;
  can_filter_st.FilterBank = 14;
  can_filter_st.SlaveStartFilterBank = 14;
  can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
  HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
  /* USER CODE END CAN2_Init 2 */
}

/**
 * @brief SPI4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */
}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 31;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 39999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1750;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 3600;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 1000;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);
}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 1280 - 1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 2000 - 1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 113;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 145;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);
}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */
}

/**
 * @brief UART7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */
  HAL_UART_Receive_IT(&huart7, &bin, 1);

  /* USER CODE END UART7_Init 2 */
}

/**
 * @brief UART8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */
  HAL_UART_Receive_IT(&huart8, &rx_buf[0], 1);
  /* USER CODE END UART8_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 100000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_2;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  HAL_UART_Receive_IT(&huart1, &SBUS_Receive_ch, 1);
  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  HAL_UART_Receive_IT(&huart3, &SBUS_ours_Receive_ch, 1);
  /* USER CODE END USART3_Init 2 */
}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 4000000;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(chongzhi_gei_xia_cengQ2_GPIO_Port,
                    chongzhi_gei_xia_cengQ2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF,
                    YUN_TAI_TONG_XIN_1_I1_Pin | YUN_TAI_TONG_XIN_2_I2_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC,
                    qumiao_jiazi_gao_N2_Pin | qumiao_shengjiang_di_L1_Pin |
                        qumiao_shengjiang_gao_M1_Pin | quqiu_jiazi_N1_Pin |
                        AD7606_CVA_O1_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AD7606_CS_O2_GPIO_Port, AD7606_CS_O2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AD7606_RST_P2_GPIO_Port, AD7606_RST_P2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(qumiao_jiazi_di_M2_GPIO_Port, qumiao_jiazi_di_M2_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin : guang_dian_kai_guan_Z_Pin */
  GPIO_InitStruct.Pin = guang_dian_kai_guan_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(guang_dian_kai_guan_Z_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : chongzhi_gei_xia_cengQ2_Pin */
  GPIO_InitStruct.Pin = chongzhi_gei_xia_cengQ2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(chongzhi_gei_xia_cengQ2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : YUN_TAI_TONG_XIN_1_I1_Pin YUN_TAI_TONG_XIN_2_I2_Pin */
  GPIO_InitStruct.Pin = YUN_TAI_TONG_XIN_1_I1_Pin | YUN_TAI_TONG_XIN_2_I2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PH2 PH3 PH4 PH5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PD15 PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_15 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : qumiao_jiazi_gao_N2_Pin AD7606_CS_O2_Pin
     qumiao_shengjiang_di_L1_Pin qumiao_shengjiang_gao_M1_Pin quqiu_jiazi_N1_Pin
     AD7606_CVA_O1_Pin */
  GPIO_InitStruct.Pin =
      qumiao_jiazi_gao_N2_Pin | AD7606_CS_O2_Pin | qumiao_shengjiang_di_L1_Pin |
      qumiao_shengjiang_gao_M1_Pin | quqiu_jiazi_N1_Pin | AD7606_CVA_O1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PH6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : AD7606_RST_P2_Pin */
  GPIO_InitStruct.Pin = AD7606_RST_P2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AD7606_RST_P2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AD7606_BUSY_P1_Pin */
  GPIO_InitStruct.Pin = AD7606_BUSY_P1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(AD7606_BUSY_P1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : qumiao_jiazi_di_M2_Pin */
  GPIO_InitStruct.Pin = qumiao_jiazi_di_M2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(qumiao_jiazi_di_M2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  /* Infinite loop */
  for (;;)
  {
    if (SBUS_done == 1)
    {
      SBUS.CH1 =
          ((int16_t)SBUS_RX_BUF[0] >> 0 | ((int16_t)SBUS_RX_BUF[1] << 8)) &
          0x07FF;
      SBUS.CH2 =
          ((int16_t)SBUS_RX_BUF[1] >> 3 | ((int16_t)SBUS_RX_BUF[2] << 5)) &
          0x07FF;
      SBUS.CH3 =
          ((int16_t)SBUS_RX_BUF[2] >> 6 | ((int16_t)SBUS_RX_BUF[3] << 2) |
           (int16_t)SBUS_RX_BUF[4] << 10) &
          0x07FF;
      SBUS.CH4 =
          ((int16_t)SBUS_RX_BUF[4] >> 1 | ((int16_t)SBUS_RX_BUF[5] << 7)) &
          0x07FF;
      SBUS.CH5 =
          ((int16_t)SBUS_RX_BUF[5] >> 4 | ((int16_t)SBUS_RX_BUF[6] << 4)) &
          0x07FF;
      SBUS.CH6 =
          ((int16_t)SBUS_RX_BUF[6] >> 7 | ((int16_t)SBUS_RX_BUF[7] << 1) |
           (int16_t)SBUS_RX_BUF[8] << 9) &
          0x07FF;
      SBUS.CH7 =
          ((int16_t)SBUS_RX_BUF[8] >> 2 | ((int16_t)SBUS_RX_BUF[9] << 6)) &
          0x07FF;
      SBUS.CH8 =
          ((int16_t)SBUS_RX_BUF[9] >> 5 | ((int16_t)SBUS_RX_BUF[10] << 3)) &
          0x07FF;
      SBUS.CH9 =
          ((int16_t)SBUS_RX_BUF[11] >> 0 | ((int16_t)SBUS_RX_BUF[12] << 8)) &
          0x07FF;
      SBUS.CH10 =
          ((int16_t)SBUS_RX_BUF[12] >> 3 | ((int16_t)SBUS_RX_BUF[13] << 5)) &
          0x07FF;
      SBUS.CH11 =
          ((int16_t)SBUS_RX_BUF[13] >> 6 | ((int16_t)SBUS_RX_BUF[14] << 2) |
           (int16_t)SBUS_RX_BUF[15] << 10) &
          0x07FF;
      SBUS.CH12 =
          ((int16_t)SBUS_RX_BUF[15] >> 1 | ((int16_t)SBUS_RX_BUF[16] << 7)) &
          0x07FF;
      SBUS.CH13 =
          ((int16_t)SBUS_RX_BUF[16] >> 4 | ((int16_t)SBUS_RX_BUF[17] << 4)) &
          0x07FF;
      SBUS.CH14 =
          ((int16_t)SBUS_RX_BUF[17] >> 7 | ((int16_t)SBUS_RX_BUF[18] << 1) |
           (int16_t)SBUS_RX_BUF[19] << 9) &
          0x07FF;
      SBUS.CH15 =
          ((int16_t)SBUS_RX_BUF[19] >> 2 | ((int16_t)SBUS_RX_BUF[20] << 6)) &
          0x07FF;
      SBUS.CH16 =
          ((int16_t)SBUS_RX_BUF[20] >> 5 | ((int16_t)SBUS_RX_BUF[21] << 3)) &
          0x07FF;
      SBUS_done = 0;
    }
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_M3508controlTask */
/**
 * @brief Function implementing the M3508control thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_M3508controlTask */
void M3508controlTask(void *argument)
{
  /* USER CODE BEGIN M3508controlTask */
  /* Infinite loop */
  for (;;)
  {
    M3508_Motor_Position_Ctrl();
    M3508_Motor_Speed_Ctrl();
    M3508_Motor_Current_Ctrl();
    M3508_Motor_Current_Ctrl_h();
    osDelay(1);
  }
  /* USER CODE END M3508controlTask */
}

/* USER CODE BEGIN Header_myTestTask */
/**
 * @brief Function implementing the myTest thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_myTestTask */
void myTestTask(void *argument)
{
  /* USER CODE BEGIN myTestTask */
  /* Infinite loop */
  for (;;)
  {
    if (SBUS.CH1 != 0)
      SBUS_Send();
    osDelay(1);
  }

  /* USER CODE END myTestTask */
}

/* USER CODE BEGIN Header_M8010controlTask */
/**
 * @brief Function implementing the M8010control thread.
 * @param argument: Not used
 * @retval None
 */
// ������߹�?
double S5S_Plan(double S_Pos, double all_t, double t)
{
  double now_Pos = 10 / pow(all_t, 3) * pow(t, 3) -
                   15 / pow(all_t, 4) * pow(t, 4) +
                   6 / pow(all_t, 5) * pow(t, 5);

  return now_Pos * S_Pos;
}

/* USER CODE END Header_M8010controlTask */
void M8010controlTask(void *argument)
{
  /* USER CODE BEGIN M8010controlTask */
  MOTOR_M8010_SEND.id = 1;
  MOTOR_M8010_SEND.mode =
      0; // mode = 0
         // ���������ֻ��ȡһ�ε�ǰ�����λ�ýǶ�
  SERVO_Send_recv(&MOTOR_M8010_SEND,
                  &MOTOR_M8010_RECV); // ���ͺͻ�ȡ����
  init_pos = MOTOR_M8010_RECV.Pos;    // ��ȡ��ǰ�ĽǶ�
  set_T = 3;
  need_pos =
      0; //-(init_pos - 1.0369) * 180 /
         //(6.33*3.14);//�ϵ��ֱ��ת�������λ��

  while (1)
  {
    if (t_now < set_T)
    {
      MOTOR_M8010_SEND.id = 1;
      MOTOR_M8010_SEND.mode = 1;
      // MOTOR_M8010_SEND.K_P = 0.5; // λ�øնȣ���̫��Ҫ�޸ģ������˺ܶ�ε��ĺ����?
      MOTOR_M8010_SEND.K_P = 0.8;
      MOTOR_M8010_SEND.K_W = 0;
      MOTOR_M8010_SEND.Pos =
          init_pos + S5S_Plan(need_pos, set_T, t_now) * 3.14 / 180 * 6.33; // ���õ�ǰӦ���ĽǶ�
      MOTOR_M8010_SEND.W = 0;
      MOTOR_M8010_SEND.T = 0.0;
      SERVO_Send_recv(&MOTOR_M8010_SEND,
                      &MOTOR_M8010_RECV); // ���ͺͻ�ȡ����
      // t_now += 0.005;
      t_now += 0.01;
      osDelay(1);
    }
    else
    {
      MOTOR_M8010_SEND.id = 1;
      MOTOR_M8010_SEND.mode = 0; // modeΪ0ʱ�����Ϊֹͣģ�?
      SERVO_Send_recv(&MOTOR_M8010_SEND,
                      &MOTOR_M8010_RECV); // ���ͺͻ�ȡ����
      init_pos = MOTOR_M8010_RECV.Pos;    // ��ȡ��ǰ�ĽǶ�
      //      if(pos_change_flag == 1){
      //            osDelay(50);
      //            t_now = 0;
      //     pos_change_flag = 0;
      //      }
    }
    osDelay(1);
  }
  /* USER CODE END M8010controlTask */
}

/* USER CODE BEGIN Header_DS5160 */
/**
 * @brief Function implementing the DS5160_duoji thread.
 * @param argument: Not used
 * @retval None
 */
// �ϣ�353 �У�1024�£�1695
/* USER CODE END Header_DS5160 */
void DS5160(void *argument)
{
  /* USER CODE BEGIN DS5160 */

  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 3000);

  /* Infinite loop */
  for (;;)
  {
    if (quqiu_duoji)
    {
      //__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 2500);
      // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
      // osDelay(400);
      __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 1050);
      osDelay(500);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
      osDelay(500);
      __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, quqiu_duty);
      quqiu_duoji = 0;
      osDelay(1000);
      need_pos = -rotate_angle[ball_pos_];
      t_now = 0;
      while (t_now < set_T)
      {
        osDelay(1);
      }
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
      yuntai_finish = 1;
    }

    osDelay(1);
  }

  /* USER CODE END DS5160 */
}

/* USER CODE BEGIN Header_shoot_task */
/**
 * @brief Function implementing the shoot thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_shoot_task */
void shoot_task(void *argument)
{
  /* USER CODE BEGIN shoot_task */
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 1200);

  /* Infinite loop */
  for (;;)
  {
    //      if(SBUS.CH6 == 353 && SBUS_Analyse_Switch(SBUS.CH5) != 2){
    //         need_pos = 30;
    //         pos_change_flag = 1;
    //      if(need_pos == 15){
    //        while(fa_qiu_zhuan_xiang_start != 1){
    //        osDelay(5);}
    //        while(t_now < set_T){
    //          osDelay(5);
    //        }
    //        fa_qiu_zhuan_xiang_start = 0;
    //        M3508[1].ExpSpeed = 5000;
    //  M3508[2].ExpSpeed = 3000;
    //  M3508[3].ExpSpeed = -3000;
    //      faqiuduoji = 1;
    //      }
    //
    //      if(need_pos == 30){
    //        while(fa_qiu_zhuan_xiang_start != 1){
    //        osDelay(5);}
    //        while(t_now < set_T){
    //          osDelay(5);
    //        }
    //        fa_qiu_zhuan_xiang_start = 0;
    //        M3508[1].ExpSpeed = 5000;
    //  M3508[2].ExpSpeed = 3200;
    //  M3508[3].ExpSpeed = -3200;
    //      faqiuduoji = 1;}
    //
    //      if(need_pos == 45){
    //        while(fa_qiu_zhuan_xiang_start != 1){
    //        osDelay(5);}
    //        while(t_now < set_T){
    //          osDelay(5);
    //        }
    //        fa_qiu_zhuan_xiang_start = 0;
    //        M3508[1].ExpSpeed = 8000;
    //  M3508[2].ExpSpeed = 3400;
    //  M3508[3].ExpSpeed = -3400;
    //      faqiuduoji = 1;}
    //
    //  if(faqiuduoji == 1){
    //    faqiuduoji = 0;
    //    osDelay(500);
    //    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3,1866);
    //    while(SBUS.CH6 == 353){ osDelay(5);}
    //    M3508[1].ExpSpeed = 0;
    //    M3508[2].ExpSpeed = 0;
    //    M3508[3].ExpSpeed = 0;
    //     __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3,1200);
    //     pos_change_flag = 1; need_pos = -need_pos;
    //     t_now = 0;
    //     staus_dipan = 3;
    //  }

    osDelay(5);
  }
  /* USER CODE END shoot_task */
}

/* USER CODE BEGIN Header_transmit_zero_ */
/**
 * @brief Function implementing the transmit_zero thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_transmit_zero_ */
void transmit_zero_(void *argument)
{
  /* USER CODE BEGIN transmit_zero_ */
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  // jiguang_duoji
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  osDelay(5000);
  yuntai_init_pos = init_pos;
  /* Infinite loop */
  for (;;)
  {
    //      switch(SBUS_Analyse_Switch(SBUS.CH5)){
    //  case 2://��
    // need_pos = 46;
    //    pos_change_flag = 1;
    //    osDelay(500);
    //    while(SBUS_Analyse_Switch(SBUS.CH5) == 2);
    //
    //
    //   //  Robot.Robot_in_world.velocity_exp.Vx = 1000;
    ////    if(SBUS.CH6 != 353){
    ////     need_pos = -30;
    ////     pos_change_flag = 1;
    ////    }
    ////
    //    break;
    //    case 1://�м�
    //   // Robot.Robot_in_world.velocity_exp.Vy = 1000;
    //    break;
    //    case 0://����
    ////      Robot.Robot_in_world.velocity_exp.Vx = 0;
    ////      Robot.Robot_in_world.velocity_exp.Vy = 0;
    if (bin == duoji_ok)
    {
      __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 1720);
      __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 4160);
      __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 2500);
      //      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
      bin = 0;
      // staus_dipan = 2;
    }
    else if (bin == 2)
    {
      fangmiao++;
      bin = 0;
    }
    else if (bin == 4)
    {
      qumiao = 0;
      bin = 0;
    }
    else if (bin == 3)
    {
      qumiao = 1;
      bin = 0;
    }
    else if (bin == 5)
    {
      chongzhi = 1;
      bin = 0;
    }
    else if (bin == 6)
    {
      staus_dipan = 0;
      bin = 0;
    }
    else if (bin == 7)
    {
      __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 135);
      __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 175);
      osDelay(200);
      staus_dipan = 4;
      bin = 0;
    }
    else if (bin == 8)
    {
      quqiu_duoji = 1;
      bin = 0;
    }
    else if (bin == 9)
    {
      bin = 0;
      //      need_pos = -rotate_angle[ball_pos_];
      //      t_now = 0;
      //      while (t_now < set_T) {
      //        osDelay(1);
      //      }
      while (!yuntai_finish)
      {
        osDelay(1);
      }
      yuntai_finish = 0;
      M3508[1].ExpSpeed = back_rotate_speed[ball_pos_];
      M3508[2].ExpSpeed = shoot_speed_left[ball_pos_];
      M3508[3].ExpSpeed = -shoot_speed_right[ball_pos_];

      osDelay(500);
      __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 1866);
      osDelay(1500);
      M3508[1].ExpSpeed = 0;
      M3508[2].ExpSpeed = 0;
      M3508[3].ExpSpeed = 0;
      __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 1200);
      need_pos = (yuntai_init_pos - init_pos) * 180 / (3.14 * 6.33); // rotate_angle[ball_pos_];
      t_now = 0;
      while (t_now < set_T)
      {
        osDelay(1);
      }
    }
    else if (bin >= 10 && bin <= 21)
    {
      ball_pos_ = bin - 10;
      bin = 0;
    }
    else if (bin == 22)
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
      //	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
      //	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
      __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 3600);
      __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 1750);
      bin = 0;
    }
    osDelay(1);
    // break;

    //}
  }
  /* USER CODE END transmit_zero_ */
}

/* USER CODE BEGIN Header_chuansongdai_Task */
/**
 * @brief Function implementing the chuansongdai thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_chuansongdai_Task */
void chuansongdai_Task(void *argument)
{
  /* USER CODE BEGIN chuansongdai_Task */

  /* Infinite loop */
  for (;;)
  {

    if (HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_2) == 0 && qumiao == 0)
    {
      staus_dipan = 1;
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

      // HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2,GPIO_PIN_SET);

      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
      // HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3,GPIO_PIN_SET);
      osDelay(300);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
      // HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4,GPIO_PIN_SET);

      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
      // HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5,GPIO_PIN_SET);
      //      if(chongzhi == 1){
      //        HAL_GPIO_WritePin(GPIOI, GPIO_PIN_9, GPIO_PIN_SET);
      //        chongzhi = 0;
      //        osDelay(20);
      //        HAL_GPIO_WritePin(GPIOI, GPIO_PIN_9, GPIO_PIN_RESET);
      //
      //      }
      qumiao = 1;
    }

    if ((fangmiao == 1 && qumiao == 1))
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
      // HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4,GPIO_PIN_RESET);
      osDelay(500);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

      // HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2,GPIO_PIN_SET);
    }
    if (fangmiao == 2 && qumiao == 1)
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
      // HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5,GPIO_PIN_RESET);
      osDelay(500);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
      // HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3,GPIO_PIN_RESET);
      fangmiao = 0;
    }
    osDelay(1);
  }
  /* USER CODE END chuansongdai_Task */
}

/* USER CODE BEGIN Header_CCDTask */
/**
 * @brief Function implementing the CCD thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_CCDTask */
void CCDTask(void *argument)
{
  /* USER CODE BEGIN CCDTask */
  // CCDCONCTROL();

  /* Infinite loop */
  for (;;)
  {
    osDelay(5);
  }
  /* USER CODE END CCDTask */
}

/* USER CODE BEGIN Header_GM6020Task */
/**
 * @brief Function implementing the GM6020 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_GM6020Task */
void GM6020Task(void *argument)
{
  /* USER CODE BEGIN GM6020Task */
  /* Infinite loop */
  for (;;)
  {
    // GM6030_Motor_Position_Ctrl();
    // GM6030_Motor_Speed_Ctrl();
    // GM6030_Motor_Current_Ctrl();
    // GM6030_Motor_Current_Ctrl_h();

    osDelay(1);
  }
  /* USER CODE END GM6020Task */
}

/* USER CODE BEGIN Header_jiguang_task */

/**
 * @brief Function implementing the jiguang thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_jiguang_task */
void jiguang_task(void *argument)
{
  /* USER CODE BEGIN jiguang_task */
  HAL_GPIO_WritePin(AD7606_RST_P2_GPIO_Port, AD7606_RST_P2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AD7606_CVA_O1_GPIO_Port, AD7606_CVA_O1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(AD7606_CS_O2_GPIO_Port, AD7606_CS_O2_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(AD7606_RST_P2_GPIO_Port, AD7606_RST_P2_Pin, GPIO_PIN_SET);
  __NOP();
  __NOP();
  osDelay(1);
  __NOP();
  __NOP();
  __NOP();
  HAL_GPIO_WritePin(AD7606_RST_P2_GPIO_Port, AD7606_RST_P2_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(AD7606_CVA_O1_GPIO_Port, AD7606_CVA_O1_Pin, GPIO_PIN_RESET);
  for (uint8_t i = 0; i < 8; i++)
  {
    __NOP();
  }
    osDelay(1);
  HAL_GPIO_WritePin(AD7606_CVA_O1_GPIO_Port, AD7606_CVA_O1_Pin, GPIO_PIN_SET);
  /* Infinite loop */
  for (;;)
  {
        HAL_GPIO_WritePin(AD7606_RST_P2_GPIO_Port, AD7606_RST_P2_Pin, GPIO_PIN_SET);
  		__NOP();
  		__NOP();
  		//osDelay(1);
  		__NOP();
  		__NOP();
  		__NOP();
		__NOP();
  		__NOP();
  		HAL_GPIO_WritePin(AD7606_RST_P2_GPIO_Port, AD7606_RST_P2_Pin, GPIO_PIN_RESET);
		__NOP();
  		__NOP();
  		//osDelay(1);
  		__NOP();
  		__NOP();
  		__NOP();
		__NOP();
  		__NOP();
      HAL_GPIO_WritePin(AD7606_CVA_O1_GPIO_Port, AD7606_CVA_O1_Pin, GPIO_PIN_RESET);
      for (uint8_t i = 0; i < 7; i++)
      {
        __NOP();
      }
      //osDelay(1);
      HAL_GPIO_WritePin(AD7606_CVA_O1_GPIO_Port, AD7606_CVA_O1_Pin, GPIO_PIN_SET);
      flag_ad7606 = 0;
      jishu=0;
    osDelay(5);
  }

  /* USER CODE END jiguang_task */
}

/* USER CODE BEGIN Header_IMU_TASK */
/**
 * @brief Function implementing the IMU thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_IMU_TASK */
void IMU_TASK(void *argument)
{
  /* USER CODE BEGIN IMU_TASK */
  /* Infinite loop */
  for (;;)
  {
    imu_data.yaw[0] = (raw.imu.eul[2]) * 3.141592 / 180;
    imu_data.yaw[1] = (raw.imu.gyr[2]) * 3.141592 / 180;
    if (SBUS_Analyse_Switch(SBUS.CH7) == 0)
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
      //	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
      //	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);

      __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 3600);
      __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 1750);
    }
    else if (SBUS_Analyse_Switch(SBUS.CH7) == 2)
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
      //	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
      //	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
      __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 1720);
      __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 4160);
    }
    osDelay(1);
  }
  /* USER CODE END IMU_TASK */
}

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

#ifdef USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
