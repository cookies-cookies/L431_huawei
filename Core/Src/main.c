/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// ========== TCM ZVS 参数 ==========
volatile float32_t g_out_freq_hz = 50.0f;      // 输出频率 50Hz
volatile float32_t g_fsw_center = 40000.0f;    // 中心频率 40kHz
volatile float32_t g_fsw_delta = 20000.0f;     // 频率变化量 ±20kHz
volatile float32_t g_voltage_output = 0.8f;    // 电压控制 0~1

// ========== 虚拟三相时钟 ==========
// TIM1中断频率 = 200kHz（支持5~35kHz PWM）
// 每相独立计数器，实现120°相位差
#define TIM1_IRQ_FREQ   200000.0f   // TIM1中断频率 200kHz

// 三相虚拟计数器（0~360度循环）
volatile uint16_t g_phase_cnt_a = 0;    // A相计数器
volatile uint16_t g_phase_cnt_b = 0;    // B相计数器  
volatile uint16_t g_phase_cnt_c = 0;    // C相计数器

// 三相目标半周期计数值（由TIM2计算）
volatile uint16_t g_half_period_a = 100;   // A相半周期 (200kHz/1kHz/2=100)
volatile uint16_t g_half_period_b = 100;   // B相半周期
volatile uint16_t g_half_period_c = 100;   // C相半周期

// 三相当前输出状态
volatile uint8_t g_state_a = 0;  // A相状态 0/1
volatile uint8_t g_state_b = 0;  // B相状态 0/1
volatile uint8_t g_state_c = 0;  // C相状态 0/1

// 三相死区计数器
volatile uint8_t g_dead_cnt_a = 0;  // A相死区计数
volatile uint8_t g_dead_cnt_b = 0;  // B相死区计数
volatile uint8_t g_dead_cnt_c = 0;  // C相死区计数

// ========== 三相互补PWM GPIO引脚定义 ==========
// A相: 高侧PA8, 低侧PA7
#define GPIO_AH_PORT    GPIOA
#define GPIO_AH_PIN     GPIO_PIN_8
#define GPIO_AL_PORT    GPIOA
#define GPIO_AL_PIN     GPIO_PIN_7

// B相: 高侧PA9, 低侧PB0
#define GPIO_BH_PORT    GPIOA
#define GPIO_BH_PIN     GPIO_PIN_9
#define GPIO_BL_PORT    GPIOB
#define GPIO_BL_PIN     GPIO_PIN_0

// C相: 高侧PA10, 低侧PB1
#define GPIO_CH_PORT    GPIOA
#define GPIO_CH_PIN     GPIO_PIN_10
#define GPIO_CL_PORT    GPIOB
#define GPIO_CL_PIN     GPIO_PIN_1

// 死区时间计数 (200kHz中断, 死区约500ns = 0.1个计数，取1个计数)
#define DEAD_TIME_COUNT 1
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  // ========== TIM1 高频中断 (200kHz) - GPIO翻转 ==========
  // ARR = 80MHz / 200kHz - 1 = 399
  __HAL_TIM_SET_AUTORELOAD(&htim1, 399);
  HAL_TIM_Base_Start_IT(&htim1);
  
  // ========== TIM2 计算中断 (20kHz) - 计算三相参数 ==========
  // ARR = 80MHz / 20kHz - 1 = 3999
  __HAL_TIM_SET_AUTORELOAD(&htim2, 3999);
  HAL_TIM_Base_Start_IT(&htim2);
  
  // 初始化三相相位差 (120°)
  // B相初始延迟 1/3 周期，C相初始延迟 2/3 周期
  g_phase_cnt_b = 33;    // 120° / 360° * 100 ≈ 33
  g_phase_cnt_c = 67;    // 240° / 360° * 100 ≈ 67
  /* USER CODE END 2 */

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_8;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_3;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 400-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 4000-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 2000-1;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 2000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // ========== 三相互补PWM GPIO初始化 ==========
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  // 配置为高速推挽输出
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  
  // A相: PA8(高侧), PA7(低侧)
  GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);  // AL = 0
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);  // AH = 0
  
  // B相: PA9(高侧), PB0(低侧)
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);  // BH = 0
  
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  // BL = 0
  
  // C相: PA10(高侧), PB1(低侧)
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // CH = 0
  
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);  // CL = 0
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ========== TIM1 高频中断 (200kHz) - GPIO翻转产生三相互补PWM ==========
// 状态机状态定义
#define STATE_HIGH      0   // 高侧导通
#define STATE_DEAD_HL   1   // 死区(高->低)
#define STATE_LOW       2   // 低侧导通
#define STATE_DEAD_LH   3   // 死区(低->高)

void TIM1_UP_TIM16_IRQHandler(void)  // TIM1更新中断
{
  if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE))
  {
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    
    // ========== A相处理 ==========
    g_phase_cnt_a++;
    if (g_phase_cnt_a >= g_half_period_a) {
      g_phase_cnt_a = 0;
      // 状态切换: HIGH -> DEAD_HL -> LOW -> DEAD_LH -> HIGH
      g_state_a = (g_state_a + 1) & 0x03;  // 0->1->2->3->0 循环
      
      switch(g_state_a) {
        case STATE_HIGH:    // 高侧导通，低侧关断
          GPIO_AH_PORT->BSRR = GPIO_AH_PIN;          // AH = 1
          GPIO_AL_PORT->BRR  = GPIO_AL_PIN;          // AL = 0
          break;
        case STATE_DEAD_HL: // 死区：两个都关断
          GPIO_AH_PORT->BRR  = GPIO_AH_PIN;          // AH = 0
          GPIO_AL_PORT->BRR  = GPIO_AL_PIN;          // AL = 0
          g_dead_cnt_a = DEAD_TIME_COUNT;
          break;
        case STATE_LOW:     // 低侧导通，高侧关断
          GPIO_AH_PORT->BRR  = GPIO_AH_PIN;          // AH = 0
          GPIO_AL_PORT->BSRR = GPIO_AL_PIN;          // AL = 1
          break;
        case STATE_DEAD_LH: // 死区：两个都关断
          GPIO_AH_PORT->BRR  = GPIO_AH_PIN;          // AH = 0
          GPIO_AL_PORT->BRR  = GPIO_AL_PIN;          // AL = 0
          g_dead_cnt_a = DEAD_TIME_COUNT;
          break;
      }
    }
    // 死区计数处理
    if (g_dead_cnt_a > 0) {
      g_dead_cnt_a--;
      if (g_dead_cnt_a == 0) {
        // 死区结束，进入下一状态
        if (g_state_a == STATE_DEAD_HL) {
          g_state_a = STATE_LOW;
          GPIO_AL_PORT->BSRR = GPIO_AL_PIN;          // AL = 1
        } else if (g_state_a == STATE_DEAD_LH) {
          g_state_a = STATE_HIGH;
          GPIO_AH_PORT->BSRR = GPIO_AH_PIN;          // AH = 1
        }
      }
    }
    
    // ========== B相处理 (120°相位差) ==========
    g_phase_cnt_b++;
    if (g_phase_cnt_b >= g_half_period_b) {
      g_phase_cnt_b = 0;
      g_state_b = (g_state_b + 1) & 0x03;
      
      switch(g_state_b) {
        case STATE_HIGH:
          GPIO_BH_PORT->BSRR = GPIO_BH_PIN;
          GPIO_BL_PORT->BRR  = GPIO_BL_PIN;
          break;
        case STATE_DEAD_HL:
          GPIO_BH_PORT->BRR  = GPIO_BH_PIN;
          GPIO_BL_PORT->BRR  = GPIO_BL_PIN;
          g_dead_cnt_b = DEAD_TIME_COUNT;
          break;
        case STATE_LOW:
          GPIO_BH_PORT->BRR  = GPIO_BH_PIN;
          GPIO_BL_PORT->BSRR = GPIO_BL_PIN;
          break;
        case STATE_DEAD_LH:
          GPIO_BH_PORT->BRR  = GPIO_BH_PIN;
          GPIO_BL_PORT->BRR  = GPIO_BL_PIN;
          g_dead_cnt_b = DEAD_TIME_COUNT;
          break;
      }
    }
    if (g_dead_cnt_b > 0) {
      g_dead_cnt_b--;
      if (g_dead_cnt_b == 0) {
        if (g_state_b == STATE_DEAD_HL) {
          g_state_b = STATE_LOW;
          GPIO_BL_PORT->BSRR = GPIO_BL_PIN;
        } else if (g_state_b == STATE_DEAD_LH) {
          g_state_b = STATE_HIGH;
          GPIO_BH_PORT->BSRR = GPIO_BH_PIN;
        }
      }
    }
    
    // ========== C相处理 (240°相位差) ==========
    g_phase_cnt_c++;
    if (g_phase_cnt_c >= g_half_period_c) {
      g_phase_cnt_c = 0;
      g_state_c = (g_state_c + 1) & 0x03;
      
      switch(g_state_c) {
        case STATE_HIGH:
          GPIO_CH_PORT->BSRR = GPIO_CH_PIN;
          GPIO_CL_PORT->BRR  = GPIO_CL_PIN;
          break;
        case STATE_DEAD_HL:
          GPIO_CH_PORT->BRR  = GPIO_CH_PIN;
          GPIO_CL_PORT->BRR  = GPIO_CL_PIN;
          g_dead_cnt_c = DEAD_TIME_COUNT;
          break;
        case STATE_LOW:
          GPIO_CH_PORT->BRR  = GPIO_CH_PIN;
          GPIO_CL_PORT->BSRR = GPIO_CL_PIN;
          break;
        case STATE_DEAD_LH:
          GPIO_CH_PORT->BRR  = GPIO_CH_PIN;
          GPIO_CL_PORT->BRR  = GPIO_CL_PIN;
          g_dead_cnt_c = DEAD_TIME_COUNT;
          break;
      }
    }
    if (g_dead_cnt_c > 0) {
      g_dead_cnt_c--;
      if (g_dead_cnt_c == 0) {
        if (g_state_c == STATE_DEAD_HL) {
          g_state_c = STATE_LOW;
          GPIO_CL_PORT->BSRR = GPIO_CL_PIN;
        } else if (g_state_c == STATE_DEAD_LH) {
          g_state_c = STATE_HIGH;
          GPIO_CH_PORT->BSRR = GPIO_CH_PIN;
        }
      }
    }
  }
}

// ========== TIM2 计算中断 (20kHz) - 计算三相半周期 ==========
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static float32_t phase_deg = 0.0f;
  
  if (htim->Instance == TIM2)
  {
    // 50Hz相位递增 (20kHz中断频率)
    phase_deg += g_out_freq_hz * 360.0f / 20000.0f;
    if (phase_deg >= 360.0f) {
      phase_deg -= 360.0f;
    }
    
    // 计算三相频率对应的半周期计数值
    // 半周期 = (60kHz / f_sw) / 2
    float32_t sin_a, sin_b, sin_c;
    arm_sin_cos_f32(phase_deg, &sin_a, NULL);
    arm_sin_cos_f32(phase_deg + 120.0f, &sin_b, NULL);
    arm_sin_cos_f32(phase_deg + 240.0f, &sin_c, NULL);
    
    // TCM ZVS频率调制
    float32_t f_a = g_fsw_center + g_fsw_delta * sin_a * g_voltage_output;
    float32_t f_b = g_fsw_center + g_fsw_delta * sin_b * g_voltage_output;
    float32_t f_c = g_fsw_center + g_fsw_delta * sin_c * g_voltage_output;
    
    // 转换为半周期计数值 (TIM1中断频率60kHz)
    // half_period = TIM1_IRQ_FREQ / (2 * f_sw)
    g_half_period_a = (uint16_t)(TIM1_IRQ_FREQ / (2.0f * f_a));
    g_half_period_b = (uint16_t)(TIM1_IRQ_FREQ / (2.0f * f_b));
    g_half_period_c = (uint16_t)(TIM1_IRQ_FREQ / (2.0f * f_c));
    
    // 限制最小值，防止频率过高
    if (g_half_period_a < 5) g_half_period_a = 5;
    if (g_half_period_b < 5) g_half_period_b = 5;
    if (g_half_period_c < 5) g_half_period_c = 5;
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
