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
  * If no LICENSE file accompanies this software, it is provided AS-IS.
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
volatile float32_t g_fsw_center = 5000.0f;     // 中心频率 5kHz
volatile float32_t g_fsw_delta = 3000.0f;      // 频率变化量 ±3kHz
volatile float32_t g_voltage_output = 0.8f;    // 电压控制 0~1
volatile float32_t g_duty_cycle = 0.5f;        // 占空比 0~1

// ========== Sin/Cos查表 (360点，每度一个值) ==========
#define SIN_TABLE_SIZE 360
static const float32_t sin_table[360] = {
    0.0000f, 0.0175f, 0.0349f, 0.0523f, 0.0698f, 0.0872f, 0.1045f, 0.1219f, 0.1392f, 0.1564f,
    0.1736f, 0.1908f, 0.2079f, 0.2250f, 0.2419f, 0.2588f, 0.2756f, 0.2924f, 0.3090f, 0.3256f,
    0.3420f, 0.3584f, 0.3746f, 0.3907f, 0.4067f, 0.4226f, 0.4384f, 0.4540f, 0.4695f, 0.4848f,
    0.5000f, 0.5150f, 0.5299f, 0.5446f, 0.5592f, 0.5736f, 0.5878f, 0.6018f, 0.6157f, 0.6293f,
    0.6428f, 0.6561f, 0.6691f, 0.6820f, 0.6947f, 0.7071f, 0.7193f, 0.7314f, 0.7431f, 0.7547f,
    0.7660f, 0.7771f, 0.7880f, 0.7986f, 0.8090f, 0.8192f, 0.8290f, 0.8387f, 0.8480f, 0.8572f,
    0.8660f, 0.8746f, 0.8829f, 0.8910f, 0.8988f, 0.9063f, 0.9135f, 0.9205f, 0.9272f, 0.9336f,
    0.9397f, 0.9455f, 0.9511f, 0.9563f, 0.9613f, 0.9659f, 0.9703f, 0.9744f, 0.9781f, 0.9816f,
    0.9848f, 0.9877f, 0.9903f, 0.9925f, 0.9945f, 0.9962f, 0.9976f, 0.9986f, 0.9994f, 0.9998f,
    1.0000f, 0.9998f, 0.9994f, 0.9986f, 0.9976f, 0.9962f, 0.9945f, 0.9925f, 0.9903f, 0.9877f,
    0.9848f, 0.9816f, 0.9781f, 0.9744f, 0.9703f, 0.9659f, 0.9613f, 0.9563f, 0.9511f, 0.9455f,
    0.9397f, 0.9336f, 0.9272f, 0.9205f, 0.9135f, 0.9063f, 0.8988f, 0.8910f, 0.8829f, 0.8746f,
    0.8660f, 0.8572f, 0.8480f, 0.8387f, 0.8290f, 0.8192f, 0.8090f, 0.7986f, 0.7880f, 0.7771f,
    0.7660f, 0.7547f, 0.7431f, 0.7314f, 0.7193f, 0.7071f, 0.6947f, 0.6820f, 0.6691f, 0.6561f,
    0.6428f, 0.6293f, 0.6157f, 0.6018f, 0.5878f, 0.5736f, 0.5592f, 0.5446f, 0.5299f, 0.5150f,
    0.5000f, 0.4848f, 0.4695f, 0.4540f, 0.4384f, 0.4226f, 0.4067f, 0.3907f, 0.3746f, 0.3584f,
    0.3420f, 0.3256f, 0.3090f, 0.2924f, 0.2756f, 0.2588f, 0.2419f, 0.2250f, 0.2079f, 0.1908f,
    0.1736f, 0.1564f, 0.1392f, 0.1219f, 0.1045f, 0.0872f, 0.0698f, 0.0523f, 0.0349f, 0.0175f,
    0.0000f,-0.0175f,-0.0349f,-0.0523f,-0.0698f,-0.0872f,-0.1045f,-0.1219f,-0.1392f,-0.1564f,
   -0.1736f,-0.1908f,-0.2079f,-0.2250f,-0.2419f,-0.2588f,-0.2756f,-0.2924f,-0.3090f,-0.3256f,
   -0.3420f,-0.3584f,-0.3746f,-0.3907f,-0.4067f,-0.4226f,-0.4384f,-0.4540f,-0.4695f,-0.4848f,
   -0.5000f,-0.5150f,-0.5299f,-0.5446f,-0.5592f,-0.5736f,-0.5878f,-0.6018f,-0.6157f,-0.6293f,
   -0.6428f,-0.6561f,-0.6691f,-0.6820f,-0.6947f,-0.7071f,-0.7193f,-0.7314f,-0.7431f,-0.7547f,
   -0.7660f,-0.7771f,-0.7880f,-0.7986f,-0.8090f,-0.8192f,-0.8290f,-0.8387f,-0.8480f,-0.8572f,
   -0.8660f,-0.8746f,-0.8829f,-0.8910f,-0.8988f,-0.9063f,-0.9135f,-0.9205f,-0.9272f,-0.9336f,
   -0.9397f,-0.9455f,-0.9511f,-0.9563f,-0.9613f,-0.9659f,-0.9703f,-0.9744f,-0.9781f,-0.9816f,
   -0.9848f,-0.9877f,-0.9903f,-0.9925f,-0.9945f,-0.9962f,-0.9976f,-0.9986f,-0.9994f,-0.9998f,
   -1.0000f,-0.9998f,-0.9994f,-0.9986f,-0.9976f,-0.9962f,-0.9945f,-0.9925f,-0.9903f,-0.9877f,
   -0.9848f,-0.9816f,-0.9781f,-0.9744f,-0.9703f,-0.9659f,-0.9613f,-0.9563f,-0.9511f,-0.9455f,
   -0.9397f,-0.9336f,-0.9272f,-0.9205f,-0.9135f,-0.9063f,-0.8988f,-0.8910f,-0.8829f,-0.8746f,
   -0.8660f,-0.8572f,-0.8480f,-0.8387f,-0.8290f,-0.8192f,-0.8090f,-0.7986f,-0.7880f,-0.7771f,
   -0.7660f,-0.7547f,-0.7431f,-0.7314f,-0.7193f,-0.7071f,-0.6947f,-0.6820f,-0.6691f,-0.6561f,
   -0.6428f,-0.6293f,-0.6157f,-0.6018f,-0.5878f,-0.5736f,-0.5592f,-0.5446f,-0.5299f,-0.5150f,
   -0.5000f,-0.4848f,-0.4695f,-0.4540f,-0.4384f,-0.4226f,-0.4067f,-0.3907f,-0.3746f,-0.3584f,
   -0.3420f,-0.3256f,-0.3090f,-0.2924f,-0.2756f,-0.2588f,-0.2419f,-0.2250f,-0.2079f,-0.1908f,
   -0.1736f,-0.1564f,-0.1392f,-0.1219f,-0.1045f,-0.0872f,-0.0698f,-0.0523f,-0.0349f,-0.0175f
};
// cos表 = sin表偏移90度
#define cos_table(idx) sin_table[((idx) + 90) % 360]

// ========== 虚拟三相时钟 ==========
// TIM1中断频率 = 100kHz（ARR=1000-1, 100MHz/1000=100kHz）
#define TIM1_IRQ_FREQ   100000.0f

// 三相虚拟计数器
volatile uint16_t g_phase_cnt_a = 0;
volatile uint16_t g_phase_cnt_b = 0;
volatile uint16_t g_phase_cnt_c = 0;

// 三相目标周期计数值
volatile uint16_t g_period_a = 40;
volatile uint16_t g_period_b = 40;
volatile uint16_t g_period_c = 40;

// 三相HIGH时间计数
volatile uint16_t g_high_cnt_a = 20;
volatile uint16_t g_high_cnt_b = 20;
volatile uint16_t g_high_cnt_c = 20;

// 三相当前输出状态
#define STATE_HIGH      0
#define STATE_LOW       2
volatile uint8_t g_state_a = STATE_HIGH;
volatile uint8_t g_state_b = STATE_HIGH;
volatile uint8_t g_state_c = STATE_HIGH;

// ========== 三相互补PWM GPIO引脚定义 ==========
// A相: 高侧PA8, 低侧PB13
#define GPIO_AH_PORT    GPIOA
#define GPIO_AH_PIN     GPIO_PIN_8
#define GPIO_AL_PORT    GPIOB
#define GPIO_AL_PIN     GPIO_PIN_13

// B相: 高侧PA9, 低侧PB14
#define GPIO_BH_PORT    GPIOA
#define GPIO_BH_PIN     GPIO_PIN_9
#define GPIO_BL_PORT    GPIOB
#define GPIO_BL_PIN     GPIO_PIN_14

// C相: 高侧PA10, 低侧PB1
#define GPIO_CH_PORT    GPIOA
#define GPIO_CH_PIN     GPIO_PIN_10
#define GPIO_CL_PORT    GPIOB
#define GPIO_CL_PIN     GPIO_PIN_1
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM7_Init(void);
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim7);
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
  RCC_OscInitStruct.PLL.PLLN = 50;
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
  hadc1.Init.DMAContinuousRequests = DISABLE;
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
  htim1.Init.Period = 1000-1;
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
  htim7.Init.Period = 10000-1;
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
  // TIM7 = 10kHz (ARR=10000-1, 100MHz/10000=10kHz)
  /* USER CODE END TIM7_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB1 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// TIM1 200kHz中断 - 汇编优化版本，最小化中断延迟
void TIM1_Update_Handler(void)
{
    __asm volatile (
        // ========== A相处理 ==========
        "ldr r0, =g_phase_cnt_a \n\t"
        "ldr r1, =g_period_a \n\t"
        "ldr r2, =g_high_cnt_a \n\t"
        "ldr r3, =g_state_a \n\t"
        
        "ldrh r4, [r0] \n\t"         // r4 = g_phase_cnt_a
        "add r4, r4, #1 \n\t"        // g_phase_cnt_a++
        "strh r4, [r0] \n\t"         // 保存
        
        "ldrh r5, [r1] \n\t"         // r5 = g_period_a
        "cmp r4, r5 \n\t"            // if (cnt >= period)
        "blt a_check_high \n\t"
        
        // A相复位到HIGH
        "mov r4, #0 \n\t"
        "strh r4, [r0] \n\t"         // g_phase_cnt_a = 0
        "mov r6, #0 \n\t"
        "strb r6, [r3] \n\t"         // g_state_a = STATE_HIGH
        
        // GPIO: PA8=1, PB13=0
        "ldr r7, =0x48000018 \n\t"   // GPIOA BSRR (基地址0x48000000)
        "ldr r6, =0x00000100 \n\t"   // PA8 set
        "str r6, [r7] \n\t"
        "ldr r7, =0x48000428 \n\t"   // GPIOB BRR
        "ldr r6, =0x2000 \n\t"       // PB13 reset
        "str r6, [r7] \n\t"
        "b b_phase \n\t"
        
        "a_check_high: \n\t"
        "ldrh r5, [r2] \n\t"         // r5 = g_high_cnt_a
        "cmp r4, r5 \n\t"            // if (cnt >= high_cnt)
        "blt b_phase \n\t"
        "ldrb r6, [r3] \n\t"         // r6 = g_state_a
        "cmp r6, #0 \n\t"            // if (state == STATE_HIGH)
        "bne b_phase \n\t"
        
        // A相切换到LOW
        "mov r6, #2 \n\t"
        "strb r6, [r3] \n\t"         // g_state_a = STATE_LOW
        "ldr r7, =0x48000028 \n\t"   // GPIOA BRR (基地址0x48000000)
        "ldr r6, =0x0100 \n\t"       // PA8 reset
        "str r6, [r7] \n\t"
        "ldr r7, =0x48000418 \n\t"   // GPIOB BSRR
        "ldr r6, =0x2000 \n\t"       // PB13 set
        "str r6, [r7] \n\t"
        
        // ========== B相处理 ==========
        "b_phase: \n\t"
        "ldr r0, =g_phase_cnt_b \n\t"
        "ldr r1, =g_period_b \n\t"
        "ldr r2, =g_high_cnt_b \n\t"
        "ldr r3, =g_state_b \n\t"
        
        "ldrh r4, [r0] \n\t"
        "add r4, r4, #1 \n\t"
        "strh r4, [r0] \n\t"
        
        "ldrh r5, [r1] \n\t"
        "cmp r4, r5 \n\t"
        "blt b_check_high \n\t"
        
        // B相复位到HIGH
        "mov r4, #0 \n\t"
        "strh r4, [r0] \n\t"
        "mov r6, #0 \n\t"
        "strb r6, [r3] \n\t"
        
        // GPIO: PA9=1, PB14=0
        "ldr r7, =0x48000018 \n\t"   // GPIOA BSRR (基地址0x48000000)
        "ldr r6, =0x00000200 \n\t"   // PA9 set
        "str r6, [r7] \n\t"
        "ldr r7, =0x48000428 \n\t"
        "ldr r6, =0x4000 \n\t"       // PB14 reset
        "str r6, [r7] \n\t"
        "b c_phase \n\t"
        
        "b_check_high: \n\t"
        "ldrh r5, [r2] \n\t"
        "cmp r4, r5 \n\t"
        "blt c_phase \n\t"
        "ldrb r6, [r3] \n\t"
        "cmp r6, #0 \n\t"
        "bne c_phase \n\t"
        
        // B相切换到LOW
        "mov r6, #2 \n\t"
        "strb r6, [r3] \n\t"
        "ldr r7, =0x48000028 \n\t"   // GPIOA BRR (基地址0x48000000)
        "ldr r6, =0x0200 \n\t"       // PA9 reset
        "str r6, [r7] \n\t"
        "ldr r7, =0x48000418 \n\t"
        "ldr r6, =0x4000 \n\t"       // PB14 set
        "str r6, [r7] \n\t"
        
        // ========== C相处理 ==========
        "c_phase: \n\t"
        "ldr r0, =g_phase_cnt_c \n\t"
        "ldr r1, =g_period_c \n\t"
        "ldr r2, =g_high_cnt_c \n\t"
        "ldr r3, =g_state_c \n\t"
        
        "ldrh r4, [r0] \n\t"
        "add r4, r4, #1 \n\t"
        "strh r4, [r0] \n\t"
        
        "ldrh r5, [r1] \n\t"
        "cmp r4, r5 \n\t"
        "blt c_check_high \n\t"
        
        // C相复位到HIGH
        "mov r4, #0 \n\t"
        "strh r4, [r0] \n\t"
        "mov r6, #0 \n\t"
        "strb r6, [r3] \n\t"
        
        // GPIO: PA10=1, PB1=0
        "ldr r7, =0x48000018 \n\t"   // GPIOA BSRR (基地址0x48000000)
        "ldr r6, =0x00000400 \n\t"   // PA10 set
        "str r6, [r7] \n\t"
        "ldr r7, =0x48000428 \n\t"
        "ldr r6, =0x0002 \n\t"       // PB1 reset
        "str r6, [r7] \n\t"
        "b done_asm \n\t"
        
        "c_check_high: \n\t"
        "ldrh r5, [r2] \n\t"
        "cmp r4, r5 \n\t"
        "blt done_asm \n\t"
        "ldrb r6, [r3] \n\t"
        "cmp r6, #0 \n\t"
        "bne done_asm \n\t"
        
        // C相切换到LOW
        "mov r6, #2 \n\t"
        "strb r6, [r3] \n\t"
        "ldr r7, =0x48000028 \n\t"   // GPIOA BRR (基地址0x48000000)
        "ldr r6, =0x0400 \n\t"       // PA10 reset
        "str r6, [r7] \n\t"
        "ldr r7, =0x48000418 \n\t"
        "ldr r6, =0x0002 \n\t"       // PB1 set
        "str r6, [r7] \n\t"
        
        "done_asm: \n\t"
        : // 无输出
        : // 无输入
        : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "memory", "cc"
    );
}

// TIM7 10kHz中断 - 计算三相参数（使用查表法加速）
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint16_t phase_idx = 0;  // 查表索引 0-359
  
  if (htim->Instance == TIM7)
  {
    // 更新相位索引 (50Hz / 10000Hz * 360 = 1.8度/次)
    phase_idx += (uint16_t)(g_out_freq_hz * 360.0f / 10000.0f);
    if (phase_idx >= 360) {
      phase_idx -= 360;
    }
    
    // 查表获取sin/cos值（比arm_sin_cos_f32快10-100倍）
    float32_t sin_a = sin_table[phase_idx];
    float32_t cos_a = sin_table[(phase_idx + 90) % 360];
    
    // 三相120度偏移
    float32_t sin_b = -0.5f * sin_a + 0.866025404f * cos_a;
    float32_t sin_c = -0.5f * sin_a - 0.866025404f * cos_a;
    
    float32_t f_a = g_fsw_center + g_fsw_delta * sin_a * g_voltage_output;
    float32_t f_b = g_fsw_center + g_fsw_delta * sin_b * g_voltage_output;
    float32_t f_c = g_fsw_center + g_fsw_delta * sin_c * g_voltage_output;

    uint16_t new_period_a = (uint16_t)(TIM1_IRQ_FREQ / f_a);
    uint16_t new_period_b = (uint16_t)(TIM1_IRQ_FREQ / f_b);
    uint16_t new_period_c = (uint16_t)(TIM1_IRQ_FREQ / f_c);
    
    uint16_t new_high_a = (uint16_t)(new_period_a * g_duty_cycle);
    uint16_t new_high_b = (uint16_t)(new_period_b * g_duty_cycle);
    uint16_t new_high_c = (uint16_t)(new_period_c * g_duty_cycle);
    
    // A相同步
    if (g_phase_cnt_a >= new_period_a) {
        g_phase_cnt_a = 0;
        g_state_a = STATE_HIGH;
        GPIO_AH_PORT->BSRR = GPIO_AH_PIN;
        GPIO_AL_PORT->BRR  = GPIO_AL_PIN;
    } else if (g_state_a == STATE_HIGH && g_phase_cnt_a >= new_high_a) {
        GPIO_AH_PORT->BRR  = GPIO_AH_PIN;
        GPIO_AL_PORT->BSRR = GPIO_AL_PIN;
        g_state_a = STATE_LOW;
    }
    g_period_a = new_period_a;
    g_high_cnt_a = new_high_a;
    
    // B相同步
    if (g_phase_cnt_b >= new_period_b) {
        g_phase_cnt_b = 0;
        g_state_b = STATE_HIGH;
        GPIO_BH_PORT->BSRR = GPIO_BH_PIN;
        GPIO_BL_PORT->BRR  = GPIO_BL_PIN;
    } else if (g_state_b == STATE_HIGH && g_phase_cnt_b >= new_high_b) {
        GPIO_BH_PORT->BRR  = GPIO_BH_PIN;
        GPIO_BL_PORT->BSRR = GPIO_BL_PIN;
        g_state_b = STATE_LOW;
    }
    g_period_b = new_period_b;
    g_high_cnt_b = new_high_b;
    
    // C相同步
    if (g_phase_cnt_c >= new_period_c) {
        g_phase_cnt_c = 0;
        g_state_c = STATE_HIGH;
        GPIO_CH_PORT->BSRR = GPIO_CH_PIN;
        GPIO_CL_PORT->BRR  = GPIO_CL_PIN;
    } else if (g_state_c == STATE_HIGH && g_phase_cnt_c >= new_high_c) {
        GPIO_CH_PORT->BRR  = GPIO_CH_PIN;
        GPIO_CL_PORT->BSRR = GPIO_CL_PIN;
        g_state_c = STATE_LOW;
    }
    g_period_c = new_period_c;
    g_high_cnt_c = new_high_c;
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
