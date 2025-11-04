/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "fonts.h"
#include "ssd1306.h"
#include "stdio.h"

#include "ai_datatypes_defines.h"
#include "ai_platform.h"
#include "sine_model_own.h"
#include "sine_model_own_data.h"
#include <math.h>

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
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */
// --- Stepper 28BYJ-48 on ULN2003 ---
#define STP_IN1_GPIO_Port   GPIOB
#define STP_IN1_Pin         GPIO_PIN_0
#define STP_IN2_GPIO_Port   GPIOB
#define STP_IN2_Pin         GPIO_PIN_1
#define STP_IN3_GPIO_Port   GPIOB
#define STP_IN3_Pin         GPIO_PIN_2
#define STP_IN4_GPIO_Port   GPIOB
#define STP_IN4_Pin         GPIO_PIN_10

// Konfiguracja: half-step (8 stanów)
static const uint8_t stp_seq[8][4] = {
  {1,0,0,0},
  {1,1,0,0},
  {0,1,0,0},
  {0,1,1,0},
  {0,0,1,0},
  {0,0,1,1},
  {0,0,0,1},
  {1,0,0,1}
};

static volatile int      stp_idx = 0;
static volatile int      stp_dir = 0;          // -1, 0, +1
static volatile uint32_t stp_ticks = 0;        // odległość (w taktach timera) do następnego kroku
static const float       STP_FMAX_HZ = 500.0f; // max pół-kroków/s (bezpieczne dla 28BYJ-48)
static const float       STP_FMIN_HZ = 2.0f;   // próg „ruszenia”, żeby uniknąć zbyt wolnych kliknięć



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define G_W (SSD1306_WIDTH)
#define G_H (SSD1306_HEIGHT)

static int16_t g_y[G_W];
static uint16_t g_head = 0;

static inline int clampi(int v, int lo, int hi) { return (v<lo)?lo:((v>hi)?hi:v); } // ogranicznik wartosci min/max

void Graph_Init(void)
{
    for (int i = 0; i < G_W; ++i) g_y[i] = G_H/2;
    g_head = 0;
    SSD1306_Clear();
    SSD1306_UpdateScreen();
}

void Graph_PushAndDraw(float y, float y_min, float y_max, const char *label, uint32_t dur)
{
    float norm = (y - y_min) / (y_max - y_min);
    int py = (int)((1.0f - norm) * (G_H - 1));
    py = clampi(py, 0, G_H-1);

    g_y[g_head] = py;
    g_head = (g_head + 1) % G_W;

    SSD1306_Fill(SSD1306_COLOR_BLACK);

    int mid = G_H/2;
    for (int x = 0; x < G_W; ++x)
        SSD1306_DrawPixel(x, mid, SSD1306_COLOR_WHITE);

    // Przewijanie
    int prevx = 0;
    int prevy = g_y[g_head % G_W];
    for (int i = 1; i < G_W; ++i) {
        int idx = (g_head + i) % G_W;
        int ypix = g_y[idx];
        SSD1306_DrawLine(prevx, prevy, i, ypix, SSD1306_COLOR_WHITE);
        prevx = i; prevy = ypix;
    }

    if (label) {
        SSD1306_GotoXY(0, 0);
        SSD1306_Puts((char*)label, &Font_7x10, 1);
    }
    char line[24];
   	  SSD1306_GotoXY(0, 50);
   	  snprintf(line, sizeof(line), "t=%l8 us", (unsigned long)dur);
   	  SSD1306_Puts(line, &Font_7x10, 1);
    SSD1306_UpdateScreen();
}

void HandleOutput()
{
	static uint8_t is_initialized = 0;

	if(!is_initialized)
	{
		HAL_GPIO_WritePin(GPIOA, Led_GREEN_Pin, GPIO_PIN_RESET);
		is_initialized = 1;
	}
}

static inline uint16_t pwm_from_y(float y, uint16_t arr)
{
    float t = (y + 1.0f) * 0.5f;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return (uint16_t)(t * (float)arr);
}

//////////////////////// stepmotor ///////////////////////

// 1 MHz? 1.68 MHz? – liczona z aktualnych ustawień timera.
// U Ciebie PSC=49 → tick = 84 MHz / (49+1) = 1.68 MHz.
static inline uint32_t TIM1_TICK_HZ(void) {
  uint32_t timclk = HAL_RCC_GetPCLK2Freq(); // TIM1 na APB2
  // jeśli APB2 prescaler > 1 to timerclk = 2*PCLK2 (u Ciebie APB2=DIV1, więc bez mnożnika)
  return timclk / (htim1.Init.Prescaler + 1);
}

static inline void Stepper_ApplyPhase(int idx) {
  const uint8_t *ph = stp_seq[idx & 7];
  HAL_GPIO_WritePin(STP_IN1_GPIO_Port, STP_IN1_Pin, ph[0]?GPIO_PIN_SET:GPIO_PIN_RESET);
  HAL_GPIO_WritePin(STP_IN2_GPIO_Port, STP_IN2_Pin, ph[1]?GPIO_PIN_SET:GPIO_PIN_RESET);
  HAL_GPIO_WritePin(STP_IN3_GPIO_Port, STP_IN3_Pin, ph[2]?GPIO_PIN_SET:GPIO_PIN_RESET);
  HAL_GPIO_WritePin(STP_IN4_GPIO_Port, STP_IN4_Pin, ph[3]?GPIO_PIN_SET:GPIO_PIN_RESET);
}

static inline void Stepper_AllOff(void) {
  HAL_GPIO_WritePin(STP_IN1_GPIO_Port, STP_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(STP_IN2_GPIO_Port, STP_IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(STP_IN3_GPIO_Port, STP_IN3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(STP_IN4_GPIO_Port, STP_IN4_Pin, GPIO_PIN_RESET);
}

// Wylicz prędkość z y_val i przelicz na odstęp czasowy (w tickach TIM1) między krokami.
static inline void Stepper_SetFromAI(float y) {
  float a = fabsf(y);
  if (a < 0.001f) {
    stp_dir   = 0;
    Stepper_AllOff();
    return;
  }

  stp_dir = (y >= 0.0f) ? +1 : -1;

	// Skaluje |y| → [FMIN..FMAX]; poniżej FMIN – stop i off (żeby nie grzać cewek)
	float f = STP_FMIN_HZ + (STP_FMAX_HZ - STP_FMIN_HZ) * a;
	if (f < STP_FMIN_HZ) {
	  stp_dir = 0;
	  Stepper_AllOff();
	  return;
	}

	uint32_t tick_hz = TIM1_TICK_HZ();        // np. 1_680_000 Hz
	  uint32_t ticks   = (uint32_t)( (float)tick_hz / f );
	  if (ticks < 200)  ticks = 200;            // górny limit prędkości (ochrona)
	  if (ticks > 1000000) ticks = 1000000;     // dolny limit (unikaj overflow)

	  stp_ticks = ticks;
	}

	// Jeden pół-krok (wołane z przerwania OC1)
	static inline void Stepper_StepISR(void) {
	  if (stp_dir == 0) return;
	  stp_idx += stp_dir;
	  Stepper_ApplyPhase(stp_idx);
	}

	// Callback od TIM1 OC1
	void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
	{
	  if (htim->Instance == TIM1 && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
	  {
	    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
	    uint32_t cnt = __HAL_TIM_GET_COUNTER(&htim1);

	    // Jeżeli silnik stoi (stp_ticks = 0), daj mały odstęp, żeby timer nie umarł
	    uint32_t step = stp_ticks ? stp_ticks : ((arr + 1) >> 1);

	    // Oblicz nowy moment porównania z zawinięciem po ARR
	    uint32_t next = (cnt + step) % (arr + 1);
	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, next);

	    Stepper_StepISR(); // wykonaj półkrok
	  }
	}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	char buff[50];

	int buf_len = 0;
	ai_error ai_err;
	ai_i32 nbatch;
	volatile uint32_t timestamp;
	float y_val;

	// memory used to hold intermediate values for neural network
	AI_ALIGNED(32) ai_u8 activations[AI_SINE_MODEL_OWN_DATA_ACTIVATIONS_SIZE];

	// Buffers used to store input and output tensors
	AI_ALIGNED(32) ai_float in_data[AI_SINE_MODEL_OWN_IN_1_SIZE_BYTES];
	AI_ALIGNED(32) ai_float out_data[AI_SINE_MODEL_OWN_OUT_1_SIZE_BYTES];

	// pointer to model
	ai_handle sine_model = AI_HANDLE_NULL;

	// initialize structs that hold pointers to data and infor about the data (tensors height, width, channels)
	ai_buffer ai_input[AI_SINE_MODEL_OWN_IN_NUM];
	ai_buffer ai_output[AI_SINE_MODEL_OWN_OUT_NUM];

	ai_network_params ai_params = {
			AI_SINE_MODEL_OWN_DATA_WEIGHTS(ai_sine_model_own_data_weights_get()),
			AI_SINE_MODEL_OWN_DATA_ACTIVATIONS(activations)
	};

	 const ai_buffer* in_desc  = ai_sine_model_own_inputs_get(sine_model, NULL);
	 const ai_buffer* out_desc = ai_sine_model_own_outputs_get(sine_model, NULL);

	 ai_input[0]  = in_desc[0];
	 ai_output[0] = out_desc[0];

	// set pointers to our data buffers
	//ai_input[0].shape.dims[0] = 1;
	ai_input[0].data = AI_HANDLE_PTR(in_data);
	//ai_output[0].shape.dims[0] = 1;
	ai_output[0].data = AI_HANDLE_PTR(out_data);


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  Graph_Init();

  float x = 0.0f;
  const float step = 2.0f * (float)M_PI / 64.0f;


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM11_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim11);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // PA6
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // PA8
  uint32_t arr2 = __HAL_TIM_GET_AUTORELOAD(&htim1);
  uint32_t cnt = __HAL_TIM_GET_COUNTER(&htim1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (cnt + 1000) % (arr2 + 1)); // pierwszy „tik”
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
  NVIC_SetPriority(TIM1_CC_IRQn, 0);
  NVIC_EnableIRQ(TIM1_CC_IRQn);
  HandleOutput();


  SSD1306_Init();
  SSD1306_Clear();
  int temp = 1;

  sprintf(buff,"AI_SINUS() ");
  SSD1306_GotoXY (0, 0);
  SSD1306_Puts (buff, &Font_11x18, 1);
  sprintf(buff," WELCOME!");
  SSD1306_GotoXY (0, 40);
  SSD1306_Puts (buff, &Font_11x18, 1);
  SSD1306_UpdateScreen();

  HAL_Delay(2000);
  SSD1306_Clear();


  // create instance of neural network
  ai_err = ai_sine_model_own_create(&sine_model, AI_SINE_MODEL_OWN_DATA_CONFIG);
  if(ai_err.type != AI_ERROR_CODE_NONE)
  {
	  sprintf(buff,"%dError: could not create a NN instance");
	  	SSD1306_Clear();
	  	SSD1306_GotoXY (0, 0);
	    SSD1306_Puts (buff, &Font_11x18, 1);
		SSD1306_UpdateScreen();
		while(true);
  }

  if(!ai_sine_model_own_init(sine_model, &ai_params))
  {
	  sprintf(buff,"%dError: could not initialize NN");
	  	SSD1306_Clear();
	    SSD1306_GotoXY (0, 0);
	    SSD1306_Puts (buff, &Font_11x18, 1);
		SSD1306_UpdateScreen();
		while(true);
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ((ai_float*)ai_input[0].data)[0] = x;

	  timestamp = htim11.Instance->CNT;

	  nbatch = ai_sine_model_own_run(sine_model, &ai_input[0],&ai_output[0]);
	  if(nbatch != 1)
	  {
		  sprintf(buff,"Error: could not inference NN");
			SSD1306_Clear();
			SSD1306_GotoXY (0, 0);
			SSD1306_Puts (buff, &Font_11x18, 1);
			SSD1306_UpdateScreen();
			while(true);
	  }

	  y_val = ((float*)out_data)[0];
	  uint32_t dur = htim11.Instance->CNT - timestamp;

	  uint16_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_from_y(y_val, arr));

	  Stepper_SetFromAI(y_val); // stepmotor
	  Graph_PushAndDraw(y_val, -1.2f, 1.2f, "AI_SIN", dur);

	  x += step;
	  if (x > 2.0f * (float)M_PI) x -= 2.0f * (float)M_PI;

	  HAL_Delay(20); // ~50 Hz

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 49;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 33601;
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
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 83;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 0xffff-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */


  /* USER CODE END TIM11_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STP_IN1_Pin|STP_IN2_Pin|STP_IN3_Pin|STP_IN4_Pin
                          |Led_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STP_IN1_Pin STP_IN2_Pin STP_IN3_Pin STP_IN4_Pin
                           Led_GREEN_Pin */
  GPIO_InitStruct.Pin = STP_IN1_Pin|STP_IN2_Pin|STP_IN3_Pin|STP_IN4_Pin
                          |Led_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  //GPIO_InitTypeDef GPIO_InitStruct = {0};
  //__HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = STP_IN1_Pin; HAL_GPIO_Init(STP_IN1_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = STP_IN2_Pin; HAL_GPIO_Init(STP_IN2_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = STP_IN3_Pin; HAL_GPIO_Init(STP_IN3_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = STP_IN4_Pin; HAL_GPIO_Init(STP_IN4_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(STP_IN1_GPIO_Port, STP_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(STP_IN2_GPIO_Port, STP_IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(STP_IN3_GPIO_Port, STP_IN3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(STP_IN4_GPIO_Port, STP_IN4_Pin, GPIO_PIN_RESET);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
