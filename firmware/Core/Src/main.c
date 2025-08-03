#include "main.h"
#include <stdio.h>
#include <string.h>

#define TRIGGER_PULSE_LEN 10 //trigger for 10 us
#define ECHO_TIMEOUT_MS 30 //if no echo received in 30ms, no objct detected
#define SENSOR_INTERVAL_MS 60 //60ms interval between sensor triggers
#define TRIGGER_PIN GPIO_PIN_10
#define ECHO_PIN GPIO_PIN_3
#define LED_PIN GPIO_PIN_5
#define BUZZER_PIN  GPIO_PIN_4
#define TRIG_PORT       GPIOA
#define ECHO_PORT       GPIOB

#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define SERVO_STEP 1                  // degrees per update
#define SERVO_UPDATE_INTERVAL 30     // ms between steps

#define ALARM_ITER 6
#define ALARM_LEN 150 //length of flash/alarm

static uint16_t servo_angle = 0;
static uint8_t servo_direction = 1;  // 1 = forward, 0 = backward
static uint32_t last_servo_update = 0;

/* IC measurement state: */
volatile uint32_t ic_start = 0;
volatile uint32_t ic_stop  = 0;
volatile uint8_t  ic_state = 0;      // 0 = next edge = rising, 1 = next = falling
volatile float    last_distance = 0;
volatile uint8_t  dist_ready = 0;    // Flag: distance just computed


uint16_t object_found = 0;
int echo_time = -1; //-1 if no object detected, else length of echo pulse
int trig_start; // moment when trigger set
float distance = -1; //distance in cm
int buzzer_last = 0;
uint8_t buzzer = 0;

TIM_HandleTypeDef htim2; //Sensor timer
TIM_HandleTypeDef htim3; //PWM timer

UART_HandleTypeDef huart2;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
void TriggerSensor(void);
void servo_loop(void);
void alarm_loop();
void BuzzerOn();
void print_distance_and_angle();

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        if (ic_state == 0)
        {
            /* Rising edge detected -> record start time */
            ic_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

            /* Reconfigure capture to catch falling edge next */
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
            ic_state = 1;
        }
        else if (ic_state == 1)
        {
            /* Falling edge detected -> record stop time */
            ic_stop = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

            /* Compute pulse duration (handle wrap if necessary) */
            uint32_t duration;
            if (ic_stop >= ic_start) {
                duration = ic_stop - ic_start;
            } else {
                duration = (0xFFFF - ic_start) + ic_stop + 1;
            }

            /* Convert to distance in centimeters: duration(µs) / 58 */
            if (duration >= 38000U) {
                /* out of range: mark negative distance */
                last_distance = -1.0f;
            } else {
                last_distance = (float)duration / 58.0f;
            }
            dist_ready = 1;

            /* Re-arm for next rising edge */
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
            ic_state = 0;
        }
    }
}

int main(void)
{
//  char buf[64];
  uint32_t last_trigger_ms = 0;

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  /* Main loop:
       – Every 200 ms, fire a 10 µs TRIG pulse (non-blocking except ~12 µs busy-wait).
       – Whenever dist_ready == 1, print the new distance.
       – Otherwise, do other work here…
  */
  while (1)
  {
      uint32_t now = HAL_GetTick();


      if ((now - last_trigger_ms) >= 200)
      {
          last_trigger_ms = now;
          TriggerSensor();
      }
      if (dist_ready)
      {
          dist_ready = 0;
          print_distance_and_angle();
      }
      if (buzzer)
      		BuzzerOn();

      servo_loop();
      alarm_loop();
  }
}

void TriggerSensor(){
    /* Generate a 10 µs high pulse on TRIG_PIN using TIM2 counter */
    HAL_GPIO_WritePin(TRIG_PORT, TRIGGER_PIN, GPIO_PIN_RESET);
    /* Short low to ensure clean rising edge (~2 µs) */
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while (__HAL_TIM_GET_COUNTER(&htim2) < 2);; /* ~2 µs */

    HAL_GPIO_WritePin(TRIG_PORT, TRIGGER_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while (__HAL_TIM_GET_COUNTER(&htim2) < 10);; /* ~10 µs */
    HAL_GPIO_WritePin(TRIG_PORT, TRIGGER_PIN, GPIO_PIN_RESET);

    /* Now TIM2 IC interrupt will catch rising/falling on ECHO (PB3) */
}

void print_distance_and_angle(){
    float d_cm = last_distance;
    char data[64];

    if (d_cm < 0.0f || d_cm > 20.0f)   // overflow or out-of-range
    {
        snprintf(data, sizeof(data), "\n%d,%0.1f\r\n", servo_angle, -1.0f);
        object_found = 0;
    }
    else
    {
    	object_found = 1;
		snprintf(data, sizeof(data), "\n%d,%0.1f\r\n", servo_angle, d_cm);
    }

	HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data) - 1, HAL_MAX_DELAY);
}

void servo_loop(){
	 //wait	20 ms
	 if (HAL_GetTick() - last_servo_update >= SERVO_UPDATE_INTERVAL) {
		last_servo_update = HAL_GetTick();

		// Update angle
		if (servo_direction)
			servo_angle += SERVO_STEP;
		else
			servo_angle -= SERVO_STEP;

		// Change direction at limits
		if (servo_angle >= SERVO_MAX_ANGLE) {
			servo_angle = SERVO_MAX_ANGLE;
			servo_direction = 0; // go backwards
		} else if (servo_angle <= SERVO_MIN_ANGLE) {
			servo_angle = SERVO_MIN_ANGLE;
			servo_direction = 1; //go forwards
		}

		// Convert angle to PWM pulse width
		int pulse_width = 500 + ((servo_angle * 2000) / SERVO_MAX_ANGLE);
//		char angle_str [50];
//		sprintf(angle_str, "current angle: %d, pulse len: %d\n\r", servo_angle, pulse_width);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_width); //update CCR
	}
}

void alarm_loop(){
	static uint8_t cnt = 0;
	static uint8_t blinking = 0;
	static uint32_t last_toggle = 0;
	if (object_found) {
		blinking = 1;
		buzzer = 1;
		object_found = 0;
		last_toggle = HAL_GetTick();
	}
	if (blinking) {
		if (HAL_GetTick() - last_toggle >= ALARM_LEN) {
			if (cnt % 2 == 0)
				HAL_GPIO_WritePin(GPIOB, LED_PIN, GPIO_PIN_SET); //turn on LED
			else
				HAL_GPIO_WritePin(GPIOB, LED_PIN, GPIO_PIN_RESET); //turn off LED
			last_toggle = HAL_GetTick();
			cnt++;
			if (cnt >= 6) {
				cnt = 0;
				blinking = 0;
				buzzer = 0;
			}
		}
	}
}

void BuzzerOn(){
	if (TIM3 -> CNT - buzzer_last >= 5){
		HAL_GPIO_TogglePin(GPIOB, BUZZER_PIN);
		buzzer_last = TIM3 -> CNT;
	}
}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

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
