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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

uint8_t phase = 0;
uint8_t ramp = 1;

// RAMP UP VARIABLES

uint32_t pulse_width;          // PWM pulse width
uint32_t timer_ticks;          // Open Loop Timer controller ticks

uint8_t duty_initial = 22;    // initial duty
uint8_t duty_final = 15;      // final duty
uint8_t period_initial = 4;   // initial electrical phase period in ms
uint8_t period_final = 1;     // final electrical phase period in ms

uint32_t pulse_width_final;
uint32_t timer_ticks_final;
uint32_t pulse_width_delta;
uint32_t timer_ticks_delta;

uint16_t rampup_iterations;
uint16_t current_iteration = 0;

uint8_t rampup_time = 5;      // ramp-up time in seconds, approximating by a linear rampup


// TRANSITION VARIABLES

uint8_t transition = 0;
uint16_t transition_iterations;
uint8_t transition_time = 1;  // transition to closed-loop time

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void open_loop(void);
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



	// x1000 to increment sensibility
  pulse_width = duty_initial * 10 * 1000;	// x10 as fPWM = 1kHz -> pulse_width = duty*1000 with duty in range [0,1]
  pulse_width_final = duty_final * 10 * 1000;
  
  timer_ticks = period_initial * 1000;
  timer_ticks_final = period_final * 1000;

  rampup_iterations = (rampup_time * 1000000) / timer_ticks;

	// x10 to increment sensibility
  timer_ticks *= 10;
  timer_ticks_final *= 10;

  pulse_width_delta = (pulse_width - pulse_width_final) / rampup_iterations;
  timer_ticks_delta = (timer_ticks - timer_ticks_final) / rampup_iterations;

  transition_iterations = 1000*transition_time/period_final;

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // START TIMER
  HAL_TIM_Base_Start_IT(&htim3);


  // INICIAL STATE

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); 	// AL INICIAL STATE OFF
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); 	  // BL INICIAL STATE ON
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); 	// CL INICIAL STATE OFF

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 				      // PWMA ON
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);               // PWMB OFF
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);  				      // PWMC OFF

    ramp = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if(ramp){                 // OPEN LOOP STATE

    }
    else if(transition){      // TRANSITION STATE

    }
    else{                     // CLOSED LOOP STATE

    }
    
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC6
                           PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB11 PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void open_loop(void){
    /*
        FUNCIÓ PEL CONTROL OPEN LOOP

        EN FUNCIÓ DE LA FASE ELÈCTRICA ES MODIFIQUEN LES ENTRADES DELS PONTS H

        EN CADA SALT ENTRE FASES HI HA NOMÉS DOS CANVIS I S'ALTERNEN ENTRE PART ALTA I BAIXA DELS PONTS

            PHASE 1: PWMA -> ON; PWMC -> OFF
            PHASE 2: LC -> ON; LB -> OFF
            PHASE 3: PWMB -> ON; PWMA -> OFF
            PHASE 4: LC -> OFF; LA -> ON
            PHASE 5: PWMC -> ON; PWMB -> OFF
            PHASE 6: LA -> OFF; LB -> ON

        _______________________________________
            HA: --------________________
            LA: ____________--------____

            HB: ________--------________
            LB: ----________________----

            HC: ________________--------
            LC: ____--------____________
        _______________________________________

    */

    if(phase > 6)   phase = 1; // ENSURE PERIODICITY

    //LÒGICA PER L'EXCITACIÓ DEL MOTOR

    if(phase == 1){         // PWMA -> ON; PWMC -> OFF
    	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 				// PWMA ON
    	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);  				// PWMC OFF
    }
    else if(phase == 2){    // BL -> OFF; CL -> ON
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);   	// CL ON
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); 	// BL OFF
    }
    else if(phase == 3){    // PWMA -> OFF; PWMB -> ON
    	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  				// PWMB ON
    	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); 				// PWMA OFF
    }
    else if(phase == 4){    // AL -> ON; CL -> OFF
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); 	// AL ON
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);	// CL OFF
    }
    else if(phase == 5){    // PWMB -> OFF; PWMC -> ON
    	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  				// PWMC ON
    	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2); 				// PWMB OFF
    }
    else{   //phase = 6 ||     AL -> OFF; BL -> ON
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);		// BL ON
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);	// AL OFF
    }
}


void closed_loop(void){
/*

        FUNCIÓ PEL CONTROL CLOSED LOOP

        S'APLICA LA MATEIXA LÒGICA QUE EL CONTROL OPEN LOOP, AMB LA DIFERÈNCIA
        QUE EL SALT ENTRE FASES VE DONAT PER LA DETECCIÓ DEL PAS PER ZERO DEL BEMF

        QUAN UNA DE LES FASES ESTA FLOTANT TÉ LLOC EL PAS PER ZERO, CAL
        ACTIVAR LA INTERRUPCIÓ RESPECTIVA PER A DETECTAR-LA

            PHASE 1: DETECT C FALLING - (PWMA -> ON; PWMC -> OFF)
            PHASE 2: DETECT B RISING - (LC -> ON; LB -> OFF)
            PHASE 3: DETECT A FALLING - (PWMB -> ON; PWMA -> OFF)
            PHASE 4: DETECT C RISING - (LC -> OFF; LA -> ON)
            PHASE 5: DETECT B FALLING - (PWMC -> ON; PWMB -> OFF)
            PHASE 6: DETECT A RISING - (LA -> OFF; LB -> ON)

        _______________________________________
            HA: --------_|____________|_
            LA: __________|_--------_|__

            HB: ______|_--------_|______
            LB: ----_|____________|_----

            HC: _|____________|_--------
            LC: __|_--------_|__________
        _______________________________________

*/

  // INITIAL RESET

  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);  				// PWMA OFF
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);  				// PWMA OFF
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);  				// PWMA OFF

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_RESET);  // LA, LB, LC OFF

  if(phase > 6)   phase = 1; // ENSURE PERIODICITY

  if(phase == 1){         // READ PHASE C FALLING
      // PZC -> FALLING

      EXTI->IMR1 |= GPIO_PIN_13;                                      // ENABLE INT C

      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);                       // HA
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);              // LB
  }

  else if(phase == 2){    // READ PHASE B RISING
      // PZB -> RISING

      EXTI->IMR1 |= GPIO_PIN_12;                                      // ENABLE INT B

      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);                       // HA
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);              // LC
  }

  else if(phase == 3){    // READ PHASE A FALLING
      // PZA -> FALLING

      EXTI->IMR1 |= GPIO_PIN_11;                                      // ENABLE INT A

      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);                       // HB
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);              // LC

  }

  else if(phase == 4){    // READ PHASE C RISING
      // PZC -> RISING

      EXTI->IMR1 |= GPIO_PIN_13;                                      // ENABLE INT C

      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);                       // HB        
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);              // LA


  }

  else if(phase == 5){    // READ PHASE B FALLING
      // PE4 -> FALLING

      EXTI->IMR1 |= GPIO_PIN_12;                                      // ENABLE INT B

      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);                       // HC
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);              // LA

  }
  else{                   // READ PHASE A RISING
      // PB1 -> RISING

      EXTI->IMR1 |= GPIO_PIN_11;                                      // ENABLE INT A

      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);                       // HC
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);              // LB

  }

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if (htim->Instance == TIM3){	//RAMPA

    if(ramp){   // RAMPUP

      pulse_width -= pulse_width_delta;
      timer_ticks -= timer_ticks_delta;

      __HAL_TIM_SET_AUTORELOAD(&htim3, timer_ticks/10);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_width/1000);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse_width/1000);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_width/1000);

      current_iteration++;

      if(current_iteration >= rampup_iterations){
        ramp = 0;
        transition = 1;
        current_iteration = 0;
      }

    }

    else if(transition){  // TRANSITIONING TO CLOSED LOOP

      current_iteration++;
      if(current_iteration >= transition_iterations){
    	  transition = 0;
      }

    }

    else{   // ENTERING CLOSED LOOP
    	  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    	  HAL_NVIC_DisableIRQ(TIM3_IRQn);

    }

	phase = (phase % 6) + 1;	// LOOP INCREMENT
	open_loop();

    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_11){

      EXTI->IMR1 &= ~GPIO_PIN_11; // DISABLE A INTERRUPTS

      if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) == GPIO_PIN_SET)  phase = 1;  // A RISING
      else  phase = 4; // A FALLING

    }
    else if(GPIO_Pin == GPIO_PIN_12){

      EXTI->IMR1 &= ~GPIO_PIN_12; // DISABLE B INTERRUPTS

      if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_SET)  phase = 3;  // B RISING
      else  phase = 6; // B FALLING

    }
    else if(GPIO_Pin == GPIO_PIN_13){
      EXTI->IMR1 &= ~GPIO_PIN_13; // DISABLE C INTERRUPTS

      if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET)  phase = 5;  // C RISING
      else  phase = 2; // C FALLING
    }

    closed_loop();
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
