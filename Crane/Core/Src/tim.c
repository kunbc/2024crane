/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"

/* USER CODE BEGIN 0 */
#include "bsp_can.h"
#include "pid.h"
#define PERIMETER 0.00290888208665721596153948461415f

#define readl1 HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_1)
#define readl2 HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_2)
#define readl3 HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_3)
#define readl4 HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_4)
#define readl5 HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_5)
#define readl6 HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_6)
#define readl7 HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)
#define readl8 HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_8)

// uint8_t poschange=1;
uint8_t stop=0;//停车标志位
float adjust=0;//调整速度变量
uint8_t Send=0;//发送信息标志位

uint8_t path[10]={1,4,5,8,3,0};
uint8_t newplace=1;
tPid stop_control;
// const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
// const motor_measure_t *data[4];
static motor_control classic_move;
// extern tPid stop_control;
/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
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
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}
/* TIM2 init function */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspPostInit 0 */

  /* USER CODE END TIM2_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA0-WKUP     ------> TIM2_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM2_MspPostInit 1 */

  /* USER CODE END TIM2_MspPostInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /* TIM1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /* TIM2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim==(&htim1))
  {
    position_control();
    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
      classic_move.motor[i].data=get_chassis_motor_measure_point(i);
      classic_move.motor[i].speed=classic_move.motor[i].data->speed_rpm*PERIMETER;
      if(i==0||i==1)
      {
        classic_move.motor[i].tar_speed=-(SPEED+adjust);
        // classic_move.motor[i].tar_speed=-(SPEED+stop_control.out);
        // classic_move.motor[i].tar_speed=-SPEED;
      }
      else
      {
        classic_move.motor[i].tar_speed=(SPEED+adjust);
        // classic_move.motor[i].tar_speed=(SPEED+stop_control.out);
        // classic_move.motor[i].tar_speed=SPEED+0.8f;
      }
      PID_realize(&classic_move.pid[i],classic_move.motor[i].speed,classic_move.motor[i].tar_speed);
      // CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
      classic_move.motor[i].given_current=(int16_t)classic_move.pid[i].out;
      /* code */
    }
    set_moto_current(classic_move.motor[0].given_current, classic_move.motor[1].given_current, classic_move.motor[2].given_current, classic_move.motor[3].given_current);
		// set_moto_current(500, 0, 0, 0);
    // printf("%d\n",classic_move.motor[0].data->speed_rpm);
    // printf("%d\r\n",(int16_t)stop_control.out);
    // printf("%d,%d,%d,%d\r\n",(int16_t)classic_move.motor[0].speed,(int16_t)classic_move.motor[2].speed,(int16_t)classic_move.motor[0].given_current,(int16_t)classic_move.motor[2].given_current);
    // printf("%d,%d\r\n",(int16_t)classic_move.motor[0].speed,(int16_t)classic_move.motor[2].speed);
    // printf("%d\n",classic_move.motor[0].given_current);
  }
}

void Classic_init(void)
{
  pid_init(&classic_move);
  PID_Speed_reset(&stop_control,5.0,0.1,3.0);
}

void position_control(void)
{
  // if(readl1==0||readl8==0)
  // {
  //   poschange=1;
    
  //     // else
  //     // {
  //     //   adjust=0.0f;
  //     // }
  //     // if(adjust!=0)
  //     // {
  //     //   PID_stop_realize(&stop_control,adjust,-SPEED);
  //     // }
  //   }
  // }
  if(readl8==0)
  {
    stop=1;
    newplace+=1;
  }
  if(stop==1)
  {
    if(readl2==0&&readl3==1&&readl4==1&&readl5==1&&readl6==1&&readl7==1)
    {
      adjust=-SPEED*1.5f;
      printf("1");
    }
    else if(readl2==0&&readl3==0&&readl4==1&&readl5==1&&readl6==1&&readl7==1)
    {
      adjust=-SPEED*1.4f;
      printf("2");
    }
    else if(readl2==1&&readl3==0&&readl4==0&&readl5==1&&readl6==1&&readl7==1)
    {
      adjust=-SPEED*1.4f;
      printf("3");
    }
    else if(readl2==0&&readl3==0&&readl4==0&&readl5==1&&readl6==1&&readl7==1)
    {
      adjust=-SPEED*1.4f;
      printf("3");
    }
    else if(readl2==1&&readl3==0&&readl4==0&&readl5==0&&readl6==1&&readl7==1)
    {
      adjust=-SPEED*1.2f;
      printf("4");
    }
    else if(readl2==1&&readl3==1&&readl4==0&&readl5==0&&readl6==1&&readl7==1)
    {
      adjust=-SPEED;
      printf("5");
      Send=1;
    }
    else if(readl2==1&&readl3==1&&readl4==0&&readl5==0&&readl6==0&&readl7==1)
    {
      adjust=-SPEED*0.9f;
      printf("6");
    }
    else if(readl2==1&&readl3==1&&readl4==1&&readl5==0&&readl6==0&&readl7==1)
    {
      adjust=-SPEED*0.8f;
      printf("7");
    }
    else if(readl2==1&&readl3==1&&readl4==1&&readl5==0&&readl6==0&&readl7==0)
    {
      adjust=-SPEED*0.8f;
      printf("7");
    }
    else if(readl2==1&&readl3==1&&readl4==1&&readl5==1&&readl6==0&&readl7==0)
    {
      adjust=-SPEED*0.8f;
      printf("8");
    }
    else if(readl2==1&&readl3==1&&readl4==1&&readl5==1&&readl6==1&&readl7==0)
    {
      adjust=-SPEED*0.7f;
      printf("9");
    }
    // if(adjust!=0)
    // {
    //   PID_stop_realize(&stop_control,adjust,-SPEED);
    // }
  }
  // else
  // {
  //   stop_control.out=0.0f;
  // }
}
/* USER CODE END 1 */
