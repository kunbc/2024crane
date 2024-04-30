/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "bsp_can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// #define readl1 HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_1)
// #define readl2 HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_2)
// #define readl3 HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_3)
// #define readl4 HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_4)
// #define readl5 HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_5)
// #define readl6 HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_6)
// #define readl7 HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)
// #define readl8 HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_8)

// #define readl1 HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_1)
// #define readl2 HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2)
// #define readl3 HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_3)
// #define readl4 HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_4)
// #define readl5 HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_5)
// #define readl6 HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6)
// #define readl7 HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_7)
// #define readl8 HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_9)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */




/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
extern uint8_t Send;
extern uint8_t path[10];
extern uint8_t newplace;
extern uint8_t stop;
extern float adjust;//调整速度变量
/* USER CODE BEGIN PV */
// uint8_t poschange=1;
// uint8_t stop=1;
// static int8_t adjust=0;
// tPid stop_control;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// void position_control(void);
void Position_RTx(void);
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
  Classic_init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN2_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer2, 1);
	

  // HAL_Delay (1500);
	can_filter_init(&hcan2);
  HAL_TIM_Base_Start_IT(&htim1);
//	for(int i=0; i<4; i++)
//  {	
//    pid_init(&motor_pid[i]);
//    motor_pid[i].f_param_init(&motor_pid[i],16384,5000,10,0,5000,0,1.0,0.0005,0);    
//  }
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		HAL_Delay(50);
		// set_moto_current(500, 500, 500, 500);
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Position_RTx();
   
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// void position_control(void)
// {
//   if(readl1==0||readl8==0)
//   {
//     poschange=1;
//     if(stop==1)
//     {
//       if(readl2==0&&readl3==1&&readl4==1&&readl5==1&&readl6==1&&readl7==1)
//       {
//         adjust=-SPEED*0.5f;
//       }
//       else if(readl2==1&&readl3==0&&readl4==1&&readl5==1&&readl6==1&&readl7==1)
//       {
//         adjust=-SPEED*0.7f;
//       }
//       else if(readl2==1&&readl3==1&&readl4==0&&readl5==1&&readl6==1&&readl7==1)
//       {
//         adjust=-SPEED*0.9f;
//       }
//       else if(readl2==1&&readl3==1&&readl4==1&&readl5==0&&readl6==1&&readl7==1)
//       {
//         adjust=-SPEED*1.1f;
//       }
//       else if(readl2==1&&readl3==1&&readl4==1&&readl5==1&&readl6==0&&readl7==1)
//       {
//         adjust=-SPEED*1.3f;
//       }
//       else if(readl2==1&&readl3==1&&readl4==1&&readl5==1&&readl6==1&&readl7==0)
//       {
//         adjust=-SPEED*1.5f;
//       }
//       else if(readl2==1&&readl3==1&&readl4==0&&readl5==0&&readl6==1&&readl7==1)
//       {
//         adjust=-SPEED;
//       }
//       // else
//       // {
//       //   adjust=0.0f;
//       // }
//       if(adjust!=0)
//       {
//         PID_stop_realize(&stop_control,adjust,-SPEED);
//       }
//     }
//   }
//   else
//   {
//     stop_control.out=0.0f;
//   }
// }
void Position_RTx(void)
{
  // if(Send==1)
  // {
  //   HAL_UART_Transmit(&huart2,&path[newplace],sizeof(&path[newplace]),10000);
  //   Send=0;
  // }
  if(stop==1)
  {
    HAL_Delay(20000);
    stop=0;
    adjust=0.0f;
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
