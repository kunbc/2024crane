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
/*	【引脚定义】

[步进电机]  PUL    DIR       
motor1：    PC6    PG11
motor1：    PC7    PG12
motor1：    PC8    PG13
motor1：    PC9    PG14

[舵机]	    PWM
servo1：    PA6
servo1：    PA7

[TOF]       TX     RX
TOF1：      PA2    PA3
TOF2：      PB10   PB11

[通信]      TX     RX
上位机：    PA9    PA10
单片机2：   PC12   PD2
*/

/*
通讯解析
三个识别点：1，4，7
发送的字符信号：1，2，3
解析：
第1个位置有，  或者第4个位置、第7个位置如果全有砝码， 发送1 -> 规划路径
第1个位置没有, 或者第4个位置、第7个位置如果非全有, 		发送2 -> 规划路径
抓完物品、放完物品后或者直接走，大车行进，						发送3，-> 按规划的路径直接行进

接收的字符信号：a，1，2，3，4，5，6，7，8，9
解析：初始化完后发送 “a” ，标志开始。
其余数字：位置信息。
*/

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "TOFSense.h"
#include "stepper_motor.h"
#include "malloc.h"
#include "servo.h"
#include "logic.h"
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
/* USER CODE BEGIN PV */

/***********步进电机步数数组**************/
/*步进电机运动方向
	起升motor1------------ +:升 -:降
	起升motor2------------ +:升 -:降
	平移motor3------------ +:边 -:中
	平移motor4------------ +:边 -:中
*/

uint8_t cs_flag=0;//测试参数
double crane_init[3]={1.0,1.0,-3.27},/*motor1升  motor2升  motor3中*/
//初始化步数数组

sec1_bj[3]={-0.85,1.02,-0.21},/*motor1降  motor1升  motor1降  motor1升*/
//第一部分步进电机1、2起升升步数记录

sce2_zhubei[2]={0.04,3.27},/*motor1降  motor3边*/
//第二部分步进电机1，3运动步数记录

sec2o3_qs[2][3]={
{-0.85,1.87,-0.21},/*motor1降  motor1升  motor1降  motor1升*/
{-0.85,1.87,-0.21} /*motor2降  motor2升  motor2降  motor2升*/
},

sec2o3_py[2][3]={
{3.195,0.985,4.152},//motor3:内平移（边）外平移（边）大平移（边）
{3.160,1.090,4.249} //motor4:内平移（边）外平移（边）大平移（边）
},

sce3_zhubei[3]={-0.81,-4.152,-4.249};/*motor1/motor2降  3大平移/4大平移（中）*/

/****标志位所用参数***/
uint8_t action=0;//初始化移动标志位

/****TOF测距使用参数****/
unsigned char TOF_data[32] = {0};   //store 2 TOF frames
unsigned char TOF_data2[32] = {0};
unsigned char TOF_length = 16;
unsigned char TOF_header[3] = {0x57,0x00,0xFF};
unsigned long TOF_system_time = 0;
unsigned long TOF_distance1 = 0;
unsigned long TOF_distance2 = 0;
unsigned long dis1 = 0;
unsigned long dis2 = 0;
unsigned char TOF_status = 0;
unsigned int TOF_signal = 0;
unsigned char TOF_check = 0;
uint8_t count = 0;
uint8_t location = 0; //初始参数为0，表示0号位
uint8_t servo1_zhuaqv = 0;//1为立即抓，2为在下一个位置抓
uint8_t servo2_zhuaqv = 0;//1为立即抓，2为在下一个位置抓
int qvwu_infer[2][9]={{0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0}};

/****串口接收全局数组****/
uint16_t DataBuff[50]={0};uint16_t DataBuff5[50]={0};
extern uint16_t RxBuff[1],RxBuff5[1];

/****步进电机加减速使用参数和宏*****/
__IO double g_step_angle = 200;            /* 设置的步进步数*/
extern __IO  uint32_t g_add_pulse_count[4];    /* 脉冲个数累计*/
extern motor_state_typedef g_motor1_sta,
g_motor2_sta,
g_motor3_sta,
g_motor4_sta; 

/****内存申请所用参数****/
const char *SRAM_NAME_BUF[SRAMBANK] = {" SRAMIN  ", " SRAMCCM ", " SRAMEX  "};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_CAN2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM10_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_FSMC_Init();
  /* USER CODE BEGIN 2 */
	/*********内存初始化***********/
	my_mem_init(SRAMIN);                /* 初始化内部SRAM内存池 */
  my_mem_init(SRAMEX);                /* 初始化外部SRAM内存池 */
  my_mem_init(SRAMCCM);               /* 初始化内部CCM内存池 */
	
	/*串口中断开启函数*/
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxBuff, 1);
	HAL_UART_Receive_IT(&huart5, (uint8_t *)&RxBuff5, 1);
	HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer2, 1);
	HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer3, 1);
	/***舵机通道初始化****/
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);//启动定时器3通道1的PWM输出
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);//启动定时器3通道2的PWM输出
	steer_init(87);
	/**关闭步进电机通道**/
	HAL_TIM_OC_Stop_IT(&htim8, TIM_CHANNEL_1);
	HAL_TIM_OC_Stop_IT(&htim8, TIM_CHANNEL_2);
	HAL_TIM_OC_Stop_IT(&htim8, TIM_CHANNEL_3);
	HAL_TIM_OC_Stop_IT(&htim8, TIM_CHANNEL_4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
					zhengji();//整机运行函数
    /* USER CODE END WHILE */
		
    /* USER CODE BEGIN 3 */
    /**************电机模块测试*******************/
#if   0
		if(cs_flag==0)
		{
					HAL_Delay(1000);
					steer(ZHANGKAI,SERVO_2);
					HAL_Delay(2000);
					steer(87.0,SERVO_2);
//					HAL_Delay(2000);
//					steer(BIHE,SERVO_ALL);
//					HAL_Delay(2000);
//					steer(ZHANGKAI,SERVO_ALL);
//				g_add_pulse_count[0]=0;
//				stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,(1.0f)*SPR,STEPPER_MOTOR_1);;/* 一次加减速运动 */
//				stepper_start(STEPPER_MOTOR_1);
				cs_flag=1;	
		}
//							sec2o3_zhuaqv2();
#elif 0
		if(cs_flag==0)
		{
				g_add_pulse_count[0]=0;
				g_add_pulse_count[1]=0;
				stepmotor_move_rel(V_START,80,0.01,0.01,(-1.0f)*SPR,STEPPER_MOTOR_2);;/* 一次加减速运动 */
//				stepmotor_move_rel(V_START,100,0.01,0.01,(-1.0f)*SPR,STEPPER_MOTOR_1);;/* 一次加减速运动 */
				stepper_start(STEPPER_MOTOR_2);
//				stepper_start(STEPPER_MOTOR_1);
				cs_flag=1;
		}
#endif 
//		else if(cs_flag==1&&g_motor2_sta==STATE_IDLE)
//		{
//				HAL_Delay(1000);
//				g_add_pulse_count[0]=0;
//				stepmotor_move_rel(0,20,0.05f,0.05f,(-2.0)*SPR,STEPPER_MOTOR_2);/* 一次加减速运动 */
//				stepper_start(STEPPER_MOTOR_2);
//				cs_flag=2;
//		}
//		
		/**************激光测距模块测试***************/
//					ceju();		
/***********************************/
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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



