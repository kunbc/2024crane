/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>	
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
#define EN_USART2_RX 			1		//使能（1）/禁止（0）串口1接收
#define EN_USART3_RX 			1		//使能（1）/禁止（0）串口1接收

extern uint8_t count;
extern unsigned char TOF_data[32];   //store 2 TOF frames
extern unsigned char TOF_data2[32];
extern unsigned char TOF_length;
extern unsigned char TOF_header[3];
extern unsigned long TOF_system_time;
extern unsigned long TOF_distance;
extern unsigned char TOF_status;
extern unsigned int TOF_signal;
extern unsigned char TOF_check;

//extern unsigned long dis1;
//extern unsigned long dis2;
//extern unsigned long dis3;


extern uint8_t USART2_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern uint16_t USART2_RX_STA;         		   //接收状态标记	

extern uint8_t USART3_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern uint16_t USART3_RX_STA;         		   //接收状态标记	

#define RXBUFFERSIZE   1    								//缓存大小
extern uint8_t aRxBuffer2[RXBUFFERSIZE];    //HAL库USART接收Buffer
extern uint8_t aRxBuffer3[RXBUFFERSIZE];    //HAL库USART接收Buffer

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
int fputc(int ch, FILE *f);
int fgetc(FILE *f);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

