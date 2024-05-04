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

extern UART_HandleTypeDef huart5;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
#define EN_USART2_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
#define EN_USART3_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����

/* USER CODE END Private defines */

void MX_UART5_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
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

extern uint8_t USART2_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern uint16_t USART2_RX_STA;         		   //����״̬���	

extern uint8_t USART3_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern uint16_t USART3_RX_STA;         		   //����״̬���	

#define RXBUFFERSIZE   1    								//�����С
extern uint8_t aRxBuffer2[RXBUFFERSIZE];    //HAL��USART����Buffer
extern uint8_t aRxBuffer3[RXBUFFERSIZE];    //HAL��USART����Buffer


int fputc(int ch, FILE *f);
int fgetc(FILE *f);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

