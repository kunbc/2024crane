#include "bsp_usart.h"

 /**
  * @brief  ����Ƕ�������жϿ�����NVIC
  * @param  ��
  * @retval ��
  */
void NVIC_Usart1_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;//�����ж����ýṹ�����
  
  /* Ƕ�������жϿ�������ѡ�� */
	/* ��ʾ NVIC_PriorityGroupConfig() ����������ֻ��Ҫ����һ�����������ȼ�����*/
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  /* ����USARTΪ�ж�Դ */
  NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART1_IRQ;
  /* �������ȼ�*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  /* �����ȼ� */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  /* ʹ���ж� */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* ��ʼ������NVIC */
  NVIC_Init(&NVIC_InitStructure);
}

/*******************************************����1�ӷ�*******************************************/
 /**
  * @brief  USART GPIO ����,������������
  * @param  ��
  * @retval ��
  */
void USART1_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;//���ڶ˿����ýṹ�����
	USART_InitTypeDef USART1_InitStructure;//���ڲ������ýṹ�����

	// �򿪴���GPIO��ʱ��
	DEBUG_USART1_GPIO_APBxClkCmd(DEBUG_USART1_GPIO_CLK, ENABLE);
	
	// �򿪴��������ʱ��
	DEBUG_USART1_APBxClkCmd(DEBUG_USART1_CLK, ENABLE);

	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART1_TX_GPIO_PIN;//GPIO����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//�趨IO�ڵ�����ٶ�Ϊ50MHz
	GPIO_Init(DEBUG_USART1_TX_GPIO_PORT, &GPIO_InitStructure);//��ʼ��PA2

  // ��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART1_RX_GPIO_PIN;//GPIO����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(DEBUG_USART1_RX_GPIO_PORT, &GPIO_InitStructure);//��ʼ��PA3
	
	// ���ô��ڵĹ�������
	// ���ò�����
	USART1_InitStructure.USART_BaudRate = DEBUG_USART1_BAUDRATE;
	// ���� �������ֳ�
	USART1_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	// ����ֹͣλ
	USART1_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	// ����У��λ
	USART1_InitStructure.USART_Parity = USART_Parity_No ;//����żУ��λ
	// ����Ӳ��������
	USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	// ���ù���ģʽ���շ�һ��
	USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// ��ɴ��ڵĳ�ʼ������
	USART_Init(DEBUG_USART1, &USART1_InitStructure);
	
	// �����ж����ȼ�����
	NVIC_Usart1_Configuration();
	
	// ʹ�ܴ��ڽ����ж�
	USART_ITConfig(DEBUG_USART1, USART_IT_RXNE, ENABLE);	
	
	// ʹ�ܴ���
	USART_Cmd(DEBUG_USART1, ENABLE);

	//�����������1���ֽ��޷���ȷ���ͳ�ȥ������
	USART_ClearFlag(DEBUG_USART1, USART_FLAG_TC); //�崮�ڷ��ͱ�־
}
///�ض���c�⺯��printf�����ڣ��ض�����ʹ��printf����
int fputc(int ch, FILE *f)
{
		/* ����һ���ֽ����ݵ����� */
		USART_SendData(DEBUG_USART1, (uint8_t) ch);
		
		/* �ȴ�������� */
		while (USART_GetFlagStatus(DEBUG_USART1, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}

///�ض���c�⺯��scanf�����ڣ���д����ʹ��scanf��getchar�Ⱥ���
int fgetc(FILE *f)
{
		/* �ȴ������������� */
		while (USART_GetFlagStatus(DEBUG_USART1, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(DEBUG_USART1);
}


/*****************	����3�ӷ�	*************************/
 /**
  * @brief  ����Ƕ�������жϿ�����NVIC
  * @param  ��
  * @retval ��
  */
void NVIC_Usart3_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Ƕ�������жϿ�������ѡ�� */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  /* ����USARTΪ�ж�Դ */
  NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART3_IRQ;
  /* �������ȼ�*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  /* �����ȼ� */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  /* ʹ���ж� */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* ��ʼ������NVIC */
  NVIC_Init(&NVIC_InitStructure);
}

 /**
  * @brief  USART GPIO ����,������������
  * @param  ��
  * @retval ��
  */
void USART3_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART3_InitStructure;

	// �򿪴���GPIO��ʱ��
	DEBUG_USART3_GPIO_APBxClkCmd(DEBUG_USART3_GPIO_CLK, ENABLE);
	
	// �򿪴��������ʱ��
	DEBUG_USART3_APBxClkCmd(DEBUG_USART3_CLK, ENABLE);

	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART3_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART3_TX_GPIO_PORT, &GPIO_InitStructure);

  // ��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART3_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DEBUG_USART3_RX_GPIO_PORT, &GPIO_InitStructure);
	
	// ���ô��ڵĹ�������
	// ���ò�����
	USART3_InitStructure.USART_BaudRate = DEBUG_USART3_BAUDRATE;
	// ���� �������ֳ�
	USART3_InitStructure.USART_WordLength = USART_WordLength_8b;
	// ����ֹͣλ
	USART3_InitStructure.USART_StopBits = USART_StopBits_1;
	// ����У��λ
	USART3_InitStructure.USART_Parity = USART_Parity_No ;
	// ����Ӳ��������
	USART3_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// ���ù���ģʽ���շ�һ��
	USART3_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// ��ɴ��ڵĳ�ʼ������
	USART_Init(DEBUG_USART3, &USART3_InitStructure);
	
	// �����ж����ȼ�����
	NVIC_Usart3_Configuration();
	
	// ʹ�ܴ��ڽ����ж�
	USART_ITConfig(DEBUG_USART3, USART_IT_RXNE, ENABLE);	
	
	// ʹ�ܴ���
	USART_Cmd(DEBUG_USART3, ENABLE);		

  // ���������ɱ�־
	USART_ClearFlag(DEBUG_USART3, USART_FLAG_TC);     
}



/*****************  ����һ���ֽ� **********************/
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
	/* ����һ���ֽ����ݵ�USART */
	USART_SendData(pUSARTx,ch);
		
	/* �ȴ��������ݼĴ���Ϊ�� */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/****************** ����8λ������ ************************/
void Usart_SendArray( USART_TypeDef * pUSARTx, uint8_t *array, uint16_t num)
{
  uint8_t i;
	
	for(i=0; i<num; i++)
  {
	    /* ����һ���ֽ����ݵ�USART */
	    Usart_SendByte(pUSARTx,array[i]);	
  
  }
	/* �ȴ�������� */
	while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET);
}

/*****************  �����ַ��� **********************/
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
  do 
  {
      Usart_SendByte( pUSARTx, *(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  /* �ȴ�������� */
  while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)
  {}
}

/*****************  ����һ��16λ�� **********************/
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* ȡ���߰�λ */
	temp_h = (ch&0XFF00)>>8;
	/* ȡ���Ͱ�λ */
	temp_l = ch&0XFF;
	
	/* ���͸߰�λ */
	USART_SendData(pUSARTx,temp_h);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
	
	/* ���͵Ͱ�λ */
	USART_SendData(pUSARTx,temp_l);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}





///********F4���ݽ��պ���********/
//extern u8 name,location,startflag;
//;//����openmv���͵�����

////����F4������������
//void F4_Receive_Data(uint8_t *com_data)
//{ 
//		static uint8_t RxBuffer1[80]={0};//����F4���͵�����
//		for(int j=0; j < 2; j++)
//		{
//				RxBuffer1[j] = com_data[j];
//		}
//		USART_ITConfig(DEBUG_USART3, USART_IT_RXNE,DISABLE);//�ر�DTSABLE�ж�
//		USART_ITConfig(DEBUG_USART1, USART_IT_RXNE,DISABLE);//�ر�DTSABLE�ж�
//		
//		location=RxBuffer1[0];
//		name=RxBuffer1[1];
//		printf ("location:%d name:%d \r\n",location,name);
//		if(location==57)
//		{
//			startflag=1;
//			printf("startflag=%d ��ʼ��ʼ��\r\n",startflag);
//		}
//		USART_ITConfig(DEBUG_USART3, USART_IT_RXNE,ENABLE);
//		USART_ITConfig(DEBUG_USART1, USART_IT_RXNE,ENABLE);
//}	

