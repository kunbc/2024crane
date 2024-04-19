#include "bsp_usart.h"

 /**
  * @brief  配置嵌套向量中断控制器NVIC
  * @param  无
  * @retval 无
  */
void NVIC_Usart1_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;//串口中断配置结构体变量
  
  /* 嵌套向量中断控制器组选择 */
	/* 提示 NVIC_PriorityGroupConfig() 在整个工程只需要调用一次来配置优先级分组*/
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  /* 配置USART为中断源 */
  NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART1_IRQ;
  /* 抢断优先级*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  /* 子优先级 */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  /* 使能中断 */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* 初始化配置NVIC */
  NVIC_Init(&NVIC_InitStructure);
}

/*******************************************串口1接发*******************************************/
 /**
  * @brief  USART GPIO 配置,工作参数配置
  * @param  无
  * @retval 无
  */
void USART1_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;//串口端口配置结构体变量
	USART_InitTypeDef USART1_InitStructure;//串口参数配置结构体变量

	// 打开串口GPIO的时钟
	DEBUG_USART1_GPIO_APBxClkCmd(DEBUG_USART1_GPIO_CLK, ENABLE);
	
	// 打开串口外设的时钟
	DEBUG_USART1_APBxClkCmd(DEBUG_USART1_CLK, ENABLE);

	// 将USART Tx的GPIO配置为推挽复用模式
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART1_TX_GPIO_PIN;//GPIO引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//设定IO口的输出速度为50MHz
	GPIO_Init(DEBUG_USART1_TX_GPIO_PORT, &GPIO_InitStructure);//初始化PA2

  // 将USART Rx的GPIO配置为浮空输入模式
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART1_RX_GPIO_PIN;//GPIO引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(DEBUG_USART1_RX_GPIO_PORT, &GPIO_InitStructure);//初始化PA3
	
	// 配置串口的工作参数
	// 配置波特率
	USART1_InitStructure.USART_BaudRate = DEBUG_USART1_BAUDRATE;
	// 配置 针数据字长
	USART1_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	// 配置停止位
	USART1_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	// 配置校验位
	USART1_InitStructure.USART_Parity = USART_Parity_No ;//无奇偶校验位
	// 配置硬件流控制
	USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	// 配置工作模式，收发一起
	USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// 完成串口的初始化配置
	USART_Init(DEBUG_USART1, &USART1_InitStructure);
	
	// 串口中断优先级配置
	NVIC_Usart1_Configuration();
	
	// 使能串口接收中断
	USART_ITConfig(DEBUG_USART1, USART_IT_RXNE, ENABLE);	
	
	// 使能串口
	USART_Cmd(DEBUG_USART1, ENABLE);

	//如下语句解决第1个字节无法正确发送出去的问题
	USART_ClearFlag(DEBUG_USART1, USART_FLAG_TC); //清串口发送标志
}
///重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到串口 */
		USART_SendData(DEBUG_USART1, (uint8_t) ch);
		
		/* 等待发送完毕 */
		while (USART_GetFlagStatus(DEBUG_USART1, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}

///重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
		/* 等待串口输入数据 */
		while (USART_GetFlagStatus(DEBUG_USART1, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(DEBUG_USART1);
}


/*****************	串口3接发	*************************/
 /**
  * @brief  配置嵌套向量中断控制器NVIC
  * @param  无
  * @retval 无
  */
void NVIC_Usart3_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* 嵌套向量中断控制器组选择 */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  /* 配置USART为中断源 */
  NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART3_IRQ;
  /* 抢断优先级*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  /* 子优先级 */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  /* 使能中断 */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* 初始化配置NVIC */
  NVIC_Init(&NVIC_InitStructure);
}

 /**
  * @brief  USART GPIO 配置,工作参数配置
  * @param  无
  * @retval 无
  */
void USART3_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART3_InitStructure;

	// 打开串口GPIO的时钟
	DEBUG_USART3_GPIO_APBxClkCmd(DEBUG_USART3_GPIO_CLK, ENABLE);
	
	// 打开串口外设的时钟
	DEBUG_USART3_APBxClkCmd(DEBUG_USART3_CLK, ENABLE);

	// 将USART Tx的GPIO配置为推挽复用模式
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART3_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART3_TX_GPIO_PORT, &GPIO_InitStructure);

  // 将USART Rx的GPIO配置为浮空输入模式
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART3_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DEBUG_USART3_RX_GPIO_PORT, &GPIO_InitStructure);
	
	// 配置串口的工作参数
	// 配置波特率
	USART3_InitStructure.USART_BaudRate = DEBUG_USART3_BAUDRATE;
	// 配置 针数据字长
	USART3_InitStructure.USART_WordLength = USART_WordLength_8b;
	// 配置停止位
	USART3_InitStructure.USART_StopBits = USART_StopBits_1;
	// 配置校验位
	USART3_InitStructure.USART_Parity = USART_Parity_No ;
	// 配置硬件流控制
	USART3_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// 配置工作模式，收发一起
	USART3_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// 完成串口的初始化配置
	USART_Init(DEBUG_USART3, &USART3_InitStructure);
	
	// 串口中断优先级配置
	NVIC_Usart3_Configuration();
	
	// 使能串口接收中断
	USART_ITConfig(DEBUG_USART3, USART_IT_RXNE, ENABLE);	
	
	// 使能串口
	USART_Cmd(DEBUG_USART3, ENABLE);		

  // 清除发送完成标志
	USART_ClearFlag(DEBUG_USART3, USART_FLAG_TC);     
}



/*****************  发送一个字节 **********************/
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
	/* 发送一个字节数据到USART */
	USART_SendData(pUSARTx,ch);
		
	/* 等待发送数据寄存器为空 */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/****************** 发送8位的数组 ************************/
void Usart_SendArray( USART_TypeDef * pUSARTx, uint8_t *array, uint16_t num)
{
  uint8_t i;
	
	for(i=0; i<num; i++)
  {
	    /* 发送一个字节数据到USART */
	    Usart_SendByte(pUSARTx,array[i]);	
  
  }
	/* 等待发送完成 */
	while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET);
}

/*****************  发送字符串 **********************/
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
  do 
  {
      Usart_SendByte( pUSARTx, *(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  /* 等待发送完成 */
  while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)
  {}
}

/*****************  发送一个16位数 **********************/
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* 取出高八位 */
	temp_h = (ch&0XFF00)>>8;
	/* 取出低八位 */
	temp_l = ch&0XFF;
	
	/* 发送高八位 */
	USART_SendData(pUSARTx,temp_h);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
	
	/* 发送低八位 */
	USART_SendData(pUSARTx,temp_l);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}





///********F4数据接收函数********/
//extern u8 name,location,startflag;
//;//接收openmv发送的数据

////接收F4传过来的数据
//void F4_Receive_Data(uint8_t *com_data)
//{ 
//		static uint8_t RxBuffer1[80]={0};//放置F4发送的数据
//		for(int j=0; j < 2; j++)
//		{
//				RxBuffer1[j] = com_data[j];
//		}
//		USART_ITConfig(DEBUG_USART3, USART_IT_RXNE,DISABLE);//关闭DTSABLE中断
//		USART_ITConfig(DEBUG_USART1, USART_IT_RXNE,DISABLE);//关闭DTSABLE中断
//		
//		location=RxBuffer1[0];
//		name=RxBuffer1[1];
//		printf ("location:%d name:%d \r\n",location,name);
//		if(location==57)
//		{
//			startflag=1;
//			printf("startflag=%d 开始初始化\r\n",startflag);
//		}
//		USART_ITConfig(DEBUG_USART3, USART_IT_RXNE,ENABLE);
//		USART_ITConfig(DEBUG_USART1, USART_IT_RXNE,ENABLE);
//}	

