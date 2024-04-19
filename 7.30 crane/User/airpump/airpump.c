#include "airpump.h"
 /**
  * @brief  ��ʼ�����Ƽ̵�����IO
  * @param  ��
  * @retval ��
  */
void EleRelay_GPIO_Config(void)
{		
		/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
		GPIO_InitTypeDef GPIO_InitStructure;

		/*����LED��ص�GPIO����ʱ��*/
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);
		/*ѡ��Ҫ���Ƶ�GPIO����*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;	

		/*��������ģʽΪͨ���������*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

		/*������������Ϊ50MHz */   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

		/*���ÿ⺯������ʼ��GPIO*/
		GPIO_Init(GPIOB, &GPIO_InitStructure);	

			/*����LED��ص�GPIO����ʱ��*/
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE);
		/*ѡ��Ҫ���Ƶ�GPIO����*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;	
		/*���ÿ⺯������ʼ��GPIO*/
		GPIO_Init(GPIOC, &GPIO_InitStructure);	

		/* �õ�����1��ƽ	*/
		GPIO_ResetBits(GPIOC, GPIO_Pin_5);
		/* �õ�����2��ƽ	*/
		GPIO_ResetBits(GPIOB, GPIO_Pin_1);
		/* �õ�����3��ƽ	*/
		GPIO_ResetBits(GPIOB, GPIO_Pin_2);	 
		/* �õ�����4��ƽ	*/
		GPIO_ResetBits(GPIOC, GPIO_Pin_4);	 
}


