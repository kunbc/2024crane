// ������ʱ��TIMx,x[6,7]��ʱ��ʼ������

#include "bsp_TiMbase.h" 


/*
 * ע�⣺TIM_TimeBaseInitTypeDef�ṹ��������5����Ա��TIM6��TIM7�ļĴ�������ֻ��
 * TIM_Prescaler��TIM_Period������ʹ��TIM6��TIM7��ʱ��ֻ���ʼ����������Ա���ɣ�
 * ����������Ա��ͨ�ö�ʱ���͸߼���ʱ������.
 *-----------------------------------------------------------------------------
 *typedef struct
 *{ TIM_Prescaler            ����
 *	TIM_CounterMode			     TIMx,x[6,7]û�У���������
 *  TIM_Period               ����
 *  TIM_ClockDivision        TIMx,x[6,7]û�У���������
 *  TIM_RepetitionCounter    TIMx,x[1,8,15,16,17]����
 *}TIM_TimeBaseInitTypeDef; 
 *-----------------------------------------------------------------------------
 */

void TIM6_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		
		// ������ʱ��ʱ��,���ڲ�ʱ��CK_INT=72M
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	
		// �Զ���װ�ؼĴ�����ֵ���ۼ�TIM_Period+1��Ƶ�ʺ����һ�����»����ж�
    TIM_TimeBaseStructure.TIM_Period = 50000-1;	

	  // ʱ��Ԥ��Ƶ��Ϊ
    TIM_TimeBaseStructure.TIM_Prescaler= 72-1;
	
	  // ��ʼ����ʱ��
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
}


//΢�뼶��ʱ
void TIM6_Delayus(u16 xus)
{
	TIM_Cmd(TIM6,ENABLE); //������ʱ��
	while(TIM6->CNT < xus);
	TIM6->CNT = 0;
	TIM_Cmd(TIM6,DISABLE); //�رն�ʱ��
}
//���뼶��ʱ
void TIM6_Delayms(u16 xms)
{
	int i;
	for(i=0;i<xms;i++)
	{
		TIM6_Delayus(1000);
	}
}
/*********************************************END OF FILE**********************/
