#include "TOFSense.h"
#include "usart.h"

/*
			     TX   RX
TOF1    	 PC6	 PG11
TOF2    	 PC7	 PG12
*/

#define fulwork      		1     //�������ȫ������
#define fulfree      		2     //�������ȫ��������
#define singlework      3     //һ�Ų�๤�������Ų�಻����

int real=100; //�빳�Ӿ���       
int weight=43+56;//99 //����+��������  
int hook=43;//���ӳ���


extern unsigned char TOF_data[32] ;   //store 2 TOF frames
extern unsigned char TOF_data2[32];
extern unsigned char TOF_length;
extern unsigned char TOF_header[3];
extern unsigned long TOF_system_time;
extern unsigned long TOF_distance1;
extern unsigned long TOF_distance2;
extern unsigned long dis1;
extern unsigned long dis2;
extern unsigned char TOF_status;
extern unsigned int TOF_signal;
extern unsigned char TOF_check;
extern uint8_t count;
extern int qvwu_infer[2][9];
extern uint8_t location;
extern uint8_t servo1_zhuaqv;//1Ϊ����ץ��2Ϊ����һ��λ��ץ
extern uint8_t servo2_zhuaqv;//1Ϊ����ץ��2Ϊ����һ��λ��ץ




/**
 * @brief       ���ݰ�У�麯��
 * @param       data[],�������ݰ�
 * @param		len,���ݰ�����
 * @retval      1���ɹ���0��ʧ��
 */
int verifyCheckSum(unsigned char data[], unsigned char len)//У��
{
  int k;
	TOF_check = 0;
  for(k = 0;k<len-1;k++)
  {
     TOF_check += data[k];
  }

  if(TOF_check == data[len-1])
  {
//     printf("TOF data is ok!\r\n");
      return 1;    
  }else{
//      printf("TOF data is error!\r\n");
      return 0;  
  }

}

/**
 * @brief       ������ݴ���͹�������
 * @param       ��
 * @retval      ��
 */
void ceju(void)
{
	for(count = 0; count < 32; count++)
	{
		TOF_data[count] = USART2_RX_BUF[count];
		TOF_data2[count] = USART3_RX_BUF[count];
//		printf("%c",USART2_RX_BUF[count]);
//		printf("%c", USART3_RX_BUF[count]);
//		printf("\r\n");
	}
	if(count == 32)
	{	
		for(int j = 0; j < 16; j++)
		{
//			printf("99999");
//			printf("TOF_data[%d]==%d\r\n",j,TOF_data[j]);
			if((TOF_data[j]==0x57 && TOF_data[j+1]==0x00 && TOF_data[j+2]==0xFF) && (verifyCheckSum(&TOF_data[j],TOF_length)==1))
			{
				if(((TOF_data[j+12]) | (TOF_data[j+13]<<8) )==0)
				{
//								printf("Out of range!\r\n");
				}
				else
				{				
				 TOF_system_time = TOF_data[j+4] | TOF_data[j+5]<<8 | TOF_data[j+6]<<16 | TOF_data[j+7]<<24;
//								 printf("TOF system time is: %ld", TOF_system_time);
//								 printf("ms\r\n");
	
				 TOF_distance1 = (TOF_data[j+8]) | (TOF_data[j+9]<<8) | (TOF_data[j+10]<<16);
///								 printf("TOF distance1 is: %ld", TOF_distance1);
//								 printf("mm\r\n");
	
				 TOF_status = TOF_data[j+11];
//								 printf("TOF status is: %u", TOF_status);
//								 printf("\r\n");
	
				 TOF_signal = TOF_data[j+12] | TOF_data[j+13]<<8;
//								 printf("TOF signal is: %d", TOF_signal);
//								 printf("\r\n");
				if(TOF_data[j+3] == 0x00)	dis1 = TOF_distance1;	//����������ID��0��1
//					printf("dis1=%ldmm\r\n",dis1);

			}
		}
			if((TOF_data2[j]==0x57 && TOF_data2[j+1]==0x00 && TOF_data2[j+2]==0xFF) && (verifyCheckSum(&TOF_data2[j],TOF_length)==1))
			{
//				printf ("1");
				if(((TOF_data2[j+12]) | (TOF_data2[j+13]<<8) )==0)
				{
	//								printf("Out of range!\r\n");
				}
				else
				{						
				 TOF_system_time = TOF_data2[j+4] | TOF_data2[j+5]<<8 | TOF_data2[j+6]<<16 | TOF_data2[j+7]<<24;
	//								 printf("TOF system time is: %ld", TOF_system_time);
	//								 printf("ms\r\n");

				 TOF_distance2 = (TOF_data2[j+8]) | (TOF_data2[j+9]<<8) | (TOF_data2[j+10]<<16);
//								 printf("TOF distance2 is: %ld", TOF_distance2);
//								 printf("mm\r\n");
				
				 TOF_status = TOF_data2[j+11];
	//								 printf("TOF status is: %u", TOF_status);
	//								 printf("\r\n");

				 TOF_signal = TOF_data2[j+12] | TOF_data2[j+13]<<8;
	//								 printf("TOF signal is: %d", TOF_signal);
	//								 printf("\r\n");
				if(TOF_data2[j+3] == 0x01)  dis2 = TOF_distance2; //����������ID��0��1
//				printf("dis2=%ldmm\r\n",dis2);

				}
			}		
		}
//			printf("dis1=%ld and dis2=%ld\r\n",dis1,dis2);
			if(location == 1 )
			{
//				printf("")
					panduan(singlework,location);
//				printf("qvwu_infer[0][%d]==%d   qvwu_infer[1][%d]==%d",location-1,qvwu_infer[0][location-1],location-1,qvwu_infer[1][location-1]);
				if(qvwu_infer[0][location-1]!=0&&qvwu_infer[1][location-1]!=0)//ȡ�����鸳ֵ
				{
						if(qvwu_infer[0][location-1]==2&&qvwu_infer[1][location-1]==1)//1��צ������Ʒ��2��צ��û��Ʒ
						{
								servo1_zhuaqv=1;//����Ʒ����ץ
//								printf("/********ʶ�������й���***********/\r\n");
//								printf("\r\n");
								printf("section1ץȡ���⻷\r\n");
						}
						else if(qvwu_infer[0][location-1]==1&&qvwu_infer[1][location-1]==1)//1��צ��û����Ʒ��2��צ��û��Ʒ
						{
								servo1_zhuaqv=2;//��һ��λ��ץȡ
//								printf("/********ʶ�������޹���***********/\r\n");
//								printf("\r\n");
								printf("section1ץȡ���ڻ�\r\n");
						}
				}

			}
			else if (location == 4)
			{
					panduan(fulwork,location);
					if(qvwu_infer[0][location-1]!=0&&qvwu_infer[1][location-1]!=0)//ȡ�����鸳ֵ����
					{
							//��һ��צ��ץȡ����
							if(qvwu_infer[0][location-1]==2)//1��צ������Ʒ
							{
									servo1_zhuaqv=1;//����Ʒ����ץ
									printf("section2 servo1ץȡ���ڻ�\r\n");
							}
							else if(qvwu_infer[0][location-1]==1)//1��צ��û����Ʒ
							{
									servo1_zhuaqv=2;//����һ��λ��ץ
									printf("section2 servo1ץȡ���⻷\r\n");
							}
							//�ڶ���צ��ץȡ����
							if(qvwu_infer[1][location-1]==2)//2��צ������Ʒ
							{
									servo2_zhuaqv=1;//����Ʒ����ץ
									printf("section2 servo2ץȡ���ڻ�\r\n");
							}
							else if(qvwu_infer[1][location-1]==1)//2��צ��û����Ʒ
							{
									servo2_zhuaqv=2;//��һ��λ��ץȡ
									printf("section2 servo2ץȡ���⻷\r\n");
							}
					}
			}
			else if (location == 7)
			{
					panduan(fulwork,location);
					if(qvwu_infer[0][location-1]!=0&&qvwu_infer[1][location-1]!=0)//ȡ�����鸳ֵ
					{
							//��һ��צ��ץȡ����
							if(qvwu_infer[0][location-1]==2)//1��צ������Ʒ
							{
									servo1_zhuaqv=1;//����Ʒ����ץ
									printf("section3 servo1ץȡ���ڻ�\r\n");
							}
							else if(qvwu_infer[0][location-1]==1)//1��צ��û����Ʒ
							{
									servo1_zhuaqv=2;//����һ��λ��ץ
									printf("section3 servo1ץȡ���⻷\r\n");
							}
							//�ڶ���צ��ץȡ����
							if(qvwu_infer[1][location-1]==2)//2��צ������Ʒ
							{
									servo2_zhuaqv=1;//����Ʒ����ץ
									printf("section3 servo2ץȡ���ڻ�\r\n");
							}
							else if(qvwu_infer[1][location-1]==1)//2��צ��û����Ʒ
							{
									servo2_zhuaqv=2;//��һ��λ��ץȡ
									printf("section3 servo2ץȡ���⻷\r\n");
							}
					}
			}			
				count = 0;
	}
}
/* 
location  		1      4      7
dis1:     		1      1      1     
dis2:     		0      1      1     
quwu_infer[2][9]		
*/
/**
 * @brief       ������жϺ���
 * @param       sta,TOF�������������fulwork:ȫ��������singlework:TOF1������fulfree:ȫ������
 * @param		location,���λ��
 * @retval      void
 */
void panduan(uint8_t sta, uint8_t location)
{
	uint8_t Location = location - 1;
	switch(sta)
	{
			case fulwork:
					if(dis1<= 210)
							qvwu_infer[0][Location] = 2;//����Ʒ
					else if(dis1> 210)
							qvwu_infer[0][Location] = 1;//û����Ʒ
					if(dis2<= 205)
							qvwu_infer[1][Location] = 2;//����Ʒ
					else if(dis2> 205)
							qvwu_infer[1][Location] = 1;//û����Ʒ
					break;			
			case singlework:
					qvwu_infer[1][Location] = 1;//û����Ʒ
					if(dis1<= 210)
							qvwu_infer[0][Location] = 2;//����Ʒ
					else if(dis1> 210)
							qvwu_infer[0][Location] = 1;//û����Ʒ
					break;
			case fulfree:
					break;					
	}
}
//170-190
//232







