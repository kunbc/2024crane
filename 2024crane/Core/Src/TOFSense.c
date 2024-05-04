#include "TOFSense.h"
#include "usart.h"

/*
			     TX   RX
TOF1    	 PC6	 PG11
TOF2    	 PC7	 PG12
*/

#define fulwork      		1     //两个测距全部工作
#define fulfree      		2     //两个测距全部不工作
#define singlework      3     //一号测距工作，二号测距不工作

int real=100; //离钩子距离       
int weight=43+56;//99 //钩子+砝码身长度  
int hook=43;//钩子长度


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
extern uint8_t servo1_zhuaqv;//1为立即抓，2为在连一个位置抓
extern uint8_t servo2_zhuaqv;//1为立即抓，2为在连一个位置抓




/**
 * @brief       数据包校验函数
 * @param       data[],传入数据包
 * @param		len,数据包长度
 * @retval      1：成功；0：失败
 */
int verifyCheckSum(unsigned char data[], unsigned char len)//校验
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
 * @brief       测距数据处理和工作函数
 * @param       无
 * @retval      无
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
				if(TOF_data[j+3] == 0x00)	dis1 = TOF_distance1;	//设置了两个ID，0和1
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
				if(TOF_data2[j+3] == 0x01)  dis2 = TOF_distance2; //设置了两个ID，0和1
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
				if(qvwu_infer[0][location-1]!=0&&qvwu_infer[1][location-1]!=0)//取物数组赋值
				{
						if(qvwu_infer[0][location-1]==2&&qvwu_infer[1][location-1]==1)//1号爪子有物品，2号爪子没物品
						{
								servo1_zhuaqv=1;//有物品立即抓
//								printf("/********识别结果：有钩码***********/\r\n");
//								printf("\r\n");
								printf("section1抓取在外环\r\n");
						}
						else if(qvwu_infer[0][location-1]==1&&qvwu_infer[1][location-1]==1)//1号爪子没有物品，2号爪子没物品
						{
								servo1_zhuaqv=2;//下一个位置抓取
//								printf("/********识别结果：无钩码***********/\r\n");
//								printf("\r\n");
								printf("section1抓取在内环\r\n");
						}
				}

			}
			else if (location == 4)
			{
					panduan(fulwork,location);
					if(qvwu_infer[0][location-1]!=0&&qvwu_infer[1][location-1]!=0)//取物数组赋值检验
					{
							//第一个爪子抓取安排
							if(qvwu_infer[0][location-1]==2)//1号爪子有物品
							{
									servo1_zhuaqv=1;//有物品立即抓
									printf("section2 servo1抓取在内环\r\n");
							}
							else if(qvwu_infer[0][location-1]==1)//1号爪子没有物品
							{
									servo1_zhuaqv=2;//在下一个位置抓
									printf("section2 servo1抓取在外环\r\n");
							}
							//第二个爪子抓取安排
							if(qvwu_infer[1][location-1]==2)//2号爪子有物品
							{
									servo2_zhuaqv=1;//有物品立即抓
									printf("section2 servo2抓取在内环\r\n");
							}
							else if(qvwu_infer[1][location-1]==1)//2号爪子没有物品
							{
									servo2_zhuaqv=2;//下一个位置抓取
									printf("section2 servo2抓取在外环\r\n");
							}
					}
			}
			else if (location == 7)
			{
					panduan(fulwork,location);
					if(qvwu_infer[0][location-1]!=0&&qvwu_infer[1][location-1]!=0)//取物数组赋值
					{
							//第一个爪子抓取安排
							if(qvwu_infer[0][location-1]==2)//1号爪子有物品
							{
									servo1_zhuaqv=1;//有物品立即抓
									printf("section3 servo1抓取在内环\r\n");
							}
							else if(qvwu_infer[0][location-1]==1)//1号爪子没有物品
							{
									servo1_zhuaqv=2;//在下一个位置抓
									printf("section3 servo1抓取在外环\r\n");
							}
							//第二个爪子抓取安排
							if(qvwu_infer[1][location-1]==2)//2号爪子有物品
							{
									servo2_zhuaqv=1;//有物品立即抓
									printf("section3 servo2抓取在内环\r\n");
							}
							else if(qvwu_infer[1][location-1]==1)//2号爪子没有物品
							{
									servo2_zhuaqv=2;//下一个位置抓取
									printf("section3 servo2抓取在外环\r\n");
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
 * @brief       测距结果判断函数
 * @param       sta,TOF工作情况参数，fulwork:全部工作，singlework:TOF1工作，fulfree:全部空闲
 * @param		location,测距位置
 * @retval      void
 */
void panduan(uint8_t sta, uint8_t location)
{
	uint8_t Location = location - 1;
	switch(sta)
	{
			case fulwork:
					if(dis1<= 210)
							qvwu_infer[0][Location] = 2;//有物品
					else if(dis1> 210)
							qvwu_infer[0][Location] = 1;//没有物品
					if(dis2<= 205)
							qvwu_infer[1][Location] = 2;//有物品
					else if(dis2> 205)
							qvwu_infer[1][Location] = 1;//没有物品
					break;			
			case singlework:
					qvwu_infer[1][Location] = 1;//没有物品
					if(dis1<= 210)
							qvwu_infer[0][Location] = 2;//有物品
					else if(dis1> 210)
							qvwu_infer[0][Location] = 1;//没有物品
					break;
			case fulfree:
					break;					
	}
}
//170-190
//232







