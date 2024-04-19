#include "logic.h"
#include "bsp_usart.h"
#include "airpump.h"
#include "Servo.h"
#include "bsp_TiMbase.h" 

extern int Initflag;
extern uint8_t bjchoose1,bjchoose2,x1,y1,bj_i,bj_j,bj_m,bj_mm,bj_l,bj_n,judge[6],load;
extern uint16_t bj_dis1[2][6],bj_dis2[2][6],Myservo[7],tran[3],bj1_dis1[5][6],bj1_dis2[4][6],bj1_dis3[3][3],bj1_dis4[3][3],bj2_dis1[2][6],bj2_dis2[6],bj2_dis3[3],bj2_dis4[3],put_bj1_dis1[3],put_bj1_dis2[3];
extern u8 bj1_rise_flag,bj1_fall_flag,put_bj1_rise_flag,put_bj1_fall_flag;
extern u8 bj2_advance_flag,bj2_retreat_flag;
extern uint8_t location,name;
extern u8 djyundong_flag,bj_m_increase,bj_l_increase;
extern u8 fall_guocheng;
extern u8 tranflag,loady1,loady2,bjload_flag,num,loady,loadflag,load_bj1_rise_flag,load_bj2_retreat_flag,load_bj2_advance_flag,load_bj1_fall_flag1,load_bj1_fall_flag2,load_bj1_fall_flag3;
extern u32 bj1_count,bj2_count;
extern u8 name,location,loadflag12,startflag,finishflag1,finishflag2;


/********F4���ݽ��պ���********/
void F4_Receive_Data(uint8_t *com_data)
{ 
		static uint8_t RxBuffer1[80]={0};//����F4���͵�����
		for(int j=0; j < 2; j++)
		{
				RxBuffer1[j] = com_data[j];
		}
		USART_ITConfig(DEBUG_USART3, USART_IT_RXNE,DISABLE);//�ر�DTSABLE�ж�
		USART_ITConfig(DEBUG_USART1, USART_IT_RXNE,DISABLE);//�ر�DTSABLE�ж�
		
		location=RxBuffer1[0];
		name=RxBuffer1[1];
//		printf ("location:%d name:%d \r\n",location,name);
		if(location==57)
		{
				startflag=1;
				printf("startflag=%d ��ʼ��ʼ��\r\n",startflag);
		}
		else if(location==56)
		{
				tranflag=1;
				printf("tranflag=%d ��ʼ������ɽ׶�\r\n",tranflag);
		}
		USART_ITConfig(DEBUG_USART3, USART_IT_RXNE,ENABLE);
		USART_ITConfig(DEBUG_USART1, USART_IT_RXNE,ENABLE);
}	



void transimt(void)//���ɶ���
{
		if(num==0&&tranflag==1)
		{		
//				Usart_SendByte(DEBUG_USART3,4);//�ǵøĻش���3
//				printf("׼�����\r\n");
				if(bj1_count<tran[0])
				{
					GPIO_SetBits(GPIOD, GPIO_Pin_1);//����
					TIM_Cmd( TIM5, ENABLE);//�˶�
				}
				else
				{
						printf("����case 0 bj1_count=%d\r\n",bj1_count);
						TIM_Cmd( TIM5, DISABLE);
						bj1_count=0;
						num++;
				}
		}
		else if(num==1&&tranflag==1)
		{
				if(bj2_count<tran[1])
				{
					GPIO_SetBits(GPIOD, GPIO_Pin_3);//����
					TIM_Cmd( TIM2, ENABLE);//�˶�
				}
				else
				{		
						printf("����case 1 bj2_count=%d\r\n",bj2_count);
						TIM_Cmd( TIM2, DISABLE);
						bj2_count=0;
						num++;
				}
		}
		else if(num==2&&tranflag==1)
		{
				if(bj1_count<tran[2])
				{
						GPIO_ResetBits(GPIOD, GPIO_Pin_1);//1�½�
						TIM_Cmd( TIM5, ENABLE);//�˶�
				}
				else
				{
						printf("����case 2 bj1_count=%d\r\n",bj1_count);
						TIM_Cmd( TIM5, DISABLE);
						bj1_count=0;
						num++;
						Initflag=4;	
					  Usart_SendByte(DEBUG_USART3,6);//�ǵøĻش���3
						printf("׼������\r\n");					
						airpump4(ON);
						TIM6_Delayms(1000);
				}
		}
}

//choose1 57�˶� 1���½�  2������ 3; �����½�  4����������
//choose2 42�˶� 1��ǰ��  2������
//judge[]  1:����  2���ֿ�

void bj_int(void)
{
	finishflag1=0;
	finishflag2=0;
	name=0,location=0;
	bjchoose1=0;bjchoose2=0;
  bj_j=3,bj_n=0;
	bj1_rise_flag=0,bj1_fall_flag=0;
	put_bj1_rise_flag=0,put_bj1_fall_flag=0;
	bj2_advance_flag=0,bj2_retreat_flag=0;
	bj_m_increase=0,bj_l_increase=0;
	printf("bj_i=%d\r\n",bj_i);
	djyundong_flag=0;
	Servo_SetAngle(Myservo[0]);
	
}

void load_int(void)
{
		loadflag12=0;
		location=0,name=0;
		bjload_flag=0;
		load_bj1_fall_flag1=0;
		load_bj1_fall_flag2=0;load_bj1_fall_flag3=0;
		load_bj1_rise_flag=0;load_bj2_retreat_flag=0;
		load_bj2_advance_flag=0;loady1=0;loady2=0;
}

void djyundong(void)
{
		bj_dis_change();
	  
		if(judge[bj_i]==1)//ץȡ����--bj1�½� bj1����
		{
			bj_j=0;
			if(bj_l_increase==0)
			{
				bj_l++;
				switch(bj_l)
				{
					case 1:Servo_SetAngle(Myservo[0]);break;
					case 2:Servo_SetAngle(Myservo[1]);break;
					case 3:Servo_SetAngle(Myservo[2]);break;
					default:break;
				}

				bj_l_increase=1;
			}
      if(bj1_fall_flag==0)
			{
				bjchoose1=1;
//				printf("5555\r\n");
			}
			else if(bj1_fall_flag==1&&bj1_rise_flag ==0)
			{
				bjchoose1=2;
//				printf ("bjchoose1=%d",bjchoose1);
			}
			else if(bj1_rise_flag ==1)
			{
				djyundong_flag=1;//����
			}
		}
		else if(judge[bj_i]==2)//ץȡ�ֿ�--bj2΢ǰ�� bj1�½� bj1���� bj2���� bj1_put�½� bj1_put���� bj2 ��ǰ�� bj1�½�
		{
			bj_j=1;
			if(bj_m_increase==0)
			{
				bj_m++;
				bj_m_increase=1;
			}
			if(bj2_advance_flag==0)//bj2΢ǰ��
			{
				bjchoose2=1;
			}
			else if(bj1_fall_flag==0&&bj2_advance_flag==1)//bj1�½�
			{
				bjchoose1=1;
				fall_guocheng=1;
			}
			else if(bj1_fall_flag==1&&bj1_rise_flag ==0)//bj1���� 
			{
				bjchoose1=2;
			}
			else if(bj1_rise_flag==1&&bj2_retreat_flag ==0)//bj2����
			{
				bjchoose2=2;
			}
			else if(bj2_retreat_flag==1&&put_bj1_fall_flag ==0)//bj1_put�½�
			{
				bjchoose1=3;
			}
			else if(put_bj1_fall_flag==1&&put_bj1_rise_flag ==0)//bj1_put����
			{
				bjchoose1=4;
			}
			else if(put_bj1_rise_flag==1&&bj2_advance_flag ==1)//bj2 ��ǰ��
			{
				bjchoose2=1;
			}
			else if(bj2_advance_flag ==2&&bj1_fall_flag==1)//bj1�½�
			{
				bjchoose1=1;
				fall_guocheng=2;
			}
			else if(bj1_fall_flag==2)
			{
				djyundong_flag=1;//����
			}
		}		
/*****************///�������ͨ����������		
	  if(bjchoose1 ==1||bjchoose1 ==3)
		{
			GPIO_ResetBits(GPIOD, GPIO_Pin_1);//1�½�
			TIM_Cmd( TIM5, ENABLE);//�˶�
    //printf ("1\n");			
		}
    else if(bjchoose1 == 2||bjchoose1 ==4)
		{
			GPIO_SetBits(GPIOD, GPIO_Pin_1);//1�½�
			TIM_Cmd( TIM5, ENABLE);//�˶�		
		}
	  if(bjchoose2 ==1)
		{
			GPIO_ResetBits(GPIOD, GPIO_Pin_3);//1ǰ��
			TIM_Cmd( TIM2, ENABLE);//�˶�
//    printf ("1\n");			
		}
    else if(bjchoose2 == 2)
		{
			GPIO_SetBits(GPIOD, GPIO_Pin_3);//1ǰ��
			TIM_Cmd( TIM2, ENABLE);//�˶�
			
		}	
}

void bjload(void)
{
	bj_dis_change();
	if(load==3)
	{
		if(load_bj1_fall_flag1==0)//����
		{		
				bjchoose1=1;
		}
		else if(load_bj1_fall_flag1==1&&load_bj2_retreat_flag==0)//ǰ��
		{
				bjchoose2=1;
		}
		else if(load_bj2_retreat_flag==1&&load_bj1_rise_flag==0)//�½�
		{
				bjchoose1=2;
		}
		else if(load_bj1_rise_flag==1&&load_bj1_fall_flag2==0)//����
		{
				bjchoose1=3;
		}
		else if(load_bj1_fall_flag2==1&&load_bj2_advance_flag==0)//����
		{
				bjchoose2=2;
		}
		else if(load_bj2_advance_flag==1&&load_bj1_fall_flag3==0)//�½�
		{
				bjchoose1=4;
		}
		else if(load_bj1_fall_flag3==1)//����
		{
			loadflag++;//���õ����Ӽ���
			printf("loadflag:%d",loadflag);
			bjload_flag=1;
		}
	}
/***********************�������ͨ������
choose1 57�˶� 1��ȡ������  2�������½� 3;���� ����  4��ȡ���½�
choose2 42�˶� 1��ǰ��  2������
judge[]  1:����  2���ֿ�*******************/
	  if(bjchoose1 ==2||bjchoose1 ==4)
		{
			GPIO_ResetBits(GPIOD, GPIO_Pin_1);//1�½�
			TIM_Cmd( TIM5, ENABLE);//�˶�
    //printf ("1\n");			
		}
    else if(bjchoose1 == 1||bjchoose1 ==3)
		{
			GPIO_SetBits(GPIOD, GPIO_Pin_1);//����
			TIM_Cmd( TIM5, ENABLE);//�˶�		
		}
	  if(bjchoose2 ==1)
		{
			GPIO_ResetBits(GPIOD, GPIO_Pin_3);//1ǰ��
			TIM_Cmd( TIM2, ENABLE);//�˶�
//    printf ("1\n");			
		}
    else if(bjchoose2 == 2)
		{
			GPIO_SetBits(GPIOD, GPIO_Pin_3);//����
			TIM_Cmd( TIM2, ENABLE);//�˶�
			
		}
}



void bj_dis_change(void)
{
		if(location == 49)
		{
			bj_i=0;
      judge_change();
		}
		if(location == 50)
		{
			bj_i=1;
      judge_change();
		}
		if(location == 51)
		{
			bj_i=2;
      judge_change();
		}
		if(location == 52)
		{
			bj_i=3;
      judge_change();
		}
		if(location == 53)
		{
			bj_i=4;
      judge_change();
		}
		if(location == 54)
		{
			bj_i=5;
      judge_change();
		}
		if(location == 55)
		{
			bj_i=6;
      judge_change();
		}
}

void judge_change(void)
{
			if(name == 49)
			{
				judge[bj_i] = 1;
			}
			else if(name == 50)
			{
					judge[bj_i] = 2;
			}	
			else if(name == 55)
			{
					load = 3;
			}
}
void bjbushu(void)
{
				 if(bjchoose1 == 1)//���������� �½�
					{
//						if(bj_m==0)
//						{
//							bj_mm=1;
//						}
//						else 
//						{
//							bj_mm=bj_m;
//						}
						if(bj_j==0)//kele
						{
							bj_mm=0;
						}
						else if(bj_j==1&&fall_guocheng==1)//
						{
							bj_mm=0;
						}
						else if(bj_j==1&&fall_guocheng==2)
						{
							bj_mm=bj_m;
						}
//						if(bj1_count < bj1_dis1[bj_j+bj_mm][bj_i])
//							{
//								bj1_count++;
////								printf("bj1_count:%d\r\n",bj1_count);
//							}
						if(bj1_count > bj1_dis1[bj_j+bj_mm][bj_i])
							{
								printf("bjchoose1=1 bj1_count:%d\r\n",bj1_count);
//								printf("bj_j=%d bj_i=%d bj_m=%d",bj_j,bj_i,bj_m);
//								printf("bj1_dis1[bj_j+2*(bj_m-1)][bj_i]=%d",bj1_dis1[bj_j+2*(bj_m-1)][bj_i]);
								TIM_Cmd( TIM5, DISABLE);
								if(name==50&&bj_mm==0)
								{
									printf("����4����\r\n");
									airpump4(ON);
									TIM6_Delayms(1000);
								}
								else if(name==49)
								{
										switch(bj_l)
										{
											case 1:airpump3(ON);break;
											case 2:airpump1(ON);break;
											case 3:airpump2(ON);break;
											default:break;
										}
										TIM6_Delayms(1500);
								}
								bj1_count = 0;bjchoose1 =0;
								if(bj1_fall_flag==0)
								{
									bj1_fall_flag=1;
								}
								else if(bj1_fall_flag ==1&&bj2_advance_flag ==2)
								{
									bj1_fall_flag=2;
								}
	//												printf("�����ж�1bjchoose=%d\r\n",bjchoose);
							}
					}
				else if(bjchoose1 == 2)//���������� ����
					{	
//						if(bj1_count < bj1_dis1[bj_j+2*(bj_m-1)][bj_i])
//							{
//								bj1_count++;
////								printf("bjchoose1=2 bj1_count:%d\r\n",bj1_count);
//							}
						if(bj_j==0)//kele
						{
							bj_mm=0;
						}
						else if(bj_j==1)//
						{
							bj_mm=bj_m-1;
						}
//						if (bj_i<=5&&bj1_count>bj1_dis2[bj_j+bj_mm][bj_i]/2&&finishflag==0)
//						{
//								Usart_SendByte(DEBUG_USART3,4);//�ǵøĻش���3
//								printf("��ȡ���\r\n");
//								finishflag=1;
//						}
//						printf("bj_i=%d  bj1_count=%d  finishflag2=%d  bj_j=%d",bj_i,bj1_count,finishflag2,bj_j);
							if(bj_i<=5&&bj1_count>bj1_dis2[bj_j+bj_mm][bj_i]/2&&finishflag2==0&&bj_j==0)
							{
									Usart_SendByte(DEBUG_USART3,4);//�ǵøĻش���3
									printf("��ȡ���\r\n");
									finishflag2=1;
							}								
						if(bj1_count > bj1_dis2[bj_j+bj_mm][bj_i])
							{
								printf("bjchoose1=2 bj1_count:%d\r\n",bj1_count);
								TIM_Cmd( TIM5, DISABLE);
								if(name==50)
								{
									printf("��� bj_i:%d\r\n",bj_i);
									switch(bj_i)
									{
										case 0:Servo_SetAngle(Myservo[0]);break;
										case 1:Servo_SetAngle(Myservo[3]);break;
										case 2:Servo_SetAngle(Myservo[4]);break;
										case 3:Servo_SetAngle(Myservo[0]);break;
										case 4:Servo_SetAngle(Myservo[5]);break;
										case 5:Servo_SetAngle(Myservo[6]);break;
										default:break;
									}
								}	
								else if(name==49)
								{
										Servo_SetAngle(Myservo[0]);
								}
//								if (bj_i<5)
//								{
//										Usart_SendByte(DEBUG_USART3,4);//�ǵøĻش���3
//										printf("��ȡ���\r\n");
//								}
								bj1_count = 0;bjchoose1 =0;
								bj1_rise_flag = 1;
	//												printf("�����ж�1bjchoose=%d\r\n",bjchoose);
							}
					}
/****************************************************/	
				 else if(bjchoose1 == 3)//���������� �����½�
					{
//						if(bj1_count < put_bj1_dis1[bj_m-1])
//							{
//								bj1_count++;
////								printf("bjchoose1=3 bj1_count:%d\r\n",bj1_count);
//							}
						if(bj1_count > put_bj1_dis1[bj_m-1])
							{
								printf("bjchoose1=3 bj1_count:%d\r\n",bj1_count);
								TIM_Cmd( TIM5, DISABLE);
								airpump4(OFF);
								TIM6_Delayms(2000);
								bj1_count = 0;bjchoose1 =0;
								put_bj1_fall_flag =1;
	//												printf("�����ж�1bjchoose=%d\r\n",bjchoose);
							}
					}
				else if(bjchoose1 == 4)//���������� ���ú�����
					{	
//						if(bj1_count < put_bj1_dis1[bj_m-1])
//							{
////								printf("bjchoose1=4 bj1_count:%d\r\n",bj1_count);
//								bj1_count++;
//							}
						if(bj1_count > put_bj1_dis1[bj_m-1])
							{	
								printf("bjchoose1=4 bj1_count:%d\r\n",bj1_count);
								TIM_Cmd( TIM5, DISABLE);
								bj1_count = 0;bjchoose1 =0;
								put_bj1_rise_flag = 1;
	//												printf("�����ж�1bjchoose=%d\r\n",bjchoose);
							}
					}

					
/****************************************************/					
				 if(bjchoose2 == 1)//��������5000�� ǰ��
					{
//						if(bj2_count < bj2_dis1[bj_n][bj_i])
//							{
//								bj2_count++;
////								printf("bjchoose2=1 bj2_count:%d\r\n",bj2_count);
//							}
						if (bj_i<=5&&bj2_count>bj2_dis1[bj_j+bj_mm][bj_i]/2&&finishflag1==0&&bj2_advance_flag==1&&bj_j==1)
							{
									Usart_SendByte(DEBUG_USART3,4);//�ǵøĻش���3
									printf("��ȡ���\r\n");
									finishflag1=1;
							}
						if(bj2_count > bj2_dis1[bj_n][bj_i])
							{
								printf("bjchoose2=%d bj2_count:%d\r\n",bjchoose2,bj2_count);
								TIM_Cmd( TIM2, DISABLE);
								bj2_count = 0;bjchoose2 =0;
								bj_n++;
								if(bj_n==1)
								{
								   bj2_advance_flag =1;
								}
								else if(bj_n==2)
								{
									 bj2_advance_flag =2;
								}
	//												printf("�����ж�1bjchoose=%d\r\n",bjchoose);
							}
					}
				else if(bjchoose2 == 2)//��������5000�� ����
					{	
//						if(bj2_count < bj2_dis2[bj_i])
//							{
////								printf("bjchoose2=2 bj2_count:%d\r\n",bj2_count);
//								bj2_count++;
//							}
						if(bj2_count > bj2_dis2[bj_i])
							{	
								printf("bjchoose2=2 bj2_count:%d\r\n",bj2_count);								
								TIM_Cmd( TIM2, DISABLE);
								bj2_count = 0;bjchoose2 =0;
								bj2_retreat_flag =1;
	//												printf("�����ж�1bjchoose=%d\r\n",bjchoose);
							}
					}					
}

void load_bjbushu(void)
{
		if(bjchoose1==1)
		{
				if(bj1_count>bj1_dis3[loadflag][loady1]/2&&loadflag12==3&&bjload_flag==0)
				{
						 Usart_SendByte(DEBUG_USART3,7);//�ǵøĻش���3
						 printf("�ڶ��������ϱ�־λ\r\n");
						 loadflag12=4;
				}  

				if(bj1_count>bj1_dis3[loadflag][loady1])
				{
							printf("��һ��: bjchoose1=1  bj1_count:%d\r\n",bj1_count);	
							TIM_Cmd( TIM5, DISABLE);
							bj1_count=0;bjchoose1=0;
							load_bj1_fall_flag1=1;
							loady1++;
				}
		}
		else if(bjchoose2==1)
		{
				if(bj2_count>bj2_dis3[loadflag])
				{
							printf("�ڶ���: bjchoose2=1  bj2_count:%d\r\n",bj2_count);
							TIM_Cmd( TIM2, DISABLE);
							bj2_count=0;bjchoose2=0;
							load_bj2_retreat_flag=1;
				}
		}
		else if(bjchoose1==2)
		{
				if(bj1_count>bj1_dis4[loadflag][loady2])
				{
							printf("������: bjchoose1=2  bj1_count:%d\r\n",bj1_count);
							TIM_Cmd( TIM5, DISABLE);
							if(loadflag<2)
							{
									airpump4(OFF);
									TIM6_Delayms(2000);
									printf("ֻ������\r\n");

							}
							else
							{
									airpump1(OFF);
									airpump2(OFF);
									airpump3(OFF);
									airpump4(OFF);
									TIM6_Delayms(3000);
									printf("һ���\r\n");
							}
							bj1_count=0;bjchoose1=0;
							load_bj1_rise_flag=1;
							loady2++;
				}
		}
		else if(bjchoose1==3)
		{
				if(bj1_count>bj1_dis3[loadflag][loady1]/2&&loadflag12==1&&bjload_flag==0)
				{
							Usart_SendByte(DEBUG_USART3,5);//�ǵøĻش���3
							printf("��һ�������ϱ�־λ\r\n");
							loadflag12=2;
				}
				if(bj1_count>bj1_dis3[loadflag][loady1])
				{
							printf("���Ĳ�: bjchoose1=3  bj1_count:%d\r\n",bj1_count);
							TIM_Cmd( TIM5, DISABLE);
							bj1_count=0;bjchoose1=0;
							load_bj1_fall_flag2=1;
				}
		}
		else if(bjchoose2==2)
		{
				if(bj2_count>bj2_dis4[loadflag])
				{
							printf("���岽: bjchoose2=2 bj2_count=%d\r\n",bj2_count);
							TIM_Cmd( TIM2, DISABLE);
							bj2_count=0;bjchoose2=0;
							load_bj2_advance_flag=1;
				}
		}
		else if(bjchoose1==4)
		{
				if(bj1_count>bj1_dis4[loadflag][loady2])
				{
							printf("������\r\n");
							printf("bjchoose1=4 bj1_count=%d\r\n",bj1_count);
							TIM_Cmd( TIM5, DISABLE);
//							if(loadflag==0)
//							{
//									Usart_SendByte(DEBUG_USART3,5);//�ǵøĻش���3
//									printf("������ϱ�־λ");
//							}
							if(loadflag12==4)
							{							
								airpump4(OFF);
							}
							else
							{
								airpump4(ON);
								TIM6_Delayms(1000);
							}
							bj1_count=0;bjchoose1=0;
							load_bj1_fall_flag3=1;				
				}
		}
}
