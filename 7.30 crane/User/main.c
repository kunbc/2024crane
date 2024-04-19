#include "stm32f10x.h"
#include "timer.h"		// ���Ӷ�ʱ������
#include "steppingmotor.h" 
#include "Servo.h"
#include "bsp_usart.h"
//#include "limitswitch.h"
#include "airpump.h"
#include "bsp_TiMbase.h" 
#include "logic.h"

/*****��־λ����*****/
int  testflag=0;//���Ժ�����־λ
//int	 switch1=0;//��λ����1��־λ
//int	 switch2=0;//��λ����2��־λ
int  Initflag = 0;			//���ػ���ʼ����־λ

u8  finishflag1=0,finishflag2=0,tranflag=0;//���ȡ�ﶯ����־λ

u8 loadflag12=0,startflag=0;
u8 bjload_flag=0,num=0,loady1=0,loady2=0,loadflag=0,load_bj1_fall_flag1=0,load_bj1_fall_flag2=0,load_bj1_fall_flag3=0,load_bj1_rise_flag=0,load_bj2_retreat_flag=0,load_bj2_advance_flag=0;
u8 djyundong_flag=0,bj_m_increase=0,bj_l_increase=0;
u8 bjchoose1=0,bjchoose2=0,bj_i=0,bj_j=3,bj_m=0,bj_mm=0,bj_l=0,bj_n=0,judge[6]={0},load=0;//1_dis1[bj_j][bj_i]  bj_l����ȡ������  bj_m :�������� bj_n��ǰ��  bj_j(0���� 1�ֿ�) bj_i (λ��)
u8 bj1_rise_flag=0,bj1_fall_flag=0,put_bj1_fall_flag=0,put_bj1_rise_flag=0;
u8 bj2_advance_flag=0,bj2_retreat_flag=0;
u32 bj1_count=0,bj2_count=0;
u8 fall_guocheng=0;
//choose1 57�˶� 1���½�  2������ 3; �����½�  4����������
//choose2 42�˶� 1��ǰ��  2������
//judge[]  1:����  2���ֿ�
/*
λ��1:����-��3000��3000��
1���ֿ�-��1000��2000-
λ��2������
*/
/*
ƽ�ƣ�
->:750
<-:3650
->:2950
*/
/*****��������*****/
uint8_t location=0;
uint8_t name=0;//����F4���͵�����//�������ݣ�1����  2�ֿ�  3����		//	��ɶ�������4

//������нǶ�
uint16_t Myservo[7]={140,20,260,104,68,176,212};//[0]:cola1(Init),[1]:cola2,[2]:cola3,[4]:box spin


#if 1
u16 bj1_dis1[5][6]={
{6200,6200,6200,6200,6200,6200},//  ����1�½�    bj_j+2*(bj_m-1)
{2000,2000,2000,2000,2000,2000},//  �ֿ۵�һ���½�
{2000,2000,2000,2000,2000,2000},//  һ���ֿ۶����½�
{5500,5500,5500,5500,5500,5500},//  �����ֿ۶����½�
{10000,10000,10000,10000,10000,10000},// �����ֿ۶����½�
},//
    bj1_dis2[4][6]={
{6200,6200,6200,6200,6200,6200},// ����1����   bj_j+2*bj_m
{4000,4000,4000,4000,4000,4000},// �ֿ�1����
{7500,7500,7500,7500,7500,7500},// �ֿ�2����
{12000,12000,12000,12000,12000,12000},// �ֿ�3����
//{6200,6200,6200,6200,6200,6200},// �ֿ�4����
//{12000,12000,12000,12000,12000,12000},// �ֿ�3����
},

//bj1_dis3[3][3]={
//{2000,5000},// ��������1   bj_j+2*bj_m
//{1000,0},// ��������2
//{9000,0},// ��������3
//},
//bj1_dis4[3][3]={
//{5500,5500},// �����½�1   bj_j+2*bj_m
//{0,5250},// �����½�2
//{0,0},// �����½�3
//},

bj1_dis3[3][3]={
{2000,5000},// ��������1   bj_j+2*bj_m
{5000,200},// ��������2
{6000,6000},// ��������3
},
bj1_dis4[3][3]={
{5500,5500},// �����½�1   bj_j+2*bj_m
{0,9500},// �����½�2
{1700,0},// �����½�3
},	


bj2_dis1[2][6]={//_n  0:�ֿ�΢ǰ��   1���ֿ۴�ǰ��
{1500,1500,1500,1500,1500,1500},//�ֿ�΢ǰ��
{6500,6500,6500,6500,6500,6500},//�ֿ۴�ǰ��
},

bj2_dis2[6]={8000,8000,8000,8000,8000,8000},//�ֿ۴����

bj2_dis3[3]={8000,8000,8000},//�����ǰ��
bj2_dis4[3]={8000,8000,8000},//��������

put_bj1_dis1[3]={1300,500,1000},// 3; �����½�
put_bj1_dis2[3]={1300,500,1000},// 4����������u8 judge[6]={0};

tran[3]={10000,6500,1500};//���ɽ׶�
u8 Crane_Init_flag=0; 

#elif 0
u16 bj1_dis1[5][6]={
{0,0,0,0,0,0},//  ����1�½�    bj_j+2*(bj_m-1)
{0,0,0,0,0,0},//  �ֿ۵�һ���½�
{0,0,0,0,0,0},//  һ���ֿ۶����½�
{0,0,0,0,0,0},//  �����ֿ۶����½�
{0,0,0,0,0,0},// �����ֿ۶����½�
},//
    bj1_dis2[4][6]={
{0,0,0,0,0,0},// ����1����   bj_j+2*bj_m
{0,0,0,0,0,0},// �ֿ�1����
{0,0,0,0,0,0},// �ֿ�2����
{0,0,0,0,0,0},// �ֿ�3����
//{6200,6200,6200,6200,6200,6200},// �ֿ�4����
//{12000,12000,12000,12000,12000,12000},// �ֿ�3����
},
bj1_dis3[3][3]={
{0,0},// ��������1   bj_j+2*bj_m
{0,0},// ��������2
{0,0},// ��������2
},
bj1_dis4[3][3]={
{0,0},// �����½�1   bj_j+2*bj_m
{0,0},// �����½�2
{0,0},// �����½�3
},	

bj2_dis1[2][6]={//_n  0:�ֿ�΢ǰ��   1���ֿ۴�ǰ��
{0,0,0,0,0,0},//�ֿ�΢ǰ��
{0,0,0,0,0,0},//�ֿ۴�ǰ��
},

bj2_dis2[6]={0,0,0,0,0,0},//�ֿ۴����

bj2_dis3[3]={0,0,0},//�����ǰ��
bj2_dis4[3]={0,0,0},//��������

put_bj1_dis1[3]={0,0,0},// 3; �����½�
put_bj1_dis2[3]={0,0,0},// 4����������u8 judge[6]={0};

tran[3]={0,0,0};//���ɽ׶�
u8 Crane_Init_flag=0;

#elif 0
u16 bj1_dis1[5][6]={
{3100,3100,3100,3100,3100,3100},//  ����1�½�    bj_j+2*(bj_m-1)
{2000/2,2000/2,2000/2,2000/2,1000,1000},//  �ֿ۵�һ���½�
{2000/2,2000/2,2000/2,2000/2,2000/2,2000/2},//  һ���ֿ۶����½�
{5500/1,5500/2,5500/2,5500/2,5500/2,5500/2},//  �����ֿ۶����½�
{10000/2,10000/2,10000/2,10000/2,10000/2,10000/2},// �����ֿ۶����½�
},//
    bj1_dis2[4][6]={
{6200/2,6200/2,6200/2,6200/2,6200/2,6200/2},// ����1����   bj_j+2*bj_m
{4000/2,4000/2,4000/2,4000/2,4000/2,4000/2},// �ֿ�1����
{7500/2,7500/2,7500/2,7500/2,7500/2,7500/2},// �ֿ�2����
{12000/2,12000/2,12000/2,12000/2,12000/2,12000/2},// �ֿ�3����
//{6200,6200,6200,6200,6200,6200},// �ֿ�4����
//{12000,12000,12000,12000,12000,12000},// �ֿ�3����
},
bj1_dis3[3][3]={
{2000/2,5000/2},// ��������1   bj_j+2*bj_m
{1000/2,0},// ��������2
{9000/2,0},// ��������3
},
bj1_dis4[3][3]={
{5500/2,5500/2},// �����½�1   bj_j+2*bj_m
{0,5250/2},// �����½�2
{0,0},// �����½�3
},	

bj2_dis1[2][6]={//_n  0:�ֿ�΢ǰ��   1���ֿ۴�ǰ��
{1500/2,1500/2,1500/2,1500/2,1500/2,1500/2},//�ֿ�΢ǰ��
{6500/2,6500/2,6500/2,6500/2,6500/2,6500/2},//�ֿ۴�ǰ��
},

bj2_dis2[6]={8000/2,8000/2,8000/2,8000/2,8000/2,8000/2},//�ֿ۴����

bj2_dis3[3]={8000/2,8000/2,8000/2},//�����ǰ��
bj2_dis4[3]={8000/2,8000/2,8000/2},//��������

put_bj1_dis1[3]={1300/2,500/2,1000/2},// 3; �����½�
put_bj1_dis2[3]={1300/2,500/2,1000/2},// 4����������u8 judge[6]={0};

tran[3]={10000/2,6500/2,1500/2};//���ɽ׶�
u8 Crane_Init_flag=0; 

#endif

/**********��������**************/
void Crane_Init (void);//��ʼ��ģ��
void step1(void);//ȡ��ģ��
void step2(void);//����ģ��
void step3(void);//����ģ��

/**********���Ժ���**************/
void test(void)
{

//			if(testflag==0)
//		{

//		airpump1(ON);
//		airpump2(ON);
//		airpump3(ON);
//		airpump4(ON);
//		TIM6_Delayms(5000);
//								testflag=1;
//		}
//	}
#if 0
	

			if(bj1_count<6000)
				{
//						//�½�  ǰ��
//					GPIO_ResetBits(GPIOD, GPIO_Pin_1);
						//����  ����						
						GPIO_SetBits(GPIOD, GPIO_Pin_1);
						TIM_Cmd(TIM5, ENABLE);
				}
			else
				{
					TIM_Cmd( TIM5, DISABLE);
					printf("bj1_count:%d\r\n",bj1_count);
					testflag=1;
				}
#elif	0
//�����
//
			if(bj2_count<3000)
				{
//						//ǰ��
//					GPIO_ResetBits(GPIOD, GPIO_Pin_3);
						//����						
						GPIO_SetBits(GPIOD, GPIO_Pin_3);
						TIM_Cmd(TIM2, ENABLE);
				}
			else
				{
					TIM_Cmd( TIM2, DISABLE);
					printf("bj2_count:%d\r\n",bj2_count);
				}		
				
#endif
}


int main(void)
{
	USART1_Config();
	USART3_Config();
	SystemInit();
	Servo_Init();
	direction_GPIO_Config();
	EleRelay_GPIO_Config();
	TIM6_Init();
	TIM5_Init();
	TIM2_Init();
//	limitswitch_Init();
	
while (1)
		{
			
			Servo_SetAngle(180);
			
			
////				test();	//���Ժ���
//		/****���ػ���ʼ��****/
//				Crane_Init ();
//		/****ȡ��****/
//				step1();//ȡ��ģ��
//		/****����****/
//				step2();//����ģ��
//		/****����****/
//				step3();//����ģ��
		}
}
/**********��ʼ����������************/
void Crane_Init (void)
{
	
		if(Initflag==0&&startflag==1)
		{
				if(bj1_count<6200)
				{
//						printf("��ʼ��\r\n");
						GPIO_SetBits(GPIOD, GPIO_Pin_1);
						TIM_Cmd( TIM5, ENABLE);
				}
				else
				{
						printf("bj1_count:%d\r\n",bj1_count);					
						TIM_Cmd( TIM5, DISABLE);
						Initflag=1;
						bj1_count=0;
				}
				//steppingmotor1(500,pulse22,0);//1M/500=2kHz //�����������-���壺5000
		}
		else if(Initflag==1)
		{			
//				Usart_SendByte(DEBUG_USART3,4);//�ǵøĻش���3
				printf ("��ɳ�ʼ������\r\n");					
				Servo_SetAngle(Myservo[0]);//�����λ
				Initflag=2;
		}
}

void step1(void)//ȡ��ģ��
{
		if(Initflag==2)
		{
			if(djyundong_flag==0&&(name == 49||name ==50))
			{
					djyundong();
	//			printf("bjchoose2=%d\r\n",bjchoose2);
					if(judge[bj_i]==1||judge[bj_i]==2)
					{
							bjbushu();
					}
			}
			else if(djyundong_flag==1)
			{
				bj_int();
				if(bj_i==5)
				{
						Initflag=3;
				}
				else if (bj_i<5)
				{
						Usart_SendByte(DEBUG_USART3,6);//�ǵøĻش���3
						printf("�������\r\n");
				}
			}
		}
}

void step2(void)//����ģ��
{
		if(Initflag==3)
		{
				transimt();
		}
}

void step3(void)//����ģ��
{
			if(Initflag==4||Initflag==5)
			{
//				if(Initflag==4)
//				{
//						if(bjload_flag==0&&location==55&&name==55)
//						{
//								bjload();
//								if(load==3)
//								{
//										load_bjbushu();
////										printf("bjload_flag=%d\r\n",bjload_flag);
//								}
//						}
//						else if(bjload_flag==1)
//						{
//									load_int();
//									Initflag=5;
//									Usart_SendByte(DEBUG_USART3,6);//�ǵøĻش���3
//									printf("Initflag=%d  bjload_flag=%d  location==%d  name=%d\r\n",Initflag,bjload_flag,location,name);
//						}
//				}
//				else if(Initflag==5)
//				{
//							if(location==55&&name==55)
//							{
//									for(int i=0;i<2;i++)
//									{	
//												if(bjload_flag==0&&location==55&&name==55)
//												{	
//															bjload();
//															if(load==3)
//															{
//																	load_bjbushu();
//															}
//												}
//												else if(bjload_flag==1)
//												{
//															load_bj1_fall_flag1=0;
//															load_bj2_retreat_flag=0;
//															load_bj1_rise_flag=0;
//															load_bj1_fall_flag2=0;
//															load_bj2_advance_flag=0;
//															load_bj1_fall_flag3=0;
//															bjload_flag=0;
//															loady1=0;
//															loady2=0;
//															location=55;
//															name=55;
//												}
//									}
//							}
//							if(loadflag==3)
//							{
//									load_int();
//							}

//				}
				if(Initflag==4)
				{
							if(bjload_flag==0&&location==55&&name==55&&loadflag<2)
							{

												bjload();
												if(bjload_flag==0&&load==3)
												{
														load_bjbushu();
												}
												else if(bjload_flag==1)
												{
															load_bj1_fall_flag1=0;
															load_bj2_retreat_flag=0;
															load_bj1_rise_flag=0;
															load_bj1_fall_flag2=0;
															load_bj2_advance_flag=0;
															load_bj1_fall_flag3=0;
															bjload_flag=0;
															loadflag12=1;
															loady1=0;
															loady2=0;
															location=55;
															name=55;
															printf("bjload_flag=%d\r\n&&location=%d\r\n&&name=%d\r\n",bjload_flag,location,name);
												}
									}
							else if(loadflag==2)
							{
									load_int();
									Initflag=5;
//									Usart_SendByte(DEBUG_USART3,6);//�ǵøĻش���3
//									printf("��ʼ�ڶ���������ʶ��\r\n");
							}

				}
				else if(Initflag==5)
				{
//						printf("Initflag=%d\r\n  bjload_flag=%d\r\n  location==%d\r\n  name=%d\r\n  loadflag=%d\r\n",Initflag,bjload_flag,location,name,loadflag);
						if(bjload_flag==0&&location==55&&name==55&&loadflag==2)
						{
								if(loadflag12==0)
								{
									loadflag12=3;
								}
								bjload();
								if(load==3)
								{
										load_bjbushu();
//										printf("bjload_flag=%d\r\n",bjload_flag);
								}
						}
						else if(bjload_flag==1&&loadflag==3)
						{
									load_int();
									airpump1(OFF);
									airpump2(OFF);
									airpump3(OFF);
									airpump4(OFF);
									Initflag=6;
									printf("Initflag=%d\r\n",Initflag);
						}
				}


		}
}

