#include "stm32f10x.h"
#include "timer.h"		// 主从定时器配置
#include "steppingmotor.h" 
#include "Servo.h"
#include "bsp_usart.h"
//#include "limitswitch.h"
#include "airpump.h"
#include "bsp_TiMbase.h" 
#include "logic.h"

/*****标志位定义*****/
int  testflag=0;//测试函数标志位
//int	 switch1=0;//限位开关1标志位
//int	 switch2=0;//限位开关2标志位
int  Initflag = 0;			//起重机初始化标志位

u8  finishflag1=0,finishflag2=0,tranflag=0;//完成取物动作标志位

u8 loadflag12=0,startflag=0;
u8 bjload_flag=0,num=0,loady1=0,loady2=0,loadflag=0,load_bj1_fall_flag1=0,load_bj1_fall_flag2=0,load_bj1_fall_flag3=0,load_bj1_rise_flag=0,load_bj2_retreat_flag=0,load_bj2_advance_flag=0;
u8 djyundong_flag=0,bj_m_increase=0,bj_l_increase=0;
u8 bjchoose1=0,bjchoose2=0,bj_i=0,bj_j=3,bj_m=0,bj_mm=0,bj_l=0,bj_n=0,judge[6]={0},load=0;//1_dis1[bj_j][bj_i]  bj_l：已取可乐数  bj_m :放置箱数 bj_n：前进  bj_j(0可乐 1乐扣) bj_i (位置)
u8 bj1_rise_flag=0,bj1_fall_flag=0,put_bj1_fall_flag=0,put_bj1_rise_flag=0;
u8 bj2_advance_flag=0,bj2_retreat_flag=0;
u32 bj1_count=0,bj2_count=0;
u8 fall_guocheng=0;
//choose1 57运动 1：下降  2：上升 3; 放置下降  4：放置上升
//choose2 42运动 1：前移  2：后退
//judge[]  1:可乐  2：乐扣
/*
位置1:可乐-降3000升3000，
1个乐扣-降1000升2000-
位置2：可乐
*/
/*
平移：
->:750
<-:3650
->:2950
*/
/*****变量定义*****/
uint8_t location=0;
uint8_t name=0;//接收F4发送的数据//接收数据：1可乐  2乐扣  3堆码		//	完成动作后发送4

//舵机运行角度
uint16_t Myservo[7]={140,20,260,104,68,176,212};//[0]:cola1(Init),[1]:cola2,[2]:cola3,[4]:box spin


#if 1
u16 bj1_dis1[5][6]={
{6200,6200,6200,6200,6200,6200},//  可乐1下降    bj_j+2*(bj_m-1)
{2000,2000,2000,2000,2000,2000},//  乐扣第一次下降
{2000,2000,2000,2000,2000,2000},//  一箱乐扣二次下降
{5500,5500,5500,5500,5500,5500},//  两箱乐扣二次下降
{10000,10000,10000,10000,10000,10000},// 三箱乐扣二次下降
},//
    bj1_dis2[4][6]={
{6200,6200,6200,6200,6200,6200},// 可乐1上升   bj_j+2*bj_m
{4000,4000,4000,4000,4000,4000},// 乐扣1上升
{7500,7500,7500,7500,7500,7500},// 乐扣2上升
{12000,12000,12000,12000,12000,12000},// 乐扣3上升
//{6200,6200,6200,6200,6200,6200},// 乐扣4上升
//{12000,12000,12000,12000,12000,12000},// 乐扣3上升
},

//bj1_dis3[3][3]={
//{2000,5000},// 置物上升1   bj_j+2*bj_m
//{1000,0},// 置物上升2
//{9000,0},// 置物上升3
//},
//bj1_dis4[3][3]={
//{5500,5500},// 置物下降1   bj_j+2*bj_m
//{0,5250},// 置物下降2
//{0,0},// 置物下降3
//},

bj1_dis3[3][3]={
{2000,5000},// 置物上升1   bj_j+2*bj_m
{5000,200},// 置物上升2
{6000,6000},// 置物上升3
},
bj1_dis4[3][3]={
{5500,5500},// 置物下降1   bj_j+2*bj_m
{0,9500},// 置物下降2
{1700,0},// 置物下降3
},	


bj2_dis1[2][6]={//_n  0:乐扣微前移   1：乐扣大前移
{1500,1500,1500,1500,1500,1500},//乐扣微前移
{6500,6500,6500,6500,6500,6500},//乐扣大前移
},

bj2_dis2[6]={8000,8000,8000,8000,8000,8000},//乐扣大后退

bj2_dis3[3]={8000,8000,8000},//置物大前移
bj2_dis4[3]={8000,8000,8000},//置物大后退

put_bj1_dis1[3]={1300,500,1000},// 3; 放置下降
put_bj1_dis2[3]={1300,500,1000},// 4：放置上升u8 judge[6]={0};

tran[3]={10000,6500,1500};//过渡阶段
u8 Crane_Init_flag=0; 

#elif 0
u16 bj1_dis1[5][6]={
{0,0,0,0,0,0},//  可乐1下降    bj_j+2*(bj_m-1)
{0,0,0,0,0,0},//  乐扣第一次下降
{0,0,0,0,0,0},//  一箱乐扣二次下降
{0,0,0,0,0,0},//  两箱乐扣二次下降
{0,0,0,0,0,0},// 三箱乐扣二次下降
},//
    bj1_dis2[4][6]={
{0,0,0,0,0,0},// 可乐1上升   bj_j+2*bj_m
{0,0,0,0,0,0},// 乐扣1上升
{0,0,0,0,0,0},// 乐扣2上升
{0,0,0,0,0,0},// 乐扣3上升
//{6200,6200,6200,6200,6200,6200},// 乐扣4上升
//{12000,12000,12000,12000,12000,12000},// 乐扣3上升
},
bj1_dis3[3][3]={
{0,0},// 置物上升1   bj_j+2*bj_m
{0,0},// 置物上升2
{0,0},// 置物上升2
},
bj1_dis4[3][3]={
{0,0},// 置物下降1   bj_j+2*bj_m
{0,0},// 置物下降2
{0,0},// 置物下降3
},	

bj2_dis1[2][6]={//_n  0:乐扣微前移   1：乐扣大前移
{0,0,0,0,0,0},//乐扣微前移
{0,0,0,0,0,0},//乐扣大前移
},

bj2_dis2[6]={0,0,0,0,0,0},//乐扣大后退

bj2_dis3[3]={0,0,0},//置物大前移
bj2_dis4[3]={0,0,0},//置物大后退

put_bj1_dis1[3]={0,0,0},// 3; 放置下降
put_bj1_dis2[3]={0,0,0},// 4：放置上升u8 judge[6]={0};

tran[3]={0,0,0};//过渡阶段
u8 Crane_Init_flag=0;

#elif 0
u16 bj1_dis1[5][6]={
{3100,3100,3100,3100,3100,3100},//  可乐1下降    bj_j+2*(bj_m-1)
{2000/2,2000/2,2000/2,2000/2,1000,1000},//  乐扣第一次下降
{2000/2,2000/2,2000/2,2000/2,2000/2,2000/2},//  一箱乐扣二次下降
{5500/1,5500/2,5500/2,5500/2,5500/2,5500/2},//  两箱乐扣二次下降
{10000/2,10000/2,10000/2,10000/2,10000/2,10000/2},// 三箱乐扣二次下降
},//
    bj1_dis2[4][6]={
{6200/2,6200/2,6200/2,6200/2,6200/2,6200/2},// 可乐1上升   bj_j+2*bj_m
{4000/2,4000/2,4000/2,4000/2,4000/2,4000/2},// 乐扣1上升
{7500/2,7500/2,7500/2,7500/2,7500/2,7500/2},// 乐扣2上升
{12000/2,12000/2,12000/2,12000/2,12000/2,12000/2},// 乐扣3上升
//{6200,6200,6200,6200,6200,6200},// 乐扣4上升
//{12000,12000,12000,12000,12000,12000},// 乐扣3上升
},
bj1_dis3[3][3]={
{2000/2,5000/2},// 置物上升1   bj_j+2*bj_m
{1000/2,0},// 置物上升2
{9000/2,0},// 置物上升3
},
bj1_dis4[3][3]={
{5500/2,5500/2},// 置物下降1   bj_j+2*bj_m
{0,5250/2},// 置物下降2
{0,0},// 置物下降3
},	

bj2_dis1[2][6]={//_n  0:乐扣微前移   1：乐扣大前移
{1500/2,1500/2,1500/2,1500/2,1500/2,1500/2},//乐扣微前移
{6500/2,6500/2,6500/2,6500/2,6500/2,6500/2},//乐扣大前移
},

bj2_dis2[6]={8000/2,8000/2,8000/2,8000/2,8000/2,8000/2},//乐扣大后退

bj2_dis3[3]={8000/2,8000/2,8000/2},//置物大前移
bj2_dis4[3]={8000/2,8000/2,8000/2},//置物大后退

put_bj1_dis1[3]={1300/2,500/2,1000/2},// 3; 放置下降
put_bj1_dis2[3]={1300/2,500/2,1000/2},// 4：放置上升u8 judge[6]={0};

tran[3]={10000/2,6500/2,1500/2};//过渡阶段
u8 Crane_Init_flag=0; 

#endif

/**********函数声明**************/
void Crane_Init (void);//初始化模块
void step1(void);//取物模块
void step2(void);//过渡模块
void step3(void);//置物模块

/**********测试函数**************/
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
//						//下降  前移
//					GPIO_ResetBits(GPIOD, GPIO_Pin_1);
						//上升  后退						
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
//大后退
//
			if(bj2_count<3000)
				{
//						//前移
//					GPIO_ResetBits(GPIOD, GPIO_Pin_3);
						//后退						
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
			
			
////				test();	//测试函数
//		/****起重机初始化****/
//				Crane_Init ();
//		/****取物****/
//				step1();//取物模块
//		/****过渡****/
//				step2();//过渡模块
//		/****置物****/
//				step3();//置物模块
		}
}
/**********初始化动作函数************/
void Crane_Init (void)
{
	
		if(Initflag==0&&startflag==1)
		{
				if(bj1_count<6200)
				{
//						printf("初始化\r\n");
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
				//steppingmotor1(500,pulse22,0);//1M/500=2kHz //起升电机起升-脉冲：5000
		}
		else if(Initflag==1)
		{			
//				Usart_SendByte(DEBUG_USART3,4);//记得改回串口3
				printf ("完成初始化动作\r\n");					
				Servo_SetAngle(Myservo[0]);//舵机复位
				Initflag=2;
		}
}

void step1(void)//取物模块
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
						Usart_SendByte(DEBUG_USART3,6);//记得改回串口3
						printf("放置完毕\r\n");
				}
			}
		}
}

void step2(void)//过渡模块
{
		if(Initflag==3)
		{
				transimt();
		}
}

void step3(void)//置物模块
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
//									Usart_SendByte(DEBUG_USART3,6);//记得改回串口3
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
//									Usart_SendByte(DEBUG_USART3,6);//记得改回串口3
//									printf("开始第二个堆码区识别\r\n");
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

