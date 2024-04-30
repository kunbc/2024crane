#include "logic.h"
#include "TOFSense.h"
#include "stepper_motor.h"
#include "usart.h"
#include "servo.h"
#include "math.h"

//步进电机状态参数
extern motor_state_typedef g_motor1_sta,g_motor2_sta,g_motor3_sta,g_motor4_sta;


//TOF识别判断参数
extern uint8_t servo1_zhuaqv;//1为立即抓，2为在连一个位置抓
extern uint8_t servo2_zhuaqv;//1为立即抓，2为在连一个位置抓
uint8_t qingkuang=0,cejvfankui_flag=0;/*测距结果综合*/
uint8_t ceju_flag=0;/*测距标志位*/

//步进电机参数
extern double crane_init[3],sec1_bj[3],sce2_zhubei[2],sec2o3_qs[2][3],sec2o3_py[4],sce3_zhubei[3];/*步数数组*/
extern __IO  uint32_t g_add_pulse_count[4];    /* 脉冲个数累计*/
uint8_t bj1_jc=0,bj2_jc=0,bj3_jc=0,bj4_jc=0;/*决策标志*/

//动作函数标志位
uint8_t jcdl_flag=0,jcdl2_flag=0,s1_flag=0,s2_flag=0,s1jc_flag=0;

//位置参数、开始参数
extern  int8_t location; //初始参数为0，表示0号位
extern uint8_t action;//初始化移动标志位

//过程标志位
uint8_t Initflag=0,zhengji_flag=0,gc_flag=0;

/**
 * @brief       逻辑参数初始化函数
 * @param       无
 * @retval      void
 */
void canshu_init(void)
{
	gc_flag=0;//过程标志位
	bj1_jc=0;bj2_jc=0;bj3_jc=0;bj4_jc=0;//决策参数
	ceju_flag=0;qingkuang=0;cejvfankui_flag=0;servo1_zhuaqv=0;servo2_zhuaqv=0;//测距参数
	s1_flag=0;s2_flag=0;s1jc_flag=0;jcdl_flag=0;jcdl2_flag=0;//动作函数标志位
}
/**
 * @brief       步进电机步数初始化函数
 * @param       无
 * @retval      无
 */
void bushu_init(void)
{
		g_add_pulse_count[0]=0;
		g_add_pulse_count[1]=0;
		g_add_pulse_count[2]=0;
		g_add_pulse_count[3]=0;
}

/**
 * @brief       整机逻辑函数
 * @param       无
 * @retval      void
 */
void zhengji(void)
{		
/*****************起重机状态初始化********************/	
		if(Initflag==0&&action ==1)
		{
				Crane_Init();
		}
		else if(Initflag==1&&action ==0)
		{			
				Initflag=2;
				printf ("init over***Initflag=%d***zhengji_flag=%d\r\n",Initflag,zhengji_flag);					
		}
/**********************第一阶段***********************/	
		else if(Initflag==2)
		{
				sec1();
		}
		else if(Initflag==3)
		{
				zhengji_flag=2;
				Initflag=4;
				printf ("section1 over***Initflag=%d***zhengji_flag=%d\r\n",Initflag,zhengji_flag);					
		}
/**********************第二阶段准备***********************/	
		else if(Initflag==4)
		{
				section2_zhunbei();	
		}
		else if(Initflag==5)
		{
				zhengji_flag=4;
				Initflag=6;
				printf ("section2 zhunbei over***Initflag=%d***zhengji_flag=%d\r\n",Initflag,zhengji_flag);
		}
/**********************第二阶段****************************/
		else if(Initflag==6)
		{
				sec2o3();
		}
		else if(Initflag==7)
		{
				zhengji_flag=6;
				Initflag=8;
				HAL_Delay(3000);
				printf ("section2 over***Initflag=%d***zhengji_flag=%d\r\n",Initflag,zhengji_flag);
		}
/**********************第三阶段准备***********************/	
		else if(Initflag==8)
		{
				section3_zhunbei();
		}
		else if(Initflag==9)
		{
				zhengji_flag=8;
				Initflag=10;
				printf ("section3 zhunbei over***Initflag=%d***zhengji_flag=%d\r\n",Initflag,zhengji_flag);
		}
/**********************第三阶段****************************/
		else if(Initflag==10)
		{
				sec2o3();
		}
		else if(Initflag==11)
		{
				zhengji_flag=10;
				Initflag=12;
				printf ("section3 over***Initflag=%d***zhengji_flag=%d\r\n",Initflag,zhengji_flag);
		}

}



/***********************************************************工作模块函数*************************************************************/
/**
 * @brief       第二、三工作阶段函数
 * @param       无
 * @retval      void
 */
void sec2o3 (void)
{
/******************************参数初化**************************************/
		if(zhengji_flag==4||zhengji_flag==8)
		{
			canshu_init();
			bushu_init();
			zhengji_flag++;//5//9
		}
/******************************判断阶段**************************************/		
		if(zhengji_flag==5||zhengji_flag==9)
		{
				if((ceju_flag==0 && location==4)||(ceju_flag==0 && location==7)) 
				{//识别20次
						for(int i=0; i<20; i++)
						{
							ceju();					
						} 
						ceju_flag=1;
						cejvfankui_flag=servo1_zhuaqv+servo2_zhuaqv;
						printf("cejvfankui_flag=%d\r\n",cejvfankui_flag);
						switch(cejvfankui_flag)//判断情况
						{
								case 2:
									qingkuang=1;                  //都在内环
									HAL_UART_Transmit(&huart5, (uint8_t *)"1", 1,0xFFFF);//都有，发送1 -> 规划路径
									servo1_zhuaqv=0; 
									servo2_zhuaqv=0; printf("情况=%d\r\n",qingkuang); break;
										
								case 4:
									qingkuang=2;//都在外环
									HAL_UART_Transmit(&huart5, (uint8_t *)"2", 1,0xFFFF);//非都有，发送2 -> 规划路径
									servo1_zhuaqv=0; 
									servo2_zhuaqv=0;printf("情况=%d\r\n",qingkuang); break;
								
								case 3:
									HAL_UART_Transmit(&huart5, (uint8_t *)"2", 1,0xFFFF);//非都有，发送2 -> 规划路径
									if(servo1_zhuaqv==1&&servo2_zhuaqv==2)
									{
											qingkuang=3;//1在内环，2在外环
											servo1_zhuaqv=0; 
											servo2_zhuaqv=0;
									}
									else if(servo1_zhuaqv==2&&servo2_zhuaqv==1)
									{
											qingkuang=4;//2在内环，1在外环
											servo1_zhuaqv=0; 
											servo2_zhuaqv=0;
									}	printf("情况=%d\r\n",qingkuang);break;
									
									default: break;
							}
				}
		}
/******************************工作阶段**************************************/
		if(qingkuang==1)//都在内环
		{
				if(gc_flag==0)
				{
					if (location==4||location==7)
					{
						if(sec2o3_zhuaqv()==OK)
						{
							gc_flag++;//1
							printf("取物完成\r\n");
							HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//抓完大车走		
						}
					}
				}
				else if(gc_flag==1)
				{
						if(da_py()==OK)
						{
							gc_flag++;//2
							printf("大平移完成\r\n");		
						}
				}
				else if(gc_flag==2)
				{
					if(location==6||location==9)
					{
						if(sec2o3_fangwu()==OK)
						{
							gc_flag++;//3
							printf("放物完成\r\n");	
						}
					}
				}
				else if(gc_flag==3)
				{
					gc_flag++;//4
					printf("大车走\r\n");
					Initflag++;	//7 //9
					HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//放完大车走
				}
		}
		else if(qingkuang==2)//都在外环
		{
				if(gc_flag==0)
				{
						if(nei_py()==OK)
						{
								printf("内平移完成\r\n");
								gc_flag++;//1		
								HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//大车走，等待抓取										
						}
				}
				else if(gc_flag==1)
				{
					if (location==5||location==8)
					{
						if(sec2o3_zhuaqv()==OK)
						{
								printf("取物完成\r\n");
								gc_flag++;//2
								HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//抓完大车走		
						}
					}
				}
				else if(gc_flag==2)
				{
						if(wai_py()==OK)
						{
								printf("外平移完成\r\n");
								gc_flag++;//3
//								printf("////////jcdl_flag=%d\r\n",jcdl_flag);	
						}
				}
				else if(gc_flag==3)
				{
					if (location==6||location==9)
					{
						if(sec2o3_fangwu()==OK)
						{
								printf("放物完成\r\n");
								gc_flag++;//4	
						}
					}
				}
				else if(gc_flag==4)
				{
					gc_flag++;//5
					printf("大车走\r\n");
					Initflag++;	//7 //9
					HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//放完大车走		
				}							
		}
		else if(qingkuang==3||qingkuang==4)//1在内环，2在外环   //2在内环，1在外环
		{
				if(gc_flag==0)
				{
							if (location==4||location==7)
							{
										switch(qingkuang)
										{
											case 3:
													if(sec2o3_zhuaqv1()==OK)
													{
															gc_flag++;//1
															HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//抓完大车走
															printf("servo1 抓取完成\r\n");
													} break;
											case 4:
													if(sec2o3_zhuaqv2()==OK)
													{
															gc_flag++;//1
															HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//抓完大车走
															printf("servo2 抓取完成\r\n");									
													} break;
										}
							}
				}
				else if(gc_flag==1)
				{	
						if(nei_py()==OK)
						{
								gc_flag++;//2
								printf("内平移完成\r\n");
						}
				}
				else if(gc_flag==2)
				{
					if (location==5||location==8)
					{
						switch(qingkuang)
						{
							case 3:
									if(sec2o3_zhuaqv2()==OK)
									{
											gc_flag++;//3
											HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//抓完大车走
											printf("servo2 抓取完成\r\n");
									} break;
							case 4:
									if(sec2o3_zhuaqv1()==OK)
									{
											gc_flag++;//3
											HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//抓完大车走
											printf("servo1 抓取完成\r\n");
									} break;
							default: break;
						}
					}
				}
				else if(gc_flag==3)
				{
						if(wai_py()==OK)
						{
								gc_flag++;//4
								printf("外平移完成\r\n");
						}
						if (s1_flag==2&&s2_flag==2&&jcdl_flag==0)
						{
							jcdl_flag=2;//置2后，标志着放物
						}
				}
				else if(gc_flag==4)
				{
					if (location==6||location==9)
					{
						if(sec2o3_fangwu()==OK)
						{
								gc_flag++;//5
								printf("放物完成\r\n");
						}							
					}
				}
				else if(gc_flag==5)
				{
					printf("大车走\r\n");
					gc_flag++;//6
					Initflag++;//7 //9
					HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//放完大车走
				}
		}
}


/**
 * @brief       第三阶段准备函数
 * @param      	无
 * @retval      void
 */
void section3_zhunbei(void)
{
		if(zhengji_flag==6)
		{
			canshu_init();
			bushu_init();
			zhengji_flag=7;
		}
		if(zhengji_flag==7)
		{
				if (bj1_jc==0&&bj2_jc==0&&bj3_jc==0 && bj4_jc==0)
				{
						g_add_pulse_count[2]=0;
						g_add_pulse_count[3]=0;

						stepmotor_move_rel(V_START,fast_42END,fast_42ACTIME,fast_42DETIME,sce3_zhubei[1]*SPR,STEPPER_MOTOR_3);
						stepmotor_move_rel(V_START,fast_42END,fast_42ACTIME,fast_42DETIME,sce3_zhubei[2]*SPR,STEPPER_MOTOR_4);

						stepper_start(STEPPER_MOTOR_3);
						stepper_start(STEPPER_MOTOR_4);
						bj3_jc=1;
						bj4_jc=1;						
				}
				else if(bj1_jc==0 && bj2_jc==0&&bj3_jc==2 && bj4_jc==2)
				{
						g_add_pulse_count[0]=0;
						g_add_pulse_count[1]=0;
						
						stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,sce3_zhubei[0]*SPR,STEPPER_MOTOR_1);
						stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,sce3_zhubei[0]*SPR,STEPPER_MOTOR_2);
					
						stepper_start(STEPPER_MOTOR_1);
						stepper_start(STEPPER_MOTOR_2);
						
						bj1_jc=1;
						bj2_jc=1;
				}
				if(g_motor3_sta==STATE_IDLE&&g_motor4_sta==STATE_IDLE&&bj3_jc==1&&bj4_jc==1)
				{
						bj3_jc=2;
						bj4_jc=2;
				}
				else if(g_motor1_sta==STATE_IDLE&&g_motor2_sta==STATE_IDLE&&bj1_jc==1&&bj2_jc==1)
				{
						bj1_jc=2;
						bj2_jc=2;
						Initflag=9;					
				}				
		}
}

/**
 * @brief       第二阶段准备函数
 * @param       无
 * @retval      void
 */
void section2_zhunbei(void)
{
		if(zhengji_flag==2)
		{
			canshu_init();
			bushu_init();
			zhengji_flag=3;
		}
		if(zhengji_flag==3)
		{
			if(bj1_jc==0&&bj3_jc==0)
			{
				g_add_pulse_count[2]=0;
				stepmotor_move_rel(V_START,fast_42END,fast_42ACTIME,fast_42DETIME,sce2_zhubei[1]*SPR,STEPPER_MOTOR_3);
				bj3_jc=1;
				stepper_start(STEPPER_MOTOR_3);		
			}
			if(bj1_jc==0&&bj3_jc==1)
			{
				g_add_pulse_count[0]=0;
				stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,sce2_zhubei[0]*SPR,STEPPER_MOTOR_1);
				stepper_start(STEPPER_MOTOR_1);
				bj1_jc=1;

			}
			if(g_motor1_sta==STATE_IDLE&&g_motor3_sta==STATE_IDLE)
			{
				Initflag=5;
				bj1_jc=2;
				bj3_jc=2;
			}
		}
}
/**
 * @brief       第一阶段工作函数
 * @param       无
 * @retval      void
 */
void sec1 (void)
{
	if(zhengji_flag==0)
	{
		canshu_init();
		bushu_init();
		zhengji_flag=1;
	}
	if(zhengji_flag==1)
	{
		if(ceju_flag==0 && location==1) 
		{
			for(int n=0;n<20;n++)
			{
				ceju();					
			} 
			ceju_flag=1;//servo1_zhuaqv=1:外环,servo2_zhuaqv=2:内环
			printf("servo1_zhuaqv=%d\r\n",servo1_zhuaqv);
			if(servo1_zhuaqv==1) HAL_UART_Transmit(&huart5, (uint8_t *)"1", 1,0xFFFF);//有，发送1，—>规划路径
			else if(servo1_zhuaqv==2) HAL_UART_Transmit(&huart5, (uint8_t *)"2", 1,0xFFFF);//无，发送2，->规划路径
		}
		if(servo1_zhuaqv==1)//物品在外环
		{
			if(gc_flag==0&&location==1)//未开始动作
			{
				if(sec1_zhuaqv1()==OK)
				{
					gc_flag++;//1
					HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//抓完大车走
					printf("取物完成\r\n");	
				}
			}
			else if(gc_flag==1&&location==3)
			{
				if(sec1_fangwu1()==OK)
				{
					gc_flag++;//2
					printf("放物完成\r\n");	
				}
			}
			else if (gc_flag==2)
			{
//				printf("起升完成\r\n");
				Initflag=3;//第一阶段结束
				HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//放完大车走	
			}
		}
		else if (servo1_zhuaqv==2)//物品在内环
		{
			if(gc_flag==0&&location==1)//行进
			{
				gc_flag++;//1
				HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//大车走，等待抓取
			}
			else if(gc_flag==1&&location==2)//抓取动作
			{
				if(sec1_zhuaqv1()==OK)
				{
					gc_flag++;//2
					HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//抓完大车走
					printf("取物完成\r\n");	
				}
			}
			else if(gc_flag==2&&location==3)
			{
				if(sec1_fangwu1()==OK)//放物动作
				{
					gc_flag++;//3
					printf("放物完成\r\n");	
				}
			}
			else if (gc_flag==3)
			{
				printf("大车走\r\n");
				Initflag=3;//第一阶段结束
				HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//放完大车走	
			}
		}
	}
}


/**
 * @brief       起重机初始化运动函数
 * @param       无
 * @retval      void
 */
void Crane_Init(void)
{
			if(bj1_jc==0&&bj2_jc==0&&bj3_jc==0)
			{
					g_add_pulse_count[0]=0;
					g_add_pulse_count[1]=0;
					g_add_pulse_count[2]=0;
					
					stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,crane_init[0]*SPR,STEPPER_MOTOR_1);
					stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,crane_init[1]*SPR,STEPPER_MOTOR_2);
					stepmotor_move_rel(V_START,fast_42END,fast_42ACTIME,fast_42DETIME,crane_init[2]*SPR,STEPPER_MOTOR_3);

					bj1_jc=1;
					bj2_jc=1;
					bj3_jc=1;
			}
			if(	bj1_jc==1 && bj2_jc==1)
			{
					stepper_start(STEPPER_MOTOR_1);
					stepper_start(STEPPER_MOTOR_2);
					bj1_jc=2;
					bj2_jc=2;
			}
			if(g_motor1_sta==STATE_IDLE && g_motor2_sta==STATE_IDLE)
			{
					if(bj3_jc==1)
					{
							steer(Z_sec1,SERVO_ALL);//舵机张开
							HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//初始化完成，大车行进
							HAL_Delay(1000);
						
							stepper_start(STEPPER_MOTOR_3);
							bj3_jc=2;
					}
					if(g_motor3_sta==STATE_IDLE)
					{
							Initflag=1;
							action =0;
					}
			}	
}

/****************************************************/


/********************************************************动作模块函数*****************************************************************/
/**
 * @brief       第一阶段1号夹爪取物动作函数
 * @param       无
 * @retval      OK：成功；NONE：失败
 */
int sec1_zhuaqv1(void)
{
	if(s1jc_flag==0)//确保没抓
	{
		bj1_jc=0;
		s1jc_flag=1;
	}
	if (bj1_jc==0&&s1jc_flag==1)
	{
		g_add_pulse_count[0]=0;
		stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,sec1_bj[0]*SPR,STEPPER_MOTOR_1);
	
		stepper_start(STEPPER_MOTOR_1);
		bj1_jc=1;//1//不会再进
//				printf("1下\r\n");
	}
	else if (bj1_jc==2&&s1jc_flag==1)
	{
		steer(BIHE,SERVO_1);//舵机抓
		HAL_Delay(1000);
		g_add_pulse_count[0]=0;
		stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,sec1_bj[1]*SPR,STEPPER_MOTOR_1);
	
		stepper_start(STEPPER_MOTOR_1);
		bj1_jc=3;//3//不会再进
	}
	if(g_motor1_sta==STATE_IDLE&&s1jc_flag==1)
	{
		bj1_jc++;//2//4
		if (bj1_jc==4)
		{
			s1jc_flag=2;
			return OK;		
		}	
	}
	return NONE;
}

/**
 * @brief       第一阶段1号夹爪放物动作函数
 * @param       无
 * @retval      OK：成功；NONE：失败
 */
int sec1_fangwu1(void)
{
	if(s1jc_flag==2)//确保已经抓了
	{
		bj1_jc=0;
		s1jc_flag=3;
	}
	if(bj1_jc==0&&s1jc_flag==3)//要放先降
	{
			g_add_pulse_count[0]=0;
			stepmotor_move_rel(V_START,slow_57END,slow_57ACTIME,slow_57DETIME,sec1_bj[2]*SPR,STEPPER_MOTOR_1);
			stepper_start(STEPPER_MOTOR_1);
			bj1_jc=1;//1
	}
//	else if(bj1_jc==2&&s1jc_flag==3)//放完起升
//	{
//			steer(ZHANGKAI,SERVO_1);//舵机放
//			HAL_Delay(2000);
//			g_add_pulse_count[0]=0;
//			stepmotor_move_rel(V_START,100,0.01,0.01,sec1_bj[3]*SPR,STEPPER_MOTOR_1);
//			stepper_start(STEPPER_MOTOR_1);
//			bj1_jc++;//3
////				printf("2起\r\n");
//	}
	if(g_motor1_sta==STATE_IDLE&&s1jc_flag==3)
	{
			steer(Z_sec1,SERVO_1);//舵机放
			HAL_Delay(1000);
			bj1_jc++;//2
		
		if(bj1_jc==2)
		{
			s1jc_flag=4;
			return OK;
		}
	}
	return NONE;
}

/**
 * @brief       第二、三阶段一号夹爪取物动作函数
 * @param       无
 * @retval      OK：成功；NONE：失败
 */
int sec2o3_zhuaqv1(void)
{
		if(s1_flag==0)//确保没抓
		{
				bj1_jc=0;
				s1_flag=1;//确保要放
		}
		if(bj1_jc==0&&s1_flag==1)//未抓先降
		{
				g_add_pulse_count[0]=0;
				stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,sec2o3_qs[0][0]*SPR,STEPPER_MOTOR_1);
			
				stepper_start(STEPPER_MOTOR_1);
				bj1_jc=1;//1//不会再进
//				printf("1下\r\n");
		}
		else if(bj1_jc==2&&s1_flag==1)//抓完起升
		{
				steer(BIHE,SERVO_1);//舵机抓
				HAL_Delay(1000);
			
				g_add_pulse_count[0]=0;
				stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,sec2o3_qs[0][1]*SPR,STEPPER_MOTOR_1);
			
				stepper_start(STEPPER_MOTOR_1);
				bj1_jc=3;//3//不会再进
		}
		if(g_motor1_sta==STATE_IDLE&&s1_flag==1)
		{
				bj1_jc++;//2//4

			if(bj1_jc==4)
			{
					s1_flag=2;
					return OK;
			}
		}
		return NONE;
}

/**
 * @brief       第二、三阶段2号夹爪取物动作函数
 * @param       无
 * @retval      OK：成功；NONE：失败
 */
int sec2o3_zhuaqv2(void)
{
		if(s2_flag==0)//确保没抓
		{
				bj2_jc=0;
				s2_flag=1;//确保要放
		}
		if(bj2_jc==0&&s2_flag==1)//未抓先降
		{
				g_add_pulse_count[0]=0;
				stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,sec2o3_qs[1][0]*SPR,STEPPER_MOTOR_2);
			
				stepper_start(STEPPER_MOTOR_2);
				bj2_jc=1;//1//不会再进
//				printf("1下\r\n");
		}
		else if(bj2_jc==2&&s2_flag==1)//抓完起升
		{
				steer(BIHE,SERVO_2);//舵机抓
//				printf("*****************/duoji\r\n");
				HAL_Delay(1000);
			
				g_add_pulse_count[0]=0;
				stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,sec2o3_qs[1][1]*SPR,STEPPER_MOTOR_2);
			
				stepper_start(STEPPER_MOTOR_2);
				bj2_jc=3;//3//不会再进
		}
		if(g_motor2_sta==STATE_IDLE&&s2_flag==1)
		{
				bj2_jc++;//2//4

			if(bj2_jc==4)
			{
					s2_flag=2;
					return OK;
			}
		}
		return NONE;
}

/**
 * @brief       第二、三阶段共同取物动作函数
 * @param       无
 * @retval      OK：成功；NONE：失败
 */
int sec2o3_zhuaqv(void)
{
		if(jcdl_flag==0)//确保没抓
		{
				bj1_jc=0;
				bj2_jc=0;
				jcdl_flag=1;//确保要放
		}
		if(bj1_jc==0&&bj2_jc==0&&jcdl_flag==1)//未抓先降
		{
				g_add_pulse_count[0]=0;
				stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,sec2o3_qs[0][0]*SPR,STEPPER_MOTOR_1);
			
				g_add_pulse_count[1]=0;
				stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,sec2o3_qs[1][0]*SPR,STEPPER_MOTOR_2);
			
				stepper_start(STEPPER_MOTOR_1);
				stepper_start(STEPPER_MOTOR_2);
				bj1_jc=1;
				bj2_jc=1;//1//不会再进
//				printf("1下\r\n");
		}
		else if(bj1_jc==2&&bj2_jc==2&&jcdl_flag==1)//抓完起升
		{
				steer(BIHE,SERVO_ALL);//舵机抓
				HAL_Delay(1000);
			
				g_add_pulse_count[0]=0;
				stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,sec2o3_qs[0][1]*SPR,STEPPER_MOTOR_1);
			
				g_add_pulse_count[1]=0;
				stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,sec2o3_qs[1][1]*SPR,STEPPER_MOTOR_2);
			
				stepper_start(STEPPER_MOTOR_1);
				stepper_start(STEPPER_MOTOR_2);
				bj1_jc=3;
				bj2_jc=3;//3//不会再进
//				printf("2起\r\n");
		}
		if(g_motor1_sta==STATE_IDLE&&g_motor2_sta==STATE_IDLE&&jcdl_flag==1)
		{
				bj1_jc++;//2//4
				bj2_jc++;//2//4
//				printf("AAAAAAbj1_jc=%d",bj1_jc);
			if(bj1_jc==4&&bj2_jc==4)
			{
					jcdl_flag=2;
					return OK;
			}
		}
		return NONE;
}

/**
 * @brief       第二、三阶段共同放物动作函数
 * @param       无
 * @retval      OK：成功；NONE：失败
 */
int sec2o3_fangwu(void)
{
		if(jcdl_flag==2)//确保已经抓过了
		{
				bj1_jc=0;
				bj2_jc=0;
				jcdl_flag=3;//确保放了
		}
		else if(s1_flag==2&&s2_flag==2)
		{
			bj1_jc=0;
			bj2_jc=0;
			s1_flag=3;//确保放了
			s2_flag=3;//确保放了
			jcdl_flag=3;//变量转换
		}
		if(bj1_jc==0&&bj2_jc==0&&jcdl_flag==3)//要放先降
		{
				g_add_pulse_count[0]=0;
				stepmotor_move_rel(V_START,slow_57END,slow_57ACTIME,slow_57DETIME,sec2o3_qs[0][2]*SPR,STEPPER_MOTOR_1);
			
				g_add_pulse_count[1]=0;
				stepmotor_move_rel(V_START,slow_57END,slow_57ACTIME,slow_57DETIME,sec2o3_qs[1][2]*SPR,STEPPER_MOTOR_2);
			
				stepper_start(STEPPER_MOTOR_2);	
				stepper_start(STEPPER_MOTOR_1);

				bj1_jc++;
				bj2_jc++;//1//不会再进
		}
//		else if(bj1_jc==2&&bj2_jc==2&&jcdl_flag==3)//放完起升
//		{
//				steer(ZHANGKAI,SERVO_ALL);//舵机放
//				HAL_Delay(1000);
//			
//				g_add_pulse_count[0]=0;
//				stepmotor_move_rel(V_START,150,0.01,0.01,sec2o3_qs[0][3]*SPR,STEPPER_MOTOR_1);
//			
//				g_add_pulse_count[1]=0;
//				stepmotor_move_rel(V_START,150,0.01,0.01,sec2o3_qs[1][3]*SPR,STEPPER_MOTOR_2);
//				
//				stepper_start(STEPPER_MOTOR_1);
//				stepper_start(STEPPER_MOTOR_2);
//				bj1_jc++;
//				bj2_jc++;//3//不会再进
////				printf("2起\r\n");

//		}

		if(g_motor1_sta==STATE_IDLE&&g_motor2_sta==STATE_IDLE&&jcdl_flag==3)
		{
				steer(ZHANGKAI,SERVO_ALL);//舵机放
				HAL_Delay(1000);
			
				bj1_jc++;//2
				bj2_jc++;//2
			
				if(bj1_jc==2&&bj2_jc==2)
				{
						jcdl_flag=4;
						return OK;
				}
		}
		return NONE;
}

/**
 * @brief       内平移动作函数
 * @param       无
 * @retval      OK：成功；NONE：失败
 */
int nei_py(void)
{
		if(jcdl2_flag==0)
		{
				bj3_jc=0;
				bj4_jc=0;
				jcdl2_flag=1;
		}
		if(bj3_jc==0&&bj4_jc==0&&jcdl2_flag==1)//抓完可以移动
		{
				g_add_pulse_count[2]=0;
				g_add_pulse_count[3]=0;

				stepmotor_move_rel(V_START,fast_42END,fast_42ACTIME,fast_42DETIME,sec2o3_py[0]*SPR,STEPPER_MOTOR_3);
				stepmotor_move_rel(V_START,fast_42END,fast_42ACTIME,fast_42DETIME,sec2o3_py[0]*SPR,STEPPER_MOTOR_4);
				
				bj3_jc++;//1//不会再进
				bj4_jc++;//1//不会再进
				
				stepper_start(STEPPER_MOTOR_3);
				stepper_start(STEPPER_MOTOR_4);

		}
		if(g_motor4_sta==STATE_IDLE && g_motor3_sta==STATE_IDLE && jcdl2_flag==1)
		{	
				printf("*8888*\r\n");
				bj3_jc++;//2
				bj4_jc++;//2
				if(bj3_jc==2&&bj4_jc==2) 
				{
						jcdl2_flag=2;
						printf("*99999*\r\n");
						return OK;
				}
		}
		return NONE;
}

/**
 * @brief       外平移动作函数
 * @param       无
 * @retval      OK：成功；NONE：失败
 */
int wai_py(void)
{
		if(jcdl2_flag==2)
		{
				bj3_jc=0;
				bj4_jc=0;
				jcdl2_flag=3;
		}
		if(bj3_jc==0&&bj4_jc==0&&jcdl2_flag==3)//抓完可以移动
		{
				g_add_pulse_count[2]=0;
				stepmotor_move_rel(V_START,slow_42END,slow_42ACTIME,slow_42DETIME,sec2o3_py[1]*SPR,STEPPER_MOTOR_3);
				bj3_jc++;//1//不会再进
				
				g_add_pulse_count[3]=0;
				stepmotor_move_rel(V_START,slow_42END,slow_42ACTIME,slow_42DETIME,sec2o3_py[1]*SPR,STEPPER_MOTOR_4);
				bj4_jc++;//1//不会再进
				
				stepper_start(STEPPER_MOTOR_3);
				stepper_start(STEPPER_MOTOR_4);
		}
		
		if(g_motor4_sta==STATE_IDLE&&g_motor3_sta==STATE_IDLE&&jcdl2_flag==3)
		{
				bj3_jc++;//2
				bj4_jc++;//2
			
				if(bj3_jc==2&&bj4_jc==2) 
				{
						jcdl2_flag=4;
						return OK;
				}	
		}
		return NONE;
}

/**
 * @brief       大平移动作函数
 * @param       无
 * @retval      OK：成功；NONE：失败
 */
int da_py(void)
{
		if(jcdl2_flag==0)
		{
				bj3_jc=0;
				bj4_jc=0;
				jcdl2_flag=1;
		}
		if (jcdl2_flag==1)
		{
			if(bj3_jc==0&&bj4_jc==0)//抓完可以移动
			{
					g_add_pulse_count[2]=0;
					stepmotor_move_rel(V_START,fast_42END,fast_42ACTIME,fast_42DETIME,sec2o3_py[2]*SPR,STEPPER_MOTOR_3);
					bj3_jc++;//1//不会再进
					
					g_add_pulse_count[3]=0;
					stepmotor_move_rel(V_START,fast_42END,fast_42ACTIME,fast_42DETIME,sec2o3_py[3]*SPR,STEPPER_MOTOR_4);
					bj4_jc++;//1//不会再进
					
					stepper_start(STEPPER_MOTOR_3);
					stepper_start(STEPPER_MOTOR_4);

			}
			
			if(g_motor4_sta==STATE_IDLE&&g_motor3_sta==STATE_IDLE)
			{
					bj3_jc++;//2
					bj4_jc++;//2
				
					if(bj3_jc==2&&bj4_jc==2) 
						{
								jcdl2_flag=2;		
								return OK;
						}
			}
		}
		
		return NONE;
}
/***********************************************************/








