#include "logic.h"
#include "TOFSense.h"
#include "stepper_motor.h"
#include "usart.h"
#include "servo.h"
#include "math.h"

//�������״̬����
extern motor_state_typedef g_motor1_sta,g_motor2_sta,g_motor3_sta,g_motor4_sta;


//TOFʶ���жϲ���
extern uint8_t servo1_zhuaqv;//1Ϊ����ץ��2Ϊ����һ��λ��ץ
extern uint8_t servo2_zhuaqv;//1Ϊ����ץ��2Ϊ����һ��λ��ץ
uint8_t qingkuang=0,cejvfankui_flag=0;/*������ۺ�*/
uint8_t ceju_flag=0;/*����־λ*/

//�����������
extern double crane_init[3],sec1_bj[3],sce2_zhubei[2],sec2o3_qs[2][3],sec2o3_py[4],sce3_zhubei[3];/*��������*/
extern __IO  uint32_t g_add_pulse_count[4];    /* ��������ۼ�*/
uint8_t bj1_jc=0,bj2_jc=0,bj3_jc=0,bj4_jc=0;/*���߱�־*/

//����������־λ
uint8_t jcdl_flag=0,jcdl2_flag=0,s1_flag=0,s2_flag=0,s1jc_flag=0;

//λ�ò�������ʼ����
extern  int8_t location; //��ʼ����Ϊ0����ʾ0��λ
extern uint8_t action;//��ʼ���ƶ���־λ

//���̱�־λ
uint8_t Initflag=0,zhengji_flag=0,gc_flag=0;

/**
 * @brief       �߼�������ʼ������
 * @param       ��
 * @retval      void
 */
void canshu_init(void)
{
	gc_flag=0;//���̱�־λ
	bj1_jc=0;bj2_jc=0;bj3_jc=0;bj4_jc=0;//���߲���
	ceju_flag=0;qingkuang=0;cejvfankui_flag=0;servo1_zhuaqv=0;servo2_zhuaqv=0;//������
	s1_flag=0;s2_flag=0;s1jc_flag=0;jcdl_flag=0;jcdl2_flag=0;//����������־λ
}
/**
 * @brief       �������������ʼ������
 * @param       ��
 * @retval      ��
 */
void bushu_init(void)
{
		g_add_pulse_count[0]=0;
		g_add_pulse_count[1]=0;
		g_add_pulse_count[2]=0;
		g_add_pulse_count[3]=0;
}

/**
 * @brief       �����߼�����
 * @param       ��
 * @retval      void
 */
void zhengji(void)
{		
/*****************���ػ�״̬��ʼ��********************/	
		if(Initflag==0&&action ==1)
		{
				Crane_Init();
		}
		else if(Initflag==1&&action ==0)
		{			
				Initflag=2;
				printf ("init over***Initflag=%d***zhengji_flag=%d\r\n",Initflag,zhengji_flag);					
		}
/**********************��һ�׶�***********************/	
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
/**********************�ڶ��׶�׼��***********************/	
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
/**********************�ڶ��׶�****************************/
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
/**********************�����׶�׼��***********************/	
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
/**********************�����׶�****************************/
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



/***********************************************************����ģ�麯��*************************************************************/
/**
 * @brief       �ڶ����������׶κ���
 * @param       ��
 * @retval      void
 */
void sec2o3 (void)
{
/******************************��������**************************************/
		if(zhengji_flag==4||zhengji_flag==8)
		{
			canshu_init();
			bushu_init();
			zhengji_flag++;//5//9
		}
/******************************�жϽ׶�**************************************/		
		if(zhengji_flag==5||zhengji_flag==9)
		{
				if((ceju_flag==0 && location==4)||(ceju_flag==0 && location==7)) 
				{//ʶ��20��
						for(int i=0; i<20; i++)
						{
							ceju();					
						} 
						ceju_flag=1;
						cejvfankui_flag=servo1_zhuaqv+servo2_zhuaqv;
						printf("cejvfankui_flag=%d\r\n",cejvfankui_flag);
						switch(cejvfankui_flag)//�ж����
						{
								case 2:
									qingkuang=1;                  //�����ڻ�
									HAL_UART_Transmit(&huart5, (uint8_t *)"1", 1,0xFFFF);//���У�����1 -> �滮·��
									servo1_zhuaqv=0; 
									servo2_zhuaqv=0; printf("���=%d\r\n",qingkuang); break;
										
								case 4:
									qingkuang=2;//�����⻷
									HAL_UART_Transmit(&huart5, (uint8_t *)"2", 1,0xFFFF);//�Ƕ��У�����2 -> �滮·��
									servo1_zhuaqv=0; 
									servo2_zhuaqv=0;printf("���=%d\r\n",qingkuang); break;
								
								case 3:
									HAL_UART_Transmit(&huart5, (uint8_t *)"2", 1,0xFFFF);//�Ƕ��У�����2 -> �滮·��
									if(servo1_zhuaqv==1&&servo2_zhuaqv==2)
									{
											qingkuang=3;//1���ڻ���2���⻷
											servo1_zhuaqv=0; 
											servo2_zhuaqv=0;
									}
									else if(servo1_zhuaqv==2&&servo2_zhuaqv==1)
									{
											qingkuang=4;//2���ڻ���1���⻷
											servo1_zhuaqv=0; 
											servo2_zhuaqv=0;
									}	printf("���=%d\r\n",qingkuang);break;
									
									default: break;
							}
				}
		}
/******************************�����׶�**************************************/
		if(qingkuang==1)//�����ڻ�
		{
				if(gc_flag==0)
				{
					if (location==4||location==7)
					{
						if(sec2o3_zhuaqv()==OK)
						{
							gc_flag++;//1
							printf("ȡ�����\r\n");
							HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//ץ�����		
						}
					}
				}
				else if(gc_flag==1)
				{
						if(da_py()==OK)
						{
							gc_flag++;//2
							printf("��ƽ�����\r\n");		
						}
				}
				else if(gc_flag==2)
				{
					if(location==6||location==9)
					{
						if(sec2o3_fangwu()==OK)
						{
							gc_flag++;//3
							printf("�������\r\n");	
						}
					}
				}
				else if(gc_flag==3)
				{
					gc_flag++;//4
					printf("����\r\n");
					Initflag++;	//7 //9
					HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//�������
				}
		}
		else if(qingkuang==2)//�����⻷
		{
				if(gc_flag==0)
				{
						if(nei_py()==OK)
						{
								printf("��ƽ�����\r\n");
								gc_flag++;//1		
								HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//���ߣ��ȴ�ץȡ										
						}
				}
				else if(gc_flag==1)
				{
					if (location==5||location==8)
					{
						if(sec2o3_zhuaqv()==OK)
						{
								printf("ȡ�����\r\n");
								gc_flag++;//2
								HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//ץ�����		
						}
					}
				}
				else if(gc_flag==2)
				{
						if(wai_py()==OK)
						{
								printf("��ƽ�����\r\n");
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
								printf("�������\r\n");
								gc_flag++;//4	
						}
					}
				}
				else if(gc_flag==4)
				{
					gc_flag++;//5
					printf("����\r\n");
					Initflag++;	//7 //9
					HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//�������		
				}							
		}
		else if(qingkuang==3||qingkuang==4)//1���ڻ���2���⻷   //2���ڻ���1���⻷
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
															HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//ץ�����
															printf("servo1 ץȡ���\r\n");
													} break;
											case 4:
													if(sec2o3_zhuaqv2()==OK)
													{
															gc_flag++;//1
															HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//ץ�����
															printf("servo2 ץȡ���\r\n");									
													} break;
										}
							}
				}
				else if(gc_flag==1)
				{	
						if(nei_py()==OK)
						{
								gc_flag++;//2
								printf("��ƽ�����\r\n");
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
											HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//ץ�����
											printf("servo2 ץȡ���\r\n");
									} break;
							case 4:
									if(sec2o3_zhuaqv1()==OK)
									{
											gc_flag++;//3
											HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//ץ�����
											printf("servo1 ץȡ���\r\n");
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
								printf("��ƽ�����\r\n");
						}
						if (s1_flag==2&&s2_flag==2&&jcdl_flag==0)
						{
							jcdl_flag=2;//��2�󣬱�־�ŷ���
						}
				}
				else if(gc_flag==4)
				{
					if (location==6||location==9)
					{
						if(sec2o3_fangwu()==OK)
						{
								gc_flag++;//5
								printf("�������\r\n");
						}							
					}
				}
				else if(gc_flag==5)
				{
					printf("����\r\n");
					gc_flag++;//6
					Initflag++;//7 //9
					HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//�������
				}
		}
}


/**
 * @brief       �����׶�׼������
 * @param      	��
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
 * @brief       �ڶ��׶�׼������
 * @param       ��
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
 * @brief       ��һ�׶ι�������
 * @param       ��
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
			ceju_flag=1;//servo1_zhuaqv=1:�⻷,servo2_zhuaqv=2:�ڻ�
			printf("servo1_zhuaqv=%d\r\n",servo1_zhuaqv);
			if(servo1_zhuaqv==1) HAL_UART_Transmit(&huart5, (uint8_t *)"1", 1,0xFFFF);//�У�����1����>�滮·��
			else if(servo1_zhuaqv==2) HAL_UART_Transmit(&huart5, (uint8_t *)"2", 1,0xFFFF);//�ޣ�����2��->�滮·��
		}
		if(servo1_zhuaqv==1)//��Ʒ���⻷
		{
			if(gc_flag==0&&location==1)//δ��ʼ����
			{
				if(sec1_zhuaqv1()==OK)
				{
					gc_flag++;//1
					HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//ץ�����
					printf("ȡ�����\r\n");	
				}
			}
			else if(gc_flag==1&&location==3)
			{
				if(sec1_fangwu1()==OK)
				{
					gc_flag++;//2
					printf("�������\r\n");	
				}
			}
			else if (gc_flag==2)
			{
//				printf("�������\r\n");
				Initflag=3;//��һ�׶ν���
				HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//�������	
			}
		}
		else if (servo1_zhuaqv==2)//��Ʒ���ڻ�
		{
			if(gc_flag==0&&location==1)//�н�
			{
				gc_flag++;//1
				HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//���ߣ��ȴ�ץȡ
			}
			else if(gc_flag==1&&location==2)//ץȡ����
			{
				if(sec1_zhuaqv1()==OK)
				{
					gc_flag++;//2
					HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//ץ�����
					printf("ȡ�����\r\n");	
				}
			}
			else if(gc_flag==2&&location==3)
			{
				if(sec1_fangwu1()==OK)//���ﶯ��
				{
					gc_flag++;//3
					printf("�������\r\n");	
				}
			}
			else if (gc_flag==3)
			{
				printf("����\r\n");
				Initflag=3;//��һ�׶ν���
				HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//�������	
			}
		}
	}
}


/**
 * @brief       ���ػ���ʼ���˶�����
 * @param       ��
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
							steer(Z_sec1,SERVO_ALL);//����ſ�
							HAL_UART_Transmit(&huart5, (uint8_t *)"3", 1,0xFFFF);//��ʼ����ɣ����н�
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


/********************************************************����ģ�麯��*****************************************************************/
/**
 * @brief       ��һ�׶�1�ż�צȡ�ﶯ������
 * @param       ��
 * @retval      OK���ɹ���NONE��ʧ��
 */
int sec1_zhuaqv1(void)
{
	if(s1jc_flag==0)//ȷ��ûץ
	{
		bj1_jc=0;
		s1jc_flag=1;
	}
	if (bj1_jc==0&&s1jc_flag==1)
	{
		g_add_pulse_count[0]=0;
		stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,sec1_bj[0]*SPR,STEPPER_MOTOR_1);
	
		stepper_start(STEPPER_MOTOR_1);
		bj1_jc=1;//1//�����ٽ�
//				printf("1��\r\n");
	}
	else if (bj1_jc==2&&s1jc_flag==1)
	{
		steer(BIHE,SERVO_1);//���ץ
		HAL_Delay(1000);
		g_add_pulse_count[0]=0;
		stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,sec1_bj[1]*SPR,STEPPER_MOTOR_1);
	
		stepper_start(STEPPER_MOTOR_1);
		bj1_jc=3;//3//�����ٽ�
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
 * @brief       ��һ�׶�1�ż�צ���ﶯ������
 * @param       ��
 * @retval      OK���ɹ���NONE��ʧ��
 */
int sec1_fangwu1(void)
{
	if(s1jc_flag==2)//ȷ���Ѿ�ץ��
	{
		bj1_jc=0;
		s1jc_flag=3;
	}
	if(bj1_jc==0&&s1jc_flag==3)//Ҫ���Ƚ�
	{
			g_add_pulse_count[0]=0;
			stepmotor_move_rel(V_START,slow_57END,slow_57ACTIME,slow_57DETIME,sec1_bj[2]*SPR,STEPPER_MOTOR_1);
			stepper_start(STEPPER_MOTOR_1);
			bj1_jc=1;//1
	}
//	else if(bj1_jc==2&&s1jc_flag==3)//��������
//	{
//			steer(ZHANGKAI,SERVO_1);//�����
//			HAL_Delay(2000);
//			g_add_pulse_count[0]=0;
//			stepmotor_move_rel(V_START,100,0.01,0.01,sec1_bj[3]*SPR,STEPPER_MOTOR_1);
//			stepper_start(STEPPER_MOTOR_1);
//			bj1_jc++;//3
////				printf("2��\r\n");
//	}
	if(g_motor1_sta==STATE_IDLE&&s1jc_flag==3)
	{
			steer(Z_sec1,SERVO_1);//�����
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
 * @brief       �ڶ������׶�һ�ż�צȡ�ﶯ������
 * @param       ��
 * @retval      OK���ɹ���NONE��ʧ��
 */
int sec2o3_zhuaqv1(void)
{
		if(s1_flag==0)//ȷ��ûץ
		{
				bj1_jc=0;
				s1_flag=1;//ȷ��Ҫ��
		}
		if(bj1_jc==0&&s1_flag==1)//δץ�Ƚ�
		{
				g_add_pulse_count[0]=0;
				stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,sec2o3_qs[0][0]*SPR,STEPPER_MOTOR_1);
			
				stepper_start(STEPPER_MOTOR_1);
				bj1_jc=1;//1//�����ٽ�
//				printf("1��\r\n");
		}
		else if(bj1_jc==2&&s1_flag==1)//ץ������
		{
				steer(BIHE,SERVO_1);//���ץ
				HAL_Delay(1000);
			
				g_add_pulse_count[0]=0;
				stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,sec2o3_qs[0][1]*SPR,STEPPER_MOTOR_1);
			
				stepper_start(STEPPER_MOTOR_1);
				bj1_jc=3;//3//�����ٽ�
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
 * @brief       �ڶ������׶�2�ż�צȡ�ﶯ������
 * @param       ��
 * @retval      OK���ɹ���NONE��ʧ��
 */
int sec2o3_zhuaqv2(void)
{
		if(s2_flag==0)//ȷ��ûץ
		{
				bj2_jc=0;
				s2_flag=1;//ȷ��Ҫ��
		}
		if(bj2_jc==0&&s2_flag==1)//δץ�Ƚ�
		{
				g_add_pulse_count[0]=0;
				stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,sec2o3_qs[1][0]*SPR,STEPPER_MOTOR_2);
			
				stepper_start(STEPPER_MOTOR_2);
				bj2_jc=1;//1//�����ٽ�
//				printf("1��\r\n");
		}
		else if(bj2_jc==2&&s2_flag==1)//ץ������
		{
				steer(BIHE,SERVO_2);//���ץ
//				printf("*****************/duoji\r\n");
				HAL_Delay(1000);
			
				g_add_pulse_count[0]=0;
				stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,sec2o3_qs[1][1]*SPR,STEPPER_MOTOR_2);
			
				stepper_start(STEPPER_MOTOR_2);
				bj2_jc=3;//3//�����ٽ�
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
 * @brief       �ڶ������׶ι�ͬȡ�ﶯ������
 * @param       ��
 * @retval      OK���ɹ���NONE��ʧ��
 */
int sec2o3_zhuaqv(void)
{
		if(jcdl_flag==0)//ȷ��ûץ
		{
				bj1_jc=0;
				bj2_jc=0;
				jcdl_flag=1;//ȷ��Ҫ��
		}
		if(bj1_jc==0&&bj2_jc==0&&jcdl_flag==1)//δץ�Ƚ�
		{
				g_add_pulse_count[0]=0;
				stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,sec2o3_qs[0][0]*SPR,STEPPER_MOTOR_1);
			
				g_add_pulse_count[1]=0;
				stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,sec2o3_qs[1][0]*SPR,STEPPER_MOTOR_2);
			
				stepper_start(STEPPER_MOTOR_1);
				stepper_start(STEPPER_MOTOR_2);
				bj1_jc=1;
				bj2_jc=1;//1//�����ٽ�
//				printf("1��\r\n");
		}
		else if(bj1_jc==2&&bj2_jc==2&&jcdl_flag==1)//ץ������
		{
				steer(BIHE,SERVO_ALL);//���ץ
				HAL_Delay(1000);
			
				g_add_pulse_count[0]=0;
				stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,sec2o3_qs[0][1]*SPR,STEPPER_MOTOR_1);
			
				g_add_pulse_count[1]=0;
				stepmotor_move_rel(V_START,fast_57END,fast_57ACTIME,fast_57DETIME,sec2o3_qs[1][1]*SPR,STEPPER_MOTOR_2);
			
				stepper_start(STEPPER_MOTOR_1);
				stepper_start(STEPPER_MOTOR_2);
				bj1_jc=3;
				bj2_jc=3;//3//�����ٽ�
//				printf("2��\r\n");
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
 * @brief       �ڶ������׶ι�ͬ���ﶯ������
 * @param       ��
 * @retval      OK���ɹ���NONE��ʧ��
 */
int sec2o3_fangwu(void)
{
		if(jcdl_flag==2)//ȷ���Ѿ�ץ����
		{
				bj1_jc=0;
				bj2_jc=0;
				jcdl_flag=3;//ȷ������
		}
		else if(s1_flag==2&&s2_flag==2)
		{
			bj1_jc=0;
			bj2_jc=0;
			s1_flag=3;//ȷ������
			s2_flag=3;//ȷ������
			jcdl_flag=3;//����ת��
		}
		if(bj1_jc==0&&bj2_jc==0&&jcdl_flag==3)//Ҫ���Ƚ�
		{
				g_add_pulse_count[0]=0;
				stepmotor_move_rel(V_START,slow_57END,slow_57ACTIME,slow_57DETIME,sec2o3_qs[0][2]*SPR,STEPPER_MOTOR_1);
			
				g_add_pulse_count[1]=0;
				stepmotor_move_rel(V_START,slow_57END,slow_57ACTIME,slow_57DETIME,sec2o3_qs[1][2]*SPR,STEPPER_MOTOR_2);
			
				stepper_start(STEPPER_MOTOR_2);	
				stepper_start(STEPPER_MOTOR_1);

				bj1_jc++;
				bj2_jc++;//1//�����ٽ�
		}
//		else if(bj1_jc==2&&bj2_jc==2&&jcdl_flag==3)//��������
//		{
//				steer(ZHANGKAI,SERVO_ALL);//�����
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
//				bj2_jc++;//3//�����ٽ�
////				printf("2��\r\n");

//		}

		if(g_motor1_sta==STATE_IDLE&&g_motor2_sta==STATE_IDLE&&jcdl_flag==3)
		{
				steer(ZHANGKAI,SERVO_ALL);//�����
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
 * @brief       ��ƽ�ƶ�������
 * @param       ��
 * @retval      OK���ɹ���NONE��ʧ��
 */
int nei_py(void)
{
		if(jcdl2_flag==0)
		{
				bj3_jc=0;
				bj4_jc=0;
				jcdl2_flag=1;
		}
		if(bj3_jc==0&&bj4_jc==0&&jcdl2_flag==1)//ץ������ƶ�
		{
				g_add_pulse_count[2]=0;
				g_add_pulse_count[3]=0;

				stepmotor_move_rel(V_START,fast_42END,fast_42ACTIME,fast_42DETIME,sec2o3_py[0]*SPR,STEPPER_MOTOR_3);
				stepmotor_move_rel(V_START,fast_42END,fast_42ACTIME,fast_42DETIME,sec2o3_py[0]*SPR,STEPPER_MOTOR_4);
				
				bj3_jc++;//1//�����ٽ�
				bj4_jc++;//1//�����ٽ�
				
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
 * @brief       ��ƽ�ƶ�������
 * @param       ��
 * @retval      OK���ɹ���NONE��ʧ��
 */
int wai_py(void)
{
		if(jcdl2_flag==2)
		{
				bj3_jc=0;
				bj4_jc=0;
				jcdl2_flag=3;
		}
		if(bj3_jc==0&&bj4_jc==0&&jcdl2_flag==3)//ץ������ƶ�
		{
				g_add_pulse_count[2]=0;
				stepmotor_move_rel(V_START,slow_42END,slow_42ACTIME,slow_42DETIME,sec2o3_py[1]*SPR,STEPPER_MOTOR_3);
				bj3_jc++;//1//�����ٽ�
				
				g_add_pulse_count[3]=0;
				stepmotor_move_rel(V_START,slow_42END,slow_42ACTIME,slow_42DETIME,sec2o3_py[1]*SPR,STEPPER_MOTOR_4);
				bj4_jc++;//1//�����ٽ�
				
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
 * @brief       ��ƽ�ƶ�������
 * @param       ��
 * @retval      OK���ɹ���NONE��ʧ��
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
			if(bj3_jc==0&&bj4_jc==0)//ץ������ƶ�
			{
					g_add_pulse_count[2]=0;
					stepmotor_move_rel(V_START,fast_42END,fast_42ACTIME,fast_42DETIME,sec2o3_py[2]*SPR,STEPPER_MOTOR_3);
					bj3_jc++;//1//�����ٽ�
					
					g_add_pulse_count[3]=0;
					stepmotor_move_rel(V_START,fast_42END,fast_42ACTIME,fast_42DETIME,sec2o3_py[3]*SPR,STEPPER_MOTOR_4);
					bj4_jc++;//1//�����ٽ�
					
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








