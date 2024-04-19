#include "stepper_motor.h"
#include "tim.h"
#include "malloc.h"
#include "math.h"
#include "usart.h"
#include <stdlib.h>
/*
			     PUL   DIR
Motor1   	 PC6	 PG11
Motor2   	 PC7	 PG12
Motor3   	 PC8	 PG13
Motor4   	 PC9	 PG14

*/
/****************************************S�ͼӼ����˶�*****************************************************/
volatile int32_t g_step_pos1 = 0,g_step_pos2 = 0,g_step_pos3 = 0,g_step_pos4 = 0;     /* ��ǰλ�� */
volatile uint16_t g_toggle_pulse[4]  = {0};                  /* ����Ƶ�ʿ��� */
motor_state_typedef g_motor1_sta  = STATE_IDLE,
g_motor2_sta  = STATE_IDLE,
g_motor3_sta  = STATE_IDLE,
g_motor4_sta  = STATE_IDLE;          /* ���״̬ */

speed_calc_t g_calc_t[4] = {0} ;

__IO  uint32_t g_add_pulse_count[4]={0};                     /* ��������ۼ� */


/**
 * @brief       �ٶȱ���㺯��
 * @param       vo,���ٶ�
 * @param		vt,ĩ�ٶ�
 * @param		time,����ʱ��
 * @param		id��������
 * @retval      TRUE���ɹ���FALSE��ʧ��
 */
uint8_t calc_speed(int32_t vo, int32_t vt, float time, uint8_t id)
{
		uint8_t ID=id-1;
    uint8_t is_dec = FALSE;
    int32_t i = 0;
    int32_t vm =0;                              /* �м���ٶ� */
    int32_t inc_acc_stp = 0;                    /* �Ӽ�������Ĳ��� */
    int32_t dec_acc_stp = 0;                    /* ����������Ĳ��� */
    int32_t accel_step = 0;                     /* ���ٻ������Ҫ�Ĳ��� */
    float jerk = 0;                             /* �Ӽ��ٶ� */
    float ti = 0;                               /* ʱ���� dt */
    float sum_t = 0;                            /* ʱ���ۼ��� */
    float delta_v = 0;                          /* �ٶȵ�����dv */
    float ti_cube = 0;                          /* ʱ���������� */
    float *velocity_tab[4] = {NULL,NULL,NULL,NULL};                 /* �ٶȱ��ָ�� */

//		printf("velocity_tab[%d]=%d\n",ID,velocity_tab[ID] );
  	if(vo > vt )                                /* ���ٶȱ�ĩ�ٶȴ�,�������˶�,��ֵ�仯�������˶���ͬ */
    {                                           /* ֻ�ǽ����ʱ��ע�⽫�ٶȵ��� */

				is_dec = TRUE;                          /* ���ٶ� */
        g_calc_t[ID].vo = ROUNDPS_2_STEPPS(vt);     /* ת����λ ����:step/s */
        g_calc_t[ID].vt = ROUNDPS_2_STEPPS(vo);     /* ת����λ ĩ��:step/s */
    }
		else
    {

        is_dec = FALSE;                         /* ���ٶ� */
        g_calc_t[ID].vo = ROUNDPS_2_STEPPS(vo);
        g_calc_t[ID].vt = ROUNDPS_2_STEPPS(vt);
    }
    time = ACCEL_TIME(time);                                                    /* �õ��Ӽ��ٶε�ʱ�� */
//    printf("motor%d:time=%f\r\n",ID,time);
//    printf("motor%d:time is ok\r\n",ID);
    vm =  (g_calc_t[ID].vo + g_calc_t[ID].vt) / 2 ;                                     /* �����е��ٶ� */
    
    jerk = fabs(2.0f * (vm - g_calc_t[ID].vo) /  (time * time));                    /* �����е��ٶȼ���Ӽ��ٶ� */

    inc_acc_stp = (int32_t)(g_calc_t[ID].vo * time + INCACCELSTEP(jerk,time));      /* �Ӽ�����Ҫ�Ĳ��� */

    dec_acc_stp = (int32_t)((g_calc_t[ID].vt + g_calc_t[ID].vo) * time - inc_acc_stp);  /* ��������Ҫ�Ĳ��� S = vt * time - S1 */

    /* �����ڴ�ռ����ٶȱ� */
    accel_step = dec_acc_stp + inc_acc_stp;                                     /* ������Ҫ�Ĳ��� */
    if( accel_step  % 2 != 0)                                                   /* ���ڸ���������ת�����������ݴ��������,���������1 */
        accel_step  += 1;
    	
		/* mallo�����ڴ�ռ�,�ǵ��ͷ� */
		velocity_tab[ID] = (float*)(mymalloc(SRAMEX,((accel_step + 1) * sizeof(float))));
		if(velocity_tab[ID] == NULL)
		{
				printf("motor%d �ڴ治��!���޸Ĳ���\r\n",ID);
				return FALSE;
		}		
//				printf("velocity_tab[%d]=%d\n",ID,velocity_tab[ID] );

//		for (i = 0; i < accel_step + 1; i++) 
//		{
//			printf("accel_step=%d   velocity_tab[%d][%d]=%d\n",accel_step,ID,i,velocity_tab[ID][i] );
//	 
//		}
//			printf ("velocity_tab[]=%d\r\n",accel_step);
/*
 * Ŀ���S���ٶ������Ƕ�ʱ��ķ���,�����ڿ��Ƶ����ʱ�������Բ����ķ�ʽ����,���������V-t������ת��
 * �õ�V-S����,����õ����ٶȱ��ǹ��ڲ������ٶ�ֵ.ʹ�ò������ÿһ�����ڿ��Ƶ���
 */
/* �����һ���ٶ�,���ݵ�һ�����ٶ�ֵ�ﵽ��һ����ʱ�� */
    ti_cube  = 6.0f * 1.0f / jerk;                  /* ����λ�ƺ�ʱ��Ĺ�ʽS = 1/6 * J * ti^3 ��1����ʱ��:ti^3 = 6 * 1 / jerk */
    ti = pow(ti_cube,(1 / 3.0f));                   /* ti */
    sum_t = ti;
    delta_v = 0.5f * jerk * pow(sum_t,2);           /* ��һ�����ٶ� */
		
		velocity_tab[ID][0] = g_calc_t[ID].vo + delta_v;

/*****************************************************/
    if( velocity_tab[ID][0] <= SPEED_MIN )              /* �Ե�ǰ��ʱ��Ƶ�����ܴﵽ������ٶ� */
        velocity_tab[ID][0] = SPEED_MIN;
    
/*****************************************************/
    
    for(i = 1; i < accel_step; i++)
    {
        /* ����������ٶȾ��Ƕ�ʱ���������Ƶ��,���Լ����ÿһ����ʱ�� */
        /* �õ���i-1����ʱ�� */
        ti = 1.0f / velocity_tab[ID][i-1];              /* ���ÿ��һ����ʱ�� ti = 1 / Vn-1 */
        /* �Ӽ��ٶ��ٶȼ��� */
        if( i < inc_acc_stp)
        {
            sum_t += ti;                            /* ��0��ʼ��i��ʱ���ۻ� */
            delta_v = 0.5f * jerk * pow(sum_t,2);   /* �ٶȵı仯��: dV = 1/2 * jerk * ti^2 */
            velocity_tab[ID][i] = g_calc_t[ID].vo + delta_v;/* �õ��Ӽ��ٶ�ÿһ����Ӧ���ٶ� */
            /* �����һ����ʱ��,ʱ�䲢���ϸ����time,��������Ҫ��������,��Ϊ�����ٶε�ʱ�� */
            if(i == inc_acc_stp - 1)
                sum_t  = fabs(sum_t - time );
        }
        /* �����ٶ��ٶȼ��� */
        else
        {
            sum_t += ti;                                        /* ʱ���ۼ� */
            delta_v = 0.5f * jerk * pow(fabs( time - sum_t),2); /* dV = 1/2 * jerk *(T-t)^2 ��������򿴼����ٵ�ͼ */
            velocity_tab[ID][i] = g_calc_t[ID].vt - delta_v;            /* V = vt - delta_v */
            if(velocity_tab[ID][i] >= g_calc_t[ID].vt)
            {
                accel_step = i;
                break;
            }
        }
    }
    if(is_dec == TRUE)                                          /* ���� */
    {
        float tmp_Speed = 0;
        /* �������� */
        for(i = 0; i< (accel_step / 2); i++)
        {
            tmp_Speed = velocity_tab[ID][i];
            velocity_tab[ID][i] = velocity_tab[ID][accel_step-1 - i];   /* ͷβ�ٶȶԻ� */
            velocity_tab[ID][accel_step-1 - i] = tmp_Speed;
        }

        g_calc_t[ID].decel_tab = velocity_tab[ID];                      /* ���ٶ��ٶȱ� */
        g_calc_t[ID].decel_step = accel_step;                       /* ���ٶε��ܲ��� */

    }
    else                                                        /* ���� */
    {
        g_calc_t[ID].accel_tab = velocity_tab[ID];                      /* ���ٶ��ٶȱ� */
//        printf ("velocity_tab[%d]=%d\r\n",ID,*velocity_tab[ID]);
				g_calc_t[ID].accel_step = accel_step;                       /* ���ٶε��ܲ��� */
    }
    return TRUE;	
}

/**
 * @brief       S�������ٶȾ���
 * @param       vo:���ٶ�
 * @param		vt:ĩ�ٶ�
 * @param		AcTime:����ʱ��
 * @param		DeTime:����ʱ��
 * @param		step:����
 * @param		id:������
 * @retval      ��
 */
void stepmotor_move_rel(int32_t vo, int32_t vt, float AcTime,float DeTime,int32_t step,uint8_t id)
{
		uint8_t ID=id-1;
	
    if(calc_speed(vo,vt,AcTime,id) == FALSE) /* ��������ٶε��ٶȺͲ��� */
        return;
    if(calc_speed(vt,vo,DeTime,id) == FALSE) /* ��������ٶε��ٶȺͲ��� */
        return;
    if(step < 0)
    {
        step = -step;
				switch(id)
				{
						case STEPPER_MOTOR_1:
									ST1_DIR(CCW);
						break;
						case STEPPER_MOTOR_2:
									ST2_DIR(CCW);
						break;
						case STEPPER_MOTOR_3:
									ST3_DIR(CCW);
						break;
						case STEPPER_MOTOR_4:
									ST4_DIR(CCW);
						break;
				}
    }
    else
    {
				switch(id)
				{
						case STEPPER_MOTOR_1:
									ST1_DIR(CW);
						break;
						case STEPPER_MOTOR_2:
									ST2_DIR(CW);
						break;
						case STEPPER_MOTOR_3:
									ST3_DIR(CW);
						break;
						case STEPPER_MOTOR_4:
									ST4_DIR(CW);
						break;
				}    
		}
    
    if(step >= (g_calc_t[ID].decel_step+g_calc_t[ID].accel_step) )          /* ���ܲ������ڵ��ڼӼ��ٶȲ������ʱ���ſ���ʵ��������S�μӼ��� */
    {
        g_calc_t[ID].step = step;
        g_calc_t[ID].dec_point = g_calc_t[ID].step - g_calc_t[ID].decel_step;   /* ��ʼ���ٵĲ��� */
//				printf("/**����**/g_calc_t[%d].accel_step=%d\r\n",ID,g_calc_t[ID].accel_step);
//				printf("/**����**/g_calc_t[%d].dec_point=%d\r\n",ID,g_calc_t[ID].dec_point);
//				printf("/**����**/g_calc_t[%d].decel_step=%d\r\n",ID,g_calc_t[ID].decel_step);

    }
    else                                                            /* ���������Խ����㹻�ļӼ��� */
    {
        /* �������㲻�����˶���Ҫ��ǰ��������ٶȱ���ռ�ڴ��ͷţ��Ա�������ظ����� */
        myfree(SRAMEX,g_calc_t[ID].accel_tab);                          /* �ͷż��ٶ��ٶȱ� */
        myfree(SRAMEX,g_calc_t[ID].decel_tab);                          /* �ͷż��ٶ��ٶȱ� */
			printf("motor%d:�������㣬�������ô���!\r\n",ID);
        return;
    }
    g_calc_t[ID].step_pos = 0;
		switch(id)
			{
					case STEPPER_MOTOR_1:
							g_motor1_sta = STATE_ACCEL;                                      /* ���Ϊ����״̬ */
					break;
					case STEPPER_MOTOR_2:
							g_motor2_sta = STATE_ACCEL;                                      /* ���Ϊ����״̬ */
					break;
					case STEPPER_MOTOR_3:
							g_motor3_sta = STATE_ACCEL;                                      /* ���Ϊ����״̬ */
					break;
					case STEPPER_MOTOR_4:
							g_motor4_sta = STATE_ACCEL;                                      /* ���Ϊ����״̬ */
					break;
			}

    g_calc_t[ID].ptr = g_calc_t[ID].accel_tab;                              /* �Ѽ��ٶε��ٶȱ�洢��ptr��� */
    g_toggle_pulse[ID]  = (uint32_t)(T1_FREQ/(*g_calc_t[ID].ptr));
    g_calc_t[ID].ptr++;
//    printf ("g_toggle_pulse[%d]=%d\r\n",ID,g_toggle_pulse[ID]);

		__HAL_TIM_SET_COUNTER(&htim8,0);
		switch(id)
			{
					case STEPPER_MOTOR_1:
							__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,(uint16_t)(g_toggle_pulse[ID]/2));  /*  ���ö�ʱ���Ƚ�ֵ */
					break;
					case STEPPER_MOTOR_2:
							__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,(uint16_t)(g_toggle_pulse[ID]/2));  /*  ���ö�ʱ���Ƚ�ֵ */
					break;
					case STEPPER_MOTOR_3:
							__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,(uint16_t)(g_toggle_pulse[ID]/2));  /*  ���ö�ʱ���Ƚ�ֵ */
					break;
					case STEPPER_MOTOR_4:
							__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,(uint16_t)(g_toggle_pulse[ID]/2));  /*  ���ö�ʱ���Ƚ�ֵ */
					break;
			}

}

/**
  * @brief  ��ʱ���Ƚ��жϺ���
  * @param  htim����ʱ�����ָ��
  * @note   ��
  * @retval ��
*/
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    volatile uint32_t Tim_Count[4] = {0};
    volatile uint32_t tmp[4] = {0};
    volatile float Tim_Pulse[4] = {0};
    volatile static uint8_t num_callback[4] = {0}; 
    if(htim->Instance==TIM8)
    {
				if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
				{
							num_callback[0]++;                    /* ��ʱ���жϴ�������ֵ */
							if(num_callback[0] == 2)              /* 2�Σ�˵���Ѿ����һ���������� */
							{
									num_callback[0] = 0;              /* ���㶨ʱ���жϴ�������ֵ */
									g_step_pos1 ++;      /* ��ǰλ�� */
									if((g_motor1_sta!=STATE_IDLE)&&(g_motor1_sta != STATE_STOP))
									{
											g_calc_t[0].step_pos ++;
//											printf ("1111\r\n");

									}
									switch(g_motor1_sta)
									{
											case STATE_ACCEL:
//												  printf("ACCEL     %d    %d\r\n",g_calc_t[0].step_pos,g_calc_t[0].accel_step);
													g_add_pulse_count[0]++;
													Tim_Pulse[0] = T1_FREQ / (*g_calc_t[0].ptr);          /* ���ٶȱ�õ�ÿһ���Ķ�ʱ������ֵ */
													g_calc_t[0].ptr++;                                 /* ȡ�ٶȱ����һλ */
													g_toggle_pulse[0] = (uint16_t) (Tim_Pulse[0] / 2);    /* ��תģʽC��Ҫ����2 */
													if(g_calc_t[0].step_pos >= g_calc_t[0].accel_step)    /* �����ڼ��ٶβ����ͽ������� */
													{
															myfree(SRAMEX,g_calc_t[0].accel_tab);          /* �˶���Ҫ�ͷ��ڴ� */
															g_motor1_sta = STATE_AVESPEED;
													}
													break;
											case STATE_DECEL:
//													printf("9999999999999\r\n");
//												  printf("DECEL     %d    %d\r\n",g_calc_t[0].step_pos,g_calc_t[0].step);
													g_add_pulse_count[0]++;
													Tim_Pulse[0] = T1_FREQ / (*g_calc_t[0].ptr);          /* ���ٶȱ�õ�ÿһ���Ķ�ʱ������ֵ */
													g_calc_t[0].ptr++;
													g_toggle_pulse[0] = (uint16_t) (Tim_Pulse[0] / 2);
													if(g_calc_t[0].step_pos >= g_calc_t[0].step )
													{
															myfree(SRAMEX,g_calc_t[0].decel_tab);  
														/* �˶���Ҫ�ͷ��ڴ� */
															g_motor1_sta = STATE_STOP;
//															printf("//***here****////  ");
//															printf("g_motor1_sta=%d\r\n",g_motor1_sta);
													}
													break;
											case STATE_AVESPEED:
//												  printf("AVESPEED     %d    %d\r\n",g_calc_t[0].step_pos,g_calc_t[0].dec_point);
													g_add_pulse_count[0]++;
													Tim_Pulse[0]  = T1_FREQ /g_calc_t[0].vt;
													g_toggle_pulse[0] = (uint16_t) (Tim_Pulse[0] / 2);
													if(g_calc_t[0].step_pos >= g_calc_t[0].dec_point )
													{
															g_calc_t[0].ptr = g_calc_t[0].decel_tab;          /* �����ٶε��ٶȱ�ֵ��ptr */
															g_motor1_sta = STATE_DECEL;
													}
													break;
											case STATE_STOP:
//													printf ("STOP\r\n");
													HAL_TIM_OC_Stop_IT(&htim8, TIM_CHANNEL_1);     /* ������ӦPWMͨ�� */
													g_motor1_sta = STATE_IDLE;
													break;
											case STATE_IDLE:
													break;
									}	
							}
						/*  ���ñȽ�ֵ */
						Tim_Count[0]=__HAL_TIM_GET_COUNTER(&htim8);
						tmp[0] = 0xFFFF & (Tim_Count[0] + g_toggle_pulse[0]);
						
						__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,tmp[0]);
				}
				if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
				{
							num_callback[1]++;                    /* ��ʱ���жϴ�������ֵ */
							if(num_callback[1] == 2)              /* 2�Σ�˵���Ѿ����һ���������� */
							{
									num_callback[1] = 0;              /* ���㶨ʱ���жϴ�������ֵ */
									g_step_pos2 ++;      /* ��ǰλ�� */
//									printf("%d",g_motor2_sta);
									if((g_motor2_sta!=STATE_IDLE)&&(g_motor2_sta != STATE_STOP))
									{
											g_calc_t[1].step_pos ++;
									}
									switch(g_motor2_sta)
									{											
											case STATE_ACCEL:
//												  printf("ACCEL     %d    %d\r\n",g_calc_t[1].step_pos,g_calc_t[1].accel_step);
													g_add_pulse_count[1]++;
													Tim_Pulse[1] = T1_FREQ / (*g_calc_t[1].ptr);          /* ���ٶȱ�õ�ÿһ���Ķ�ʱ������ֵ */
													g_calc_t[1].ptr++;                                 /* ȡ�ٶȱ����һλ */
													g_toggle_pulse[1] = (uint16_t) (Tim_Pulse[1] / 2);											/* ��תģʽC��Ҫ����2 */
													if(g_calc_t[1].step_pos >= g_calc_t[1].accel_step)    /* �����ڼ��ٶβ����ͽ������� */
													{
															myfree(SRAMEX,g_calc_t[1].accel_tab);          /* �˶���Ҫ�ͷ��ڴ� */
															g_motor2_sta = STATE_AVESPEED;
													}
													break;
											case STATE_DECEL:
//												  printf("DECEL     %d    %d\r\n",g_calc_t[1].step_pos,g_calc_t[1].step);
													g_add_pulse_count[1]++;
													Tim_Pulse[1] = T1_FREQ / (*g_calc_t[1].ptr);          /* ���ٶȱ�õ�ÿһ���Ķ�ʱ������ֵ */
													g_calc_t[1].ptr++;
													g_toggle_pulse[1] = (uint16_t) (Tim_Pulse[1] / 2);
													if(g_calc_t[1].step_pos >= g_calc_t[1].step )
													{
															myfree(SRAMEX,g_calc_t[1].decel_tab);          /* �˶���Ҫ�ͷ��ڴ� */
															g_motor2_sta = STATE_STOP;
													}
													break;
											case STATE_AVESPEED:
//												  printf("AVESPEED     %d    %d\r\n",g_calc_t[1].step_pos,g_calc_t[1].dec_point);
													g_add_pulse_count[1]++;
													Tim_Pulse[1]  = T1_FREQ /g_calc_t[1].vt;
													g_toggle_pulse[1] = (uint16_t) (Tim_Pulse[1] / 2);
													if(g_calc_t[1].step_pos >= g_calc_t[1].dec_point )
													{
															g_calc_t[1].ptr = g_calc_t[1].decel_tab;          /* �����ٶε��ٶȱ�ֵ��ptr */
															g_motor2_sta = STATE_DECEL;
													}
													break;
											case STATE_STOP:
//													printf ("STOP\r\n");
													HAL_TIM_OC_Stop_IT(&htim8, TIM_CHANNEL_2);     /* ������ӦPWMͨ�� */
													g_motor2_sta = STATE_IDLE;
													break;
											case STATE_IDLE:
													break;
									}
							}
						/*  ���ñȽ�ֵ */
						Tim_Count[1]=__HAL_TIM_GET_COUNTER(&htim8);
						tmp[1] = 0xFFFF & (Tim_Count[1] + g_toggle_pulse[1]);
						__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,tmp[1]);
				}
				if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
				{
							num_callback[2]++;                    /* ��ʱ���жϴ�������ֵ */
							if(num_callback[2] == 2)              /* 2�Σ�˵���Ѿ����һ���������� */
							{
									num_callback[2] = 0;              /* ���㶨ʱ���жϴ�������ֵ */
									g_step_pos3 ++;      /* ��ǰλ�� */
									if((g_motor3_sta!=STATE_IDLE)&&(g_motor3_sta != STATE_STOP))
									{
											g_calc_t[2].step_pos ++;
									}
									switch(g_motor3_sta)
									{
											case STATE_ACCEL:
//												  printf("ACCEL     %d    %d\r\n",g_calc_t[2].step_pos,g_calc_t[2].accel_step);
													g_add_pulse_count[2]++;
													Tim_Pulse[2] = T1_FREQ / (*g_calc_t[2].ptr);          /* ���ٶȱ�õ�ÿһ���Ķ�ʱ������ֵ */
													g_calc_t[2].ptr++;                                 /* ȡ�ٶȱ����һλ */
													g_toggle_pulse[2] = (uint16_t) (Tim_Pulse[2] / 2);    /* ��תģʽC��Ҫ����2 */
													if(g_calc_t[2].step_pos >= g_calc_t[2].accel_step)    /* �����ڼ��ٶβ����ͽ������� */
													{
															myfree(SRAMEX,g_calc_t[2].accel_tab);          /* �˶���Ҫ�ͷ��ڴ� */
															g_motor3_sta = STATE_AVESPEED;
													}
													break;
											case STATE_DECEL:
//												  printf("DECEL     %d    %d\r\n",g_calc_t[1].step_pos,g_calc_t[1].step);
													g_add_pulse_count[2]++;
													Tim_Pulse[2] = T1_FREQ / (*g_calc_t[2].ptr);          /* ���ٶȱ�õ�ÿһ���Ķ�ʱ������ֵ */
													g_calc_t[2].ptr++;
													g_toggle_pulse[2] = (uint16_t) (Tim_Pulse[2] / 2);
													if(g_calc_t[2].step_pos >= g_calc_t[2].step )
													{
															myfree(SRAMEX,g_calc_t[2].decel_tab);          /* �˶���Ҫ�ͷ��ڴ� */
															g_motor3_sta = STATE_STOP;
													}
													break;
											case STATE_AVESPEED:
//												  printf("AVESPEED     %d    %d\r\n",g_calc_t[2].step_pos,g_calc_t[2].dec_point);
													g_add_pulse_count[2]++;
													Tim_Pulse[2]  = T1_FREQ /g_calc_t[2].vt;
													g_toggle_pulse[2] = (uint16_t) (Tim_Pulse[2] / 2);
													if(g_calc_t[2].step_pos >= g_calc_t[2].dec_point )
													{
															g_calc_t[2].ptr = g_calc_t[2].decel_tab;          /* �����ٶε��ٶȱ�ֵ��ptr */
															g_motor3_sta = STATE_DECEL;
													}
													break;
											case STATE_STOP:
//													printf ("STOP\r\n");
													HAL_TIM_OC_Stop_IT(&htim8, TIM_CHANNEL_3);     /* ������ӦPWMͨ�� */
													g_motor3_sta = STATE_IDLE;
													break;
											case STATE_IDLE:
													break;
									}
							}
						/*  ���ñȽ�ֵ */
						Tim_Count[2]=__HAL_TIM_GET_COUNTER(&htim8);
						tmp[2] = 0xFFFF & (Tim_Count[2] + g_toggle_pulse[2]);
//						printf ("g_toggle_pulse[2]=%d\r\n",g_toggle_pulse[2]);
						__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,tmp[2]);

				}
				if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
				{
							num_callback[3]++;                    /* ��ʱ���жϴ�������ֵ */
							if(num_callback[3] == 2)              /* 2�Σ�˵���Ѿ����һ���������� */
							{
									num_callback[3] = 0;              /* ���㶨ʱ���жϴ�������ֵ */
									g_step_pos4 ++;      /* ��ǰλ�� */
									if((g_motor4_sta!=STATE_IDLE)&&(g_motor4_sta != STATE_STOP))
									{
											g_calc_t[3].step_pos ++;
									}
									switch(g_motor4_sta)
									{
											case STATE_ACCEL:
													g_add_pulse_count[3]++;
													Tim_Pulse[3] = T1_FREQ / (*g_calc_t[3].ptr);          /* ���ٶȱ�õ�ÿһ���Ķ�ʱ������ֵ */
													g_calc_t[3].ptr++;                                 /* ȡ�ٶȱ����һλ */
													g_toggle_pulse[3] = (uint16_t) (Tim_Pulse[3] / 2);    /* ��תģʽC��Ҫ����2 */
													if(g_calc_t[3].step_pos >= g_calc_t[3].accel_step)    /* �����ڼ��ٶβ����ͽ������� */
													{
															myfree(SRAMEX,g_calc_t[3].accel_tab);          /* �˶���Ҫ�ͷ��ڴ� */
															g_motor4_sta = STATE_AVESPEED;
													}
													break;
											case STATE_DECEL:
													g_add_pulse_count[3]++;
													Tim_Pulse[3] = T1_FREQ / (*g_calc_t[3].ptr);          /* ���ٶȱ�õ�ÿһ���Ķ�ʱ������ֵ */
													g_calc_t[3].ptr++;
													g_toggle_pulse[3] = (uint16_t) (Tim_Pulse[3] / 2);
													if(g_calc_t[3].step_pos >= g_calc_t[3].step )
													{
															myfree(SRAMEX,g_calc_t[3].decel_tab);          /* �˶���Ҫ�ͷ��ڴ� */
															g_motor4_sta = STATE_STOP;
													}
													break;
											case STATE_AVESPEED:
													g_add_pulse_count[3]++;
													Tim_Pulse[3]  = T1_FREQ /g_calc_t[3].vt;
													g_toggle_pulse[3] = (uint16_t) (Tim_Pulse[3] / 2);
													if(g_calc_t[3].step_pos >= g_calc_t[3].dec_point )
													{
															g_calc_t[3].ptr = g_calc_t[3].decel_tab;          /* �����ٶε��ٶȱ�ֵ��ptr */
															g_motor4_sta = STATE_DECEL;
													}
													break;
											case STATE_STOP:
													HAL_TIM_OC_Stop_IT(&htim8, TIM_CHANNEL_4);     /* ������ӦPWMͨ�� */
													g_motor4_sta = STATE_IDLE;
													break;
											case STATE_IDLE:
													break;
									}
							}
													/*  ���ñȽ�ֵ */
						Tim_Count[3]=__HAL_TIM_GET_COUNTER(&htim8);
						tmp[3] = 0xFFFF & (Tim_Count[3] + g_toggle_pulse[3]);
						__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,tmp[3]);

				}
    }
}



/*******************************����������պ���******************************************/
/**
 * @brief       �����������
 * @param       motor_num: ��������ӿ����
 * @retval      ��
 */
void stepper_start(uint8_t motor_num)
{
    /* ������ӦPWMͨ�� */
    switch(motor_num)
		{
			case 1:
			{
					HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_1);
					break;			
			}
			case 2:
			{
					HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_2);
					break;			
			}
			case 3:
			{
					HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_3);
					break;			
			}
			case 4:
			{
					HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_4);
					break;			
			}				
		  default : break;
		}
}
/**
 * @brief       �رղ������
 * @param       motor_num: ��������ӿ����
 * @retval      ��
 */
void stepper_stop(uint8_t motor_num)
{
    /* �رն�ӦPWMͨ�� */
    switch(motor_num)
		{
			case 1:
			{
					HAL_TIM_OC_Stop_IT(&htim8, TIM_CHANNEL_1);
					break;			
			}
			case 2:
			{
					HAL_TIM_OC_Stop_IT(&htim8, TIM_CHANNEL_2);
					break;			
			}
			case 3:
			{
					HAL_TIM_OC_Stop_IT(&htim8, TIM_CHANNEL_3);
					break;			
			}
			case 4:
			{
					HAL_TIM_OC_Stop_IT(&htim8, TIM_CHANNEL_4);
					break;			
			}				
		  default : break;
		}
}




