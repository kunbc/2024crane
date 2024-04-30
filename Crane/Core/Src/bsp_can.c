#include "can.h"
#include "bsp_can.h"
#include "mytype.h"
#include "usart.h"

moto_measure_t moto_chassis[4] = {0};

CAN_FilterTypeDef 		CAN_FilterConfigStructure;
CAN_TxHeaderTypeDef 	CAN_TxMsg;
CAN_RxHeaderTypeDef 	CAN_RxMsg;
uint8_t TxMsgData[8];
uint8_t RxMsgData[8];
uint32_t mypTxMsg=0;

static motor_measure_t motor_chassis[4];

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
    }

void can_filter_init(CAN_HandleTypeDef* _hcan)
{
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfigStructure.FilterIdHigh = 0x0000;    //不屏蔽信号
	CAN_FilterConfigStructure.FilterIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
	CAN_FilterConfigStructure.FilterBank = 0;
	CAN_FilterConfigStructure.FilterActivation = ENABLE;

	HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure);

	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *_hcan)
{
	static u8 i;
	CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
	if(hcan2.Instance == CAN2)
	{
		HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header, rx_data);
		switch(rx_header.StdId)
		{
			case CAN_2006Moto1_ID:
			case CAN_2006Moto2_ID:
			case CAN_2006Moto3_ID:
			case CAN_2006Moto4_ID:
				{
					i = rx_header.StdId - CAN_2006Moto1_ID;
					get_motor_measure(&motor_chassis[i], rx_data);	
					// printf("%d,%d\n",rx_data[2],rx_data[3]);
					// printf("%d\n",motor_chassis[0].speed_rpm);
					break;	
				}
			default:
				break;
		}
	}
	__HAL_CAN_ENABLE_IT(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}


void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* _hcan)
{
//	HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &CAN_RxMsg, RxMsgData);
	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(RxMsgData[0]<<8 | RxMsgData[1]) ;
	ptr->speed_rpm  = (int16_t)(RxMsgData[2]<<8 | RxMsgData[3]);
	ptr->real_current = (RxMsgData[4]<<8 | RxMsgData[5])*5.f/16384.f;

	ptr->hall = RxMsgData[6];
	
	
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
//	printf("ptr->total_anglet=%d\r\n",ptr->total_angle);
}


void get_total_angle(moto_measure_t *p)
{
	int res1, res2, delta;
	if(p->angle < p->last_angle){			//可能的情况
		res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
		res2 = p->angle - p->last_angle;				//反转	delta=-
	}else{	//angle > last
		res1 = p->angle - 8192 - p->last_angle ;//反转	delta -
		res2 = p->angle - p->last_angle;				//正转	delta +
	}
	//不管正反转，肯定是转的角度小的那个是真的
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;

	p->total_angle += delta;
	p->last_angle = p->angle;
}

void set_moto_current(s16 iq1, s16 iq2, s16 iq3, s16 iq4)
{
	CAN_TxMsg.StdId = 0x200;   //发送ID
	CAN_TxMsg.IDE = CAN_ID_STD;  //标准模式发送
	CAN_TxMsg.RTR = CAN_RTR_DATA;  //内容为数据
	CAN_TxMsg.DLC = 0x08;  //字节长度为8位
	CAN_TxMsg.TransmitGlobalTime = DISABLE;
	TxMsgData[0] = (iq1 >> 8);
	TxMsgData[1] = iq1;
	TxMsgData[2] = (iq2 >> 8);
	TxMsgData[3] = iq2;
	TxMsgData[4] = iq3 >> 8;
	TxMsgData[5] = iq3;
	TxMsgData[6] = iq4 >> 8;
	TxMsgData[7] = iq4;
	HAL_CAN_AddTxMessage(&hcan2,&CAN_TxMsg,TxMsgData,&mypTxMsg);
}

const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
