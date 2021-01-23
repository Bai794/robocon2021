
#include "can.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "main.h"

//moto_measure_t moto_pit;
//moto_measure_t moto_yaw;
//moto_measure_t moto_poke;	//拨单电机
moto_measure_t moto_chassis[8] = {0};//前4个控制舵轮速度，后4个负着转向
moto_measure_t moto_info;


void get_total_angle(moto_measure_t *p);

void get_moto_offset(moto_measure_t *ptr, uint8_t Rx_Data[]);

/*******************************************************************************************
  * @Func		my_can_filter_init
  * @Brief    CAN1和CAN2滤波器配置
  * @Param		CAN_HandleTypeDef* hcan
  * @Retval		None
  * @Date     2015/11/30
 *******************************************************************************************/
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan)
{
    //can1 &can2 use same filter config
    CAN_FilterTypeDef CAN_FilterType;
    CAN_FilterType.FilterBank = 0;
    CAN_FilterType.FilterIdHigh = 0x0000;
    CAN_FilterType.FilterIdLow = 0x0000;
    CAN_FilterType.FilterMaskIdHigh = 0x0000; //(((uint32_t)0x1313<<3)&0xFFFF0000)>>16;
    CAN_FilterType.FilterMaskIdLow = 0x0000;//(((uint32_t)0x1313<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF;
    CAN_FilterType.FilterFIFOAssignment = CAN_RX_FIFO0;
    CAN_FilterType.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterType.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterType.FilterActivation = ENABLE;
    CAN_FilterType.SlaveStartFilterBank = 14;
    if(HAL_CAN_ConfigFilter(_hcan, &CAN_FilterType) != HAL_OK)
    {
        Error_Handler();
    }
    if(HAL_CAN_ActivateNotification(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
    if(HAL_CAN_Start(_hcan) != HAL_OK)
    {
        Error_Handler();
    }

}

/*******************************************************************************************
  * @Func			void can_filter_recv_special(CAN_HandleTypeDef* _hcan, s16 id)
  * @Brief    待测试！！！
  * @Param		只接收filtered id，其他的全屏蔽。
  * @Retval		eg： 	CAN1_FilterConfiguration(0, HOST_CONTROL_ID);
										CAN1_FilterConfiguration(1, SET_CURRENT_ID);
										CAN1_FilterConfiguration(2, SET_VOLTAGE_ID);
										CAN1_FilterConfiguration(3, ESC_CAN_DEVICE_ID);
										CAN1_FilterConfiguration(4, SET_POWER_ID);
										CAN1_FilterConfiguration(5, SET_LIMIT_RECOVER_ID);
  * @Date     2016年11月11日
 *******************************************************************************************/
//void can_filter_recv_special(CAN_HandleTypeDef* hcan, uint8_t filter_number, uint16_t filtered_id)
//{
//    CAN_FilterConfTypeDef   cf;
//    cf.FilterNumber = filter_number;	//过滤器组编号
//    cf.FilterMode = CAN_FILTERMODE_IDMASK;	//id屏蔽模式
//    cf.FilterScale = CAN_FILTERSCALE_32BIT;	//32bit 滤波
//    cf.FilterIdHigh = (filtered_id << 21) >> 16;	//high 16 bit		其实这两个结构体成员变量是16位宽
//    cf.FilterIdLow = filtered_id << 21;	//low 16bit
//    cf.FilterMaskIdHigh = 0xFFFF;
//    cf.FilterMaskIdLow = 0xFFF8;	//IDE[2], RTR[1] TXRQ[0] 低三位不考虑。
//    cf.FilterFIFOAssignment = CAN_FilterFIFO0;
//    cf.BankNumber = 14;	//can1(0-13)和can2(14-27)分别得到一半的filter
//    cf.FilterActivation = ENABLE;
//    HAL_CAN_ConfigFilter(hcan, &cf);
//}


HAL_StatusTypeDef can_send_msg()
{
//	if(_hcan->Instance->ESR){
//		//can error occured, sleep can and reset!
//		_hcan->Instance->MCR |= 0x02;
//		_hcan->Instance->MCR &= ~(0x02);
//	}//这个是zw试过的可以解决can错误  有待验证！
    return HAL_OK;
}

float ZGyroModuleAngle;
/*******************************************************************************************
  * @Func			void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
  * @Brief    这是一个回调函数,都不用声明
  * @Param
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *_hcan)
{
    //ignore can1 or can2.
//	 printf("%d\r\n",_hcan->pRxMsg->StdId);
    CAN_RxHeaderTypeDef CAN_RxHeader;
//    HAL_StatusTypeDef HAL_Retval;
    uint8_t Rx_Data[8];
    u8 i;
//    uint8_t Data_Len = 0;
//    uint32_t ID = 0;

    HAL_CAN_GetRxMessage(_hcan, CAN_RX_FIFO0, &CAN_RxHeader, Rx_Data);
    switch(CAN_RxHeader.StdId) {
    case CAN_3510Moto1_ID:
    {

        i = CAN_RxHeader.StdId - CAN_3510Moto1_ID;
        moto_chassis[i].msg_cnt++ <= 50	?	get_moto_offset(&moto_chassis[i], Rx_Data) : get_moto_measure(&moto_chassis[i], Rx_Data);
//			get_moto_measure(&moto_info, Rx_Data);
    }
    case CAN_3510Moto4_ID:
    {

        i = CAN_RxHeader.StdId - CAN_3510Moto1_ID;
        moto_chassis[i].msg_cnt++ <= 50	?	get_moto_offset(&moto_chassis[i], Rx_Data) : get_moto_measure(&moto_chassis[i], Rx_Data);
    }

    case CAN_3510Moto3_ID:
    {
        i = CAN_RxHeader.StdId - CAN_3510Moto1_ID;
        moto_chassis[i].msg_cnt++ <= 50	?	get_moto_offset(&moto_chassis[i], Rx_Data) : get_moto_measure(&moto_chassis[i], Rx_Data);
    }
    case CAN_3510Moto2_ID:
    {

        i = CAN_RxHeader.StdId - CAN_3510Moto1_ID;

        moto_chassis[i].msg_cnt++ <= 50	?	get_moto_offset(&moto_chassis[i], Rx_Data) : get_moto_measure(&moto_chassis[i], Rx_Data);
//        get_moto_measure(&moto_info, Rx_Data);
//				get_moto_measure(&moto_chassis[i], _hcan);
    }
    case CAN_3510Moto5_ID:
    {
        i = CAN_RxHeader.StdId - CAN_3510Moto1_ID;
        moto_chassis[i].msg_cnt++ <= 50	?	get_moto_offset(&moto_chassis[i], Rx_Data) : get_moto_measure(&moto_chassis[i], Rx_Data);
    }
    case CAN_3510Moto6_ID:
    {
        i = CAN_RxHeader.StdId - CAN_3510Moto1_ID;
        moto_chassis[i].msg_cnt++ <= 50	?	get_moto_offset(&moto_chassis[i], Rx_Data) : get_moto_measure(&moto_chassis[i], Rx_Data);
    }
    case CAN_3510Moto7_ID:
    {
        i = CAN_RxHeader.StdId - CAN_3510Moto1_ID;
        moto_chassis[i].msg_cnt++ <= 50	?	get_moto_offset(&moto_chassis[i], Rx_Data) : get_moto_measure(&moto_chassis[i], Rx_Data);
    }
    case CAN_3510Moto8_ID:
    {
        i = CAN_RxHeader.StdId - CAN_3510Moto1_ID;
        moto_chassis[i].msg_cnt++ <= 50	?	get_moto_offset(&moto_chassis[i], Rx_Data) : get_moto_measure(&moto_chassis[i], Rx_Data);
    }
    break;

    }

}
int Can_TxMessage(CAN_HandleTypeDef* _hcan, uint8_t ide, uint32_t id, uint8_t len, uint8_t *data)
{
    uint32_t   TxMailbox;
    CAN_TxHeaderTypeDef CAN_TxHeader;
    HAL_StatusTypeDef   HAL_RetVal;
    uint16_t i = 0;
    if(ide == 0)
    {
        CAN_TxHeader.IDE = CAN_ID_STD;	//标准帧
        CAN_TxHeader.StdId = id;
    }
    else
    {
        CAN_TxHeader.IDE = CAN_ID_EXT;			//扩展帧
        CAN_TxHeader.ExtId = id;
    }
    CAN_TxHeader.DLC = len;
    CAN_TxHeader.RTR = CAN_RTR_DATA;//数据帧,CAN_RTR_REMxx	OTE遥控帧
    CAN_TxHeader.TransmitGlobalTime = DISABLE;
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0)
    {
        i++;
        if(i > 0xfffe)
            return 1;
    }
    HAL_RetVal = HAL_CAN_AddTxMessage(_hcan, &CAN_TxHeader, data, &TxMailbox);
    if(HAL_RetVal != HAL_OK)
        return 1;
    return 0;
}

/*******************************************************************************************
  * @Func			void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief    接收云台电机,3510电机通过CAN发过来的信息
  * @Param
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
void get_moto_measure(moto_measure_t *ptr, uint8_t Rx_Data[])
{
//	u32  sum=0;
//	u8	 i = FILTER_BUF_LEN;

    /*BUG!!! dont use this para code*/
//	ptr->angle_buf[ptr->buf_idx] = (uint16_t)(Rx_Data[0]<<8 | Rx_Data[1]) ;
//	ptr->buf_idx = ptr->buf_idx++ > FILTER_BUF_LEN ? 0 : ptr->buf_idx;
//	while(i){
//		sum += ptr->angle_buf[--i];
//	}
//	ptr->fited_angle = sum / FILTER_BUF_LEN;
    ptr->last_angle = ptr->angle;
    ptr->angle = (uint16_t)(Rx_Data[0] << 8 | Rx_Data[1]) ;
    ptr->real_current  = (int16_t)(Rx_Data[2] << 8 | Rx_Data[3]);
    ptr->speed_rpm = ptr->real_current;	//这里是因为两种电调对应位不一样的信息
    ptr->given_current = (int16_t)(Rx_Data[4] << 8 | Rx_Data[5]) / -5;
    ptr->hall = Rx_Data[6];
    if(ptr->angle - ptr->last_angle > 4096)
        ptr->round_cnt --;
    else if (ptr->angle - ptr->last_angle < -4096)
        ptr->round_cnt ++;
    ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}

/*this function should be called after system+can init */
void get_moto_offset(moto_measure_t *ptr, uint8_t Rx_Data[])
{
    ptr->angle = (uint16_t)(Rx_Data[0] << 8 | Rx_Data[1]) ;
    ptr->offset_angle = ptr->angle;
}

#define ABS(x)	( (x>0) ? (x) : (-x) )
/**
*@bref 电机上电角度=0， 之后用这个函数更新3510电机的相对开机后（为0）的相对角度。
	*/
void get_total_angle(moto_measure_t *p) {

    int res1, res2, delta;
    if(p->angle < p->last_angle) {			//可能的情况
        res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
        res2 = p->angle - p->last_angle;				//反转	delta=-
    } else {	//angle > last
        res1 = p->angle - 8192 - p->last_angle ;//反转	delta -
        res2 = p->angle - p->last_angle;				//正转	delta +
    }
    //不管正反转，肯定是转的角度小的那个是真的
    if(ABS(res1) < ABS(res2))
        delta = res1;
    else
        delta = res2;

    p->total_angle += delta;
    p->last_angle = p->angle;
}


void set_moto_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4) {
//	iq1=iq1*(-1);


    uint8_t data[8] = {0};
    data[0] = iq1 >> 8;
    data[1] = iq1;
    data[2] = iq2 >> 8;
    data[3] = iq2;
    data[4] = iq3 >> 8;
    data[5] = iq3;
    data[6] = iq4 >> 8;
    data[7] = iq4;
//    hcan->pTxMsg->StdId = 0x200;
//    hcan->pTxMsg->IDE = CAN_ID_STD;
//    hcan->pTxMsg->RTR = CAN_RTR_DATA;
//    hcan->pTxMsg->DLC = 0x08;
    Can_TxMessage(hcan, 0, 0x200, 8, (uint8_t *)data); //0代表标准正
//    HAL_CAN_Transmit(hcan, 1000);
}
void set_moto2_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4) //舵轮4个电机转向
{
//	iq1=iq1*(-1);


    uint8_t data[8] = {0};
    data[0] = iq1 >> 8;
    data[1] = iq1;
    data[2] = iq2 >> 8;
    data[3] = iq2;
    data[4] = iq3 >> 8;
    data[5] = iq3;
    data[6] = iq4 >> 8;
    data[7] = iq4;
//    hcan->pTxMsg->StdId = 0x200;
//    hcan->pTxMsg->IDE = CAN_ID_STD;
//    hcan->pTxMsg->RTR = CAN_RTR_DATA;
//    hcan->pTxMsg->DLC = 0x08;
    Can_TxMessage(hcan, 0, CAN_3510Moto_2_ID, 8, (uint8_t *)data); //0代表标准正
//    HAL_CAN_Transmit(hcan, 1000);
}
