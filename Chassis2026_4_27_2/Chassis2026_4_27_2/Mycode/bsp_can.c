#include "bsp_can.h"
#include "dm_imu.h"


/*  配置过滤器  hcan1 hcan2  */
void CAN_FilterInit()
{
	/*  配置过滤器  hcan1 hcan2  */


    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;																					// 过滤器索引1
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;															// CAN1 (CAN主) 和 CAN2 （CAN从）过滤器分界
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);


//	CAN_FilterTypeDef filter;
//	
//	filter.FilterActivation=ENABLE;
//	filter.FilterBank=0;
//	filter.FilterFIFOAssignment=CAN_FilterFIFO0;
//	filter.FilterIdHigh=0x0000;
//	filter.FilterIdLow=0x0000;
//	filter.FilterMaskIdHigh=0x0000;
//	filter.FilterMaskIdLow=0x0000;
//	filter.FilterMode=CAN_FILTERMODE_IDMASK;
//	filter.FilterScale=CAN_FILTERSCALE_32BIT;
//	
//	filter.SlaveStartFilterBank=14;
//	
//	HAL_CAN_ConfigFilter(&hcan1,&filter);
//	
//	filter.FilterActivation=ENABLE;
//	filter.FilterBank=14;
//	filter.FilterFIFOAssignment=CAN_FilterFIFO0;
//	filter.FilterIdHigh=0x0000;
//	filter.FilterIdLow=0x0000;
//	filter.FilterMaskIdHigh=0x0000;
//	filter.FilterMaskIdLow=0x0000;
//	filter.FilterMode=CAN_FILTERMODE_IDMASK;
//	filter.FilterScale=CAN_FILTERSCALE_32BIT;
//	
//	HAL_CAN_ConfigFilter(&hcan2,&filter);
}

CAN_RxHeaderTypeDef rx_header;
uint8_t pdata[8];
/*  CAN接收回调函数  */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//CAN_RxHeaderTypeDef rx_header;
	//uint8_t pdata[8];
	if(hcan->Instance==CAN1)
	{
		HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,pdata);
		
		if(rx_header.StdId==0x11)
			IMU_UpdateData(pdata);
	}
	
	if(hcan->Instance==CAN2)
	{
		HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,pdata);
		
		if(rx_header.StdId==0x11)
			IMU_UpdateData(pdata);
	}
}


