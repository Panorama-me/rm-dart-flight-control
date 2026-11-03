#include "dev_can.h"
#include "Message_Task.h"
#include "tasks.h"

CANctrl CAN1_Ctrl(&hcan1);
CANctrl CAN2_Ctrl(&hcan2);

CANctrl::CANctrl( CAN_HandleTypeDef *hcanx )
{
    this->hcanx = hcanx;
}
void CANctrl::attachInterrupt(CAN_CallbackFunction_t Function)
{
    this->CAN_Function = Function;
}
void CANctrl::ChangeID(uint16_t StdID)
{
    this->StdId = StdID;
}

void CANctrl::SendData(const void *buf, uint8_t len)//，全部CAN都是从这里发送的
{
	  uint32_t send_mail_box;
    uint8_t *ch = (uint8_t *)buf;
		uint8_t  chassis_can_send_data[8];

	  CAN_TxHeaderTypeDef  TxMessage;
    TxMessage.StdId = StdId;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    while(len--)
    {
        chassis_can_send_data[len] = ch[len];
    }
    HAL_CAN_AddTxMessage(hcanx, &TxMessage,chassis_can_send_data,&send_mail_box);
}
void CANctrl::IRQHandler(void)
{
	   CAN_RxHeaderTypeDef rx_header;
     HAL_CAN_GetRxMessage(hcanx, CAN_RX_FIFO0, &rx_header, Rx_Message.Data);
	
     Rx_Message.StdId=rx_header.StdId;
	   Rx_Message.ExtId=rx_header.ExtId;
     Rx_Message.IDE=rx_header.IDE;
     Rx_Message.RTR=rx_header.RTR;
	   Rx_Message.DLC=rx_header.DLC;
     Rx_Message.FMI=rx_header.FilterMatchIndex;
	
		if(CAN_Function)
		{
				CAN_Function(&Rx_Message);
		}    
}

extern"C"
{

	void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
	{
		if(hcan->Instance==CAN1)
	    CAN1_Ctrl.IRQHandler();
		if(hcan->Instance==CAN2)
      CAN2_Ctrl.IRQHandler();
	}
}

