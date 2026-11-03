#include "Correspondence_Task.h"
#include "Tasks.h"



#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "usbd_cdc_if.h"
#include "usb_device.h"
#ifdef __cplusplus
}
#endif

correspondence_ctrl Corres;

/* USER CODE BEGIN Header_Correspondence_Task */
/**
* @brief Function implementing the Correspondence thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Correspondence_Task */
void Correspondence_Task(void const * argument)
{
  /* USER CODE BEGIN Correspondence_Task */
	MX_USB_DEVICE_Init();
  /* Infinite loop */
  for(;;)
  {
		Corres.Corres_Feedback();
		Corres.Corres_Send();
    osDelay(1);
  }
  /* USER CODE END Correspondence_Task */
}

/* USER CODE BEGIN Header_led_flow_task */

void correspondence_ctrl::Corres_Init(void)
{

	
	
}

void correspondence_ctrl::Corres_Send(void)
{
	
	
	Can1_Send_Gimbal_IMU_Data();
	CDC_Transmit_FS( q  ,sizeof(q));
	
	
	
//	CAN_Cmd.SendData(&hcan1, CAN_MPU_TO_Z, &MPU_SZ, 8);
//	CAN_Cmd.SendData(&hcan1, CAN_MPU_TO_X_Y, &MPU_SXY, 8);S	
//		Serial_Send_MPU_DataZ();
//		osDelay(1);
//		Serial_Send_MPU_DataXY();
}

void correspondence_ctrl::Corres_Feedback(void)
{
//	Statistic_Update(xTaskGetTickCount());
		MPU_SZ.AngleZ=(int16_t)(INS.Yaw*UIN16_MPU_RAD);
		MPU_SZ.Speed_Z=(int16_t)(BMI088.Gyro_G[2]);
	
    MPU_SZ.Acce_Z=INS.MotionAccel_n[2];  //(int16_t)(BMI088.Accel_G[2]);
    MPU_SZ.Acce_Y=(int16_t)(BMI088.Accel_G[1]);

	  MPU_SXY.AngleX=(int16_t)(INS.Pitch*UIN16_MPU_RAD);
		MPU_SXY.Speed_X=(int16_t)(BMI088.Gyro_G[0]);
	  MPU_SXY.AngleY=(int16_t)(INS.Roll*UIN16_MPU_RAD);
		MPU_SXY.Speed_Y=(int16_t)(BMI088.Gyro_G[1]);
	
		MPU_DataZ.HHH=0xAA;
		MPU_DataZ.KEY=0xA5;
		MPU_DataZ.AngleZ.int_16=MPU_SZ.AngleZ;
		MPU_DataZ.Speed_Z.int_16=MPU_SZ.Speed_Z;
		MPU_DataZ.Acce_Z.int_16=MPU_SZ.Acce_Z;
		MPU_DataZ.Acce_Y.int_16=MPU_SZ.Acce_Y;
		
		MPU_DataXY.HHH=0xAA;
		MPU_DataXY.KEY=0xA6;
		MPU_DataXY.AngleX.int_16=MPU_SXY.AngleX;
		MPU_DataXY.Speed_X.int_16=MPU_SXY.Speed_X;
		MPU_DataXY.AngleY.int_16=MPU_SXY.AngleY;
		MPU_DataXY.Speed_Y.int_16=MPU_SXY.Speed_Y;
		
		q[0] = 0x21;
		q[1] = ((int16_t)(INS.q[0] * 1000) >> 8);
		q[2] =  (int16_t)(INS.q[0] * 1000) ;
		q[3] = ((int16_t)(INS.q[1] * 1000) >> 8);
		q[4] =  (int16_t)(INS.q[1] * 1000) ;
		q[5] = ((int16_t)(INS.q[2] * 1000) >> 8);
    q[6] =  (int16_t)(INS.q[2] * 1000) ;
		q[7] = ((int16_t)(INS.q[3] * 1000) >> 8);
		q[8] =  (int16_t)(INS.q[3] * 1000) ;
		q[9] = Get_CRC8_Check_Sum(q, 9, 0xff);
		
		
		
		
}
void correspondence_ctrl::Serial_Send_MPU_DataXY(void)
{
	Gyro_Buffer[0]=MPU_DataXY.HHH;
	Gyro_Buffer[1]=MPU_DataXY.KEY;
	Gyro_Buffer[2]=MPU_DataXY.AngleX.uint_8[0];
	Gyro_Buffer[3]=MPU_DataXY.AngleX.uint_8[1];
	Gyro_Buffer[4]=MPU_DataXY.Speed_X.uint_8[0];
	Gyro_Buffer[5]=MPU_DataXY.Speed_X.uint_8[1];
	Gyro_Buffer[6]=MPU_DataXY.AngleY.uint_8[0];
	Gyro_Buffer[7]=MPU_DataXY.AngleY.uint_8[1];
	Gyro_Buffer[8]=MPU_DataXY.Speed_Y.uint_8[0];
	Gyro_Buffer[9]=MPU_DataXY.Speed_Y.uint_8[1];
	//HAL_UART_Transmit(&huart1,Gyro_Buffer,10,1);
	//while(!( __HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)==SET)){osDelay(1);};
}	
void correspondence_ctrl::Serial_Send_MPU_DataZ(void)
{
	Gyro_Buffer[0]=MPU_DataZ.HHH;
	Gyro_Buffer[1]=MPU_DataZ.KEY;
	Gyro_Buffer[2]=MPU_DataZ.AngleZ.uint_8[0];
	Gyro_Buffer[3]=MPU_DataZ.AngleZ.uint_8[1];
	Gyro_Buffer[4]=MPU_DataZ.Speed_Z.uint_8[0];
	Gyro_Buffer[5]=MPU_DataZ.Speed_Z.uint_8[1];
	Gyro_Buffer[6]=MPU_DataZ.Acce_Z.uint_8[0];
	Gyro_Buffer[7]=MPU_DataZ.Acce_Z.uint_8[1];
	Gyro_Buffer[8]=MPU_DataZ.Acce_Y.uint_8[0];
	Gyro_Buffer[9]=MPU_DataZ.Acce_Y.uint_8[1];
	//HAL_UART_Transmit(&huart1,Gyro_Buffer,10,1);
	
}

void correspondence_ctrl::Can1_Send_Gimbal_IMU_Data(void)
{
	Can_Gimbal_IMU_Buffer[0] = MPU_DataXY.AngleY.uint_8[0];
	Can_Gimbal_IMU_Buffer[1] = MPU_DataXY.AngleY.uint_8[1];
	Can_Gimbal_IMU_Buffer[2] = MPU_DataXY.Speed_Y.uint_8[0];
	Can_Gimbal_IMU_Buffer[3] = MPU_DataXY.Speed_Y.uint_8[1];
	Can_Gimbal_IMU_Buffer[4] = MPU_DataZ.AngleZ.uint_8[0];
	Can_Gimbal_IMU_Buffer[5] = MPU_DataZ.AngleZ.uint_8[1];
	Can_Gimbal_IMU_Buffer[6] = MPU_DataZ.Speed_Z.uint_8[0];
	Can_Gimbal_IMU_Buffer[7] = MPU_DataZ.Speed_Z.uint_8[1];
	
	CAN_TxHeaderTypeDef TxHeader;  // CAN 发送报文头
	//uint8_t TxData[8];             // CAN 数据（最多 8 字节）
	//HAL_StatusTypeDef TxStatus;    // 发送状态
	uint32_t TxMailbox;            // 发送邮箱编号（用于跟踪发送状态）
	
	TxHeader.StdId = 0x01;                // 标准帧 ID（11 位）
	TxHeader.ExtId = 0;                    // 扩展帧 ID（标准帧设为 0）
	TxHeader.RTR = CAN_RTR_DATA;           // 数据帧（非远程帧）
	TxHeader.IDE = CAN_ID_STD;             // 标准帧（若为扩展帧，设为 CAN_ID_EXT）
	TxHeader.DLC = 8;                      // 数据长度（0-8）
	TxHeader.TransmitGlobalTime = DISABLE; // 不发送全局时间（可选）
	
	//HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Can_Gimbal_IMU_Buffer, &TxMailbox);
	HAL_CAN_AddTxMessage(&hcan2, &TxHeader, Can_Gimbal_IMU_Buffer, &TxMailbox);
	
	
	
	
	
	
	
}



