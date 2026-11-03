#ifndef Correspondence_TASK_H
#define Correspondence_TASK_H

#include "drivers_statistic.h"
#include "dev_system.h"
//#include "usart.h"
#include "can.h"


#ifdef __cplusplus
extern "C" {
#endif


#include "crc_protocol.h"
void Correspondence_Task(void const * argument);

#ifdef __cplusplus
}
#endif

#define UIN16_MPU_RAD 182
//#define UIN16_MPU_RAD 10430
#define UIN16_MPU_G 557

//#define ANGLE_TO_RAD   0.01745329251994329576923690768489
//#define ANGLE_TO_RAD   57.295779513082320876798154814105

union F
{
    uint8_t I[4];
    fp32 F;
};

union int16_t_uint8_t
{
    uint8_t uint_8[2];
    int16_t int_16;
};

struct MPU_Data_t_Z
{
	    int16_t AngleZ;
	    int16_t Speed_Z;
      int16_t Acce_Z;
      int16_t Acce_Y;

};

struct MPU_Data_t_XY
{
			int16_t AngleX;
			int16_t Speed_X;

			int16_t AngleY;
			int16_t Speed_Y;	
};

struct MPU_Data_tXY
{
			uint8_t HHH;
			uint8_t KEY;
			int16_t_uint8_t AngleX;
			int16_t_uint8_t Speed_X;
			int16_t_uint8_t AngleY;
			int16_t_uint8_t Speed_Y;
};
struct MPU_Data_tZ
{
			uint8_t HHH;
			uint8_t KEY;
	    int16_t_uint8_t AngleZ;
	    int16_t_uint8_t Speed_Z;
      int16_t_uint8_t Acce_Z;
      int16_t_uint8_t Acce_Y;
};
struct Can_Gimbal_IMU_Data_t
{
	int16_t_uint8_t Pitch_Angle;
	int16_t_uint8_t Pitch_Gyro;
	int16_t_uint8_t Yaw_Angle;
	int16_t_uint8_t Yaw_Gyro;
	

};

class correspondence_ctrl:public Statistic
{
public:
    correspondence_ctrl()
    {}

    void Corres_Init(void);
    void Corres_Send(void);
    void Corres_Feedback(void);
		void Serial_Send_MPU_DataXY(void);
    void Serial_Send_MPU_DataZ(void);
			
		//can发送云台陀螺仪数据
		void Can1_Send_Gimbal_IMU_Data(void);
			
private:
	MPU_Data_t_Z  	  MPU_SZ;
	MPU_Data_t_XY   	MPU_SXY;
	MPU_Data_tZ  			MPU_DataZ;
  MPU_Data_tXY		  MPU_DataXY;
	


  uint8_t Gyro_Buffer[10];

	uint8_t q[10];


/*
struct Can_Gimbal_IMU_Data_t
{
	int16_t_uint8_t Pitch_Angle;
	int16_t_uint8_t Pitch_Gyro;
	int16_t_uint8_t Yaw_Angle;
	int16_t_uint8_t Yaw_Gyro;
	

};
*/
	uint8_t Can_Gimbal_IMU_Buffer[8];


};

extern correspondence_ctrl Corres;















#endif
