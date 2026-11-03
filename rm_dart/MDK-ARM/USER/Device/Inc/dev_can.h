#ifndef __DEVICE_CAN_H
#define __DEVICE_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "can.h"


#ifdef __cplusplus
}
#endif



	
	typedef struct
{
  uint32_t StdId;  /*!< Specifies the standard identifier.
                        This parameter can be a value between 0 to 0x7FF. */

  uint32_t ExtId;  /*!< Specifies the extended identifier.
                        This parameter can be a value between 0 to 0x1FFFFFFF. */

  uint8_t IDE;     /*!< Specifies the type of identifier for the message that 
                        will be received. This parameter can be a value of 
                        @ref CAN_identifier_type */

  uint8_t RTR;     /*!< Specifies the type of frame for the received message.
                        This parameter can be a value of 
                        @ref CAN_remote_transmission_request */

  uint8_t DLC;     /*!< Specifies the length of the frame that will be received.
                        This parameter can be a value between 0 to 8 */

  uint8_t Data[8]; /*!< Contains the data to be received. It ranges from 0 to 
                        0xFF. */

  uint8_t FMI;     /*!< Specifies the index of the filter the message stored in 
                        the mailbox passes through. This parameter can be a 
                        value between 0 to 0xFF */
} CanRx_ctrl;

class CANctrl
{
public:
	CANctrl(CAN_HandleTypeDef *hcanx);
  typedef void(*CAN_CallbackFunction_t)(CanRx_ctrl *Rx_Message);

	void attachInterrupt(CAN_CallbackFunction_t Function);

	void ChangeID(uint16_t StdID);

	void SendData(const void *buf, uint8_t len);
	void IRQHandler(void);
protected:
		CAN_HandleTypeDef *hcanx;
	  CAN_CallbackFunction_t CAN_Function;
	  CanRx_ctrl Rx_Message;
private:
	uint32_t StdId;
};

extern CANctrl CAN1_Ctrl;
extern CANctrl CAN2_Ctrl;

extern void CAN_ALL_Init();
extern void Prefence_Init(void);


#endif /* __DEVICE_CAN_H */
