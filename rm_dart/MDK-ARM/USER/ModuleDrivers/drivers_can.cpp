#include "drivers_can.h"

CANdev::CANdev( CAN_HandleTypeDef *CANx )
{
    this->CANx = CANx;
	  this->CAN_Function = 0;
}
