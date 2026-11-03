#ifndef __DRIVERS_SERIAL_H
#define __DRIVERS_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "drivers_buffer.h"
//#include "platform.h"

#ifdef __cplusplus
}
#endif

#define SERIAL_Config_Default SERIAL_8N1
#define USART_ITPending_Default USART_IT_RXNE
#define USART_IT_RXNE_AND_IDLE (USART_IT_IDLE << 16 | USART_IT_RXNE)
#define USART_PreemptionPriority_Default 10
#define USART_SubPriority_Default 1

#define USART_GetWordLength(SERIAL_x) ((uint16_t)(SERIAL_x & 0xF000))
#define USART_GetParity(SERIAL_x) ((uint16_t)(SERIAL_x & 0x0F00))
#define USART_GetStopBits(SERIAL_x) ((uint16_t)((SERIAL_x & 0x00F0) << 8))



#endif /* __DRIVERS_SERIAL_H */
