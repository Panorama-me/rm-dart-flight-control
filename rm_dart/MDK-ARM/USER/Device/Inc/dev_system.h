/*
 * MIT License
 * Copyright (c) 2019 _VIFEXTech
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef __DEV_SYSTEM_H
#define __DEV_SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

//#include "platform.h"

#ifdef __cplusplus
}
#endif

#include "stdint.h"


#include "stdlib.h"
#include "stdbool.h"

//#include "app_preference.h"

#define PI 3.1415926535897932384626433832795f
#define HALF_PI 1.5707963267948966192313216916398f
#define TWO_PI 6.283185307179586476925286766559f
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f
#define EULER 2.718281828459045235360287471352f
#define PI_TO_ECD 1303.7972938088065906186957895476f
#define ECD_TO_PI 0.00076699039394282061485904379474597f
#define sin45 0.70710678118654752440084436210485f

// #define SERIAL  0x0
#define DISPLAY 0x1

#define LSBFIRST 0x0
#define MSBFIRST 0x1

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define ABS(x)  (((x)>0)?(x):-(x))//abs(x) is define in stdlib.h
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x)        ((x)*(x))

#define lowByte(w)  ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit)  ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif 

#define clockCyclesPerMicrosecond()  ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (F_CPU / 1000L) )
#define microsecondsToClockCycles(a) ( (a) * (F_CPU / 1000000L) )

#define delay(ms)             delay_ms(ms)
#define delayMicroseconds(us) delay_us(us)

#define sei()          __set_PRIMASK(0)
#define cli()          __set_PRIMASK(1)
#define interrupts()   sei()
#define noInterrupts() cli()

#define analogInPinToBit(Pin)       (Pin)
#define digitalPinToInterrupt(Pin)  (Pin)
#define digitalPinToPort(Pin)       (PIN_MAP[Pin].GPIOx)
#define digitalPinToBitMask(Pin)    (PIN_MAP[Pin].GPIO_Pin_x)
#define portInputRegister(Port)     (&((Port)->IDR))
#define portOutputRegister(Port)    (&((Port)->ODR))

#define digitalWrite_HIGH(Pin) (GPIO_HIGH  (PIN_MAP[Pin].GPIOx,PIN_MAP[Pin].GPIO_Pin_x))
#define digitalWrite_LOW(Pin)  (GPIO_LOW   (PIN_MAP[Pin].GPIOx,PIN_MAP[Pin].GPIO_Pin_x))
#define digitalRead_FAST(Pin)  (GPIO_READ  (PIN_MAP[Pin].GPIOx,PIN_MAP[Pin].GPIO_Pin_x))
#define togglePin(Pin)         (GPIO_TOGGLE(PIN_MAP[Pin].GPIOx,PIN_MAP[Pin].GPIO_Pin_x))

#define NOT_A_PIN 0
#define NOT_A_PORT 0
#define NOT_AN_INTERRUPT -1

#define boolean bool
typedef unsigned char byte;
typedef float fp32;

typedef enum { LOW = 0, HIGH = !LOW } GPIO_State_Type;
typedef enum { Off = 0, On = !Off } _Switch_Type;
typedef enum { OFF = 0, ON = !OFF } _SWITCH_Type;

/******************************全局使用枚举******************************/

typedef enum DEV_WORK_STATE
{
	DEV_OFFLINE = 0,
	DEV_ONLINE,
	
	DEV_NO = 0,
	DEV_OK,
}Dev_Work_State_e;

/**
 * @brief 未初始化：DEV_RESET_NO 初始化完成:DEV_RESET_OK
 * 
 */
typedef enum DEV_RESET_STATE
{
	DEV_RESET_NO,
	DEV_RESET_OK,
}Dev_Reset_State_e;

/* 状态灯枚举 */
typedef enum LED_NUM
{
	LED_ORANGE,            
	LED_Blue,              
	LED_RED,
	LED_GREEN,
}LED_Num;

/* 状态灯枚举 */
typedef enum CAR_NAME
{
	LEFT=0,            
	RIGHT,          
	
	FRONT=0,
	BACK,
	
	PITCH=0,
	YAW,
	ROLL,
	
	DEFFL=0,
	DEFFR=2,
	
	GPITCH=2,
}Car_Name;


//void pinMode(uint8_t Pin, pinMode_TypeDef pinMode_x);
void digitalWrite(uint8_t Pin, uint8_t val);
uint8_t digitalRead(uint8_t Pin);
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t value);
uint32_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint32_t bitOrder);
uint32_t pulseIn(uint32_t Pin, uint32_t State, uint32_t Timeout);

template<typename T> T fmap(T x, T in_min, T in_max, T out_min, T out_max);

#endif /* __DEV_SYSTEM_H */
