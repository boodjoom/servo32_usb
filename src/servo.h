/*
 * servo.h
 *
 *  Created on: 18 февр. 2014 г.
 *      Author: shark
 */

#include "stm32f10x.h"

#define ABS(a) ((a<0)?a*(-1):a)
#define TIM_PER_MS 100

typedef struct {
	GPIO_TypeDef * _port;
	uint32_t _RCC_AHBPeriph;
	uint16_t _pin;

	uint32_t _position;
	uint32_t _new_pos;
	uint32_t _pos_step;

	uint16_t _ms_max;
	uint16_t _ms_min;
} Servo;

void ServoInit(Servo * s);
void ServoSetPosMs(Servo * s,uint16_t new_pos_ms);
void ServoNextStep(Servo * s);


//Servo array

#define MAX_SERVO 32
#define USED_PORTS 5
#define OCR_MAX 20000

typedef struct
{
	Servo _array[MAX_SERVO];
	Servo* _sorted_array[MAX_SERVO];
	uint16_t _ocr_action[MAX_SERVO+1];
	GPIO_TypeDef * _port_action[USED_PORTS];
	uint16_t _pin_action[USED_PORTS][MAX_SERVO+1];
	int _servo_need_update;

} ServoArray;

void ServoArrayUpdate(ServoArray * sa);
void ServoArraySort(ServoArray * sa);
void ServoArraySetPosition(ServoArray * sa, uint8_t index, uint16_t position, uint16_t time);
void ServoArrayNextStep(ServoArray * sa, uint8_t index);
void ServoArrayInit(ServoArray * sa);
