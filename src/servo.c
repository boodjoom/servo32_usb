/*
 * servo.c
 *
 *  Created on: 18 февр. 2014 г.
 *      Author: shark
 */

#include "servo.h"
#include <string.h>
#include <stdlib.h>


#define SIGN(a) (a>0?1:(-1))


void ServoInit(Servo * s) {
	//s._port = 0;
	//s._RCC_AHBPeriph = 0;
	//s._pin = 0;

	s->_position = 1472;
	s->_new_pos = 1472;
	s->_pos_step = 0;

	s->_ms_max = 2400;
	s->_ms_min = 600;

	RCC_APB2PeriphClockCmd(s->_RCC_AHBPeriph, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = s->_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(s->_port, &GPIO_InitStructure);

}

void ServoSetPosMs(Servo * s, uint16_t new_pos_ms) {

	if(new_pos_ms< s->_ms_min || new_pos_ms>s->_ms_max)
		return;
	s->_new_pos = (uint32_t)(new_pos_ms * TIM_PER_MS);

}

void ServoNextStep(Servo * s) {

	int16_t dpos;
	if(s->_new_pos!=s->_position)
	{
		if(s->_pos_step==0)
		{
			s->_position=s->_new_pos;
		}
		else
		{
			dpos=s->_new_pos-s->_position;
			if(abs(dpos)<s->_pos_step)
				s->_position=s->_new_pos;
			else
				s->_position+=(dpos>0?1:(-1))*s->_pos_step;
		}
	}
}


//Servo array

void ServoArrayUpdate(ServoArray * sa) {

	int i,j,k,n;

	for(j=0;j<USED_PORTS;j++)
    for(i=0;i<MAX_SERVO+1;i++)
        sa->_pin_action[j][i]=0;

	for(i=0,k=0;i<MAX_SERVO;i++,k++)
	{
		sa->_ocr_action[k] = sa->_sorted_array[i]->_position;			// Записываем их как есть
		for(n=0;n<USED_PORTS;n++)
			if(sa->_sorted_array[i]->_port == sa->_port_action[n])
				sa->_pin_action[n][k+1] = sa->_sorted_array[i]->_pin;

    if(i<MAX_SERVO-1)
    {
      if(abs(sa->_sorted_array[i]->_position - sa->_sorted_array[i+1]->_position)<6)	//Если значения уникальные								// Но если совпадает со следующим
      {
         // И в цикле ищем все аналогичные позиции, склеивая их битмаски в одну.

        for(j=1;(i+j<MAX_SERVO);j++)
        {
          if(abs(sa->_sorted_array[i]->_position - sa->_sorted_array[i+j]->_position)>5)
            break;
          for(n=0;n<USED_PORTS;n++)
            if(sa->_sorted_array[i+j]->_port == sa->_port_action[n])
              sa->_pin_action[n][k+1] |= sa->_sorted_array[i+j]->_pin;
        }
        i+=j-1;	 					// Перед выходом корректируем индекс
      }
	 	}					// На глубину зарывания в повторы
	}


	sa->_ocr_action[k] = OCR_MAX;					// В последний элемент вписываем заглушку FF.
	//sa._servo_need_update=0;

}

void ServoArraySort(ServoArray * sa) {
	int i,k;
	Servo *tmp;

	// Сортируем массив указателей.
	for(i=1;i<MAX_SERVO;i++)
	    {
	    for(k=i;((k>0)&&(sa->_sorted_array[k]->_position < sa-> _sorted_array[k-1]->_position));k--)
	        {
	        tmp = sa->_sorted_array[k];					// Swap [k,k-1]
	        sa->_sorted_array[k]=sa->_sorted_array[k-1];
	        sa->_sorted_array[k-1]=tmp;
			}

		}
}

void ServoArraySetPosition(ServoArray * sa, uint8_t index, uint16_t position, uint16_t time) {
	sa->_servo_need_update = 0;
	sa->_array[index]._new_pos=position;
	if(time==0)
	{
	  sa->_array[index]._position=sa->_array[index]._new_pos;
		sa->_array[index]._pos_step=0;
	}
	else
	{
		sa->_array[index]._pos_step=1+abs(sa->_array[index]._new_pos-sa->_array[index]._position)*20/time;
	}
	//ServoNextStep(index);
	//Servo_sort();
	//char s[20];
	//int i;
	//for(i=0;i<20;i++)
  //  s[i]=0;
  //sprintf(s,"pos of %d is %d",index,position);
  //putsu(s);
	sa->_servo_need_update = 1;
}

void ServoArrayNextStep(ServoArray * sa, uint8_t index) {
    int i;
    if(index<MAX_SERVO)
    	ServoNextStep(&sa->_array[index]);
    else
		for(i=0;i<MAX_SERVO;i++)
		{
			ServoNextStep(&sa->_array[i]);
		}

}

void ServoArrayInit(ServoArray * sa) {
	sa->_servo_need_update=0;
	int i;
	//GPIO_TypeDef * port=0;
	//uint16_t pin;
	for(i=0;i<MAX_SERVO+1;i++)
    sa->_ocr_action[i]=0;
	for(i=0;i<MAX_SERVO;i++)
	{
		ServoInit(&sa->_array[i]);
		sa->_sorted_array[i]=&sa->_array[i];
	}

	ServoArraySort(sa);
	ServoArrayUpdate(sa);

}
