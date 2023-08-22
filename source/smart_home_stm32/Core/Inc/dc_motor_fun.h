#ifndef _DC_MOTOR_
#define _DC_MOTOR_

#include "stm32f1xx_hal.h"
void pwm_DC_motor(float temp);
void stepCCV (int steps, uint16_t timer_delay);
void stepCV (int steps, uint16_t timer_delay);
void delay(uint16_t time);
#endif