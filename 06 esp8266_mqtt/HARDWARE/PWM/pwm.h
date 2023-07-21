#ifndef __PWM_H__
#define __PWM_H__

#include "stm32f4xx.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"

void sg_init(void);
void pwm_set(uint32_t duty);
void sg_angle(uint32_t angle);


#endif
