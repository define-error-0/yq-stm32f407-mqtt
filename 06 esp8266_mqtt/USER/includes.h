#ifndef __INCLUDES_H__
#define __INCLUDES_H__



/* 标准C库*/
#include <stdio.h>	
#include <stdarg.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "stack_macros.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"

/* 外设相关 */
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "dht11.h"
#include "key.h"
#include "beep.h"
#include "esp8266.h"
#include "esp8266_mqtt.h"
#include "oled.h"
#include "pwm.h"
#include "MatrixKey.h"
#include "sfm.h"
#include "iwd.h"
#include "flash.h"



/* 宏定义 */
#define EVENT_GROUP_KEY1_DOWN		0x01
#define EVENT_GROUP_KEY2_DOWN		0x02
#define EVENT_GROUP_KEY3_DOWN		0x04
#define EVENT_GROUP_KEY4_DOWN		0x08
#define EVENT_GROUP_KEYALL_DOWN		0x0F



/* 互斥型信号量句柄 */
extern SemaphoreHandle_t g_mutex_printf;

/* 事件标志组句柄 */
extern EventGroupHandle_t g_event_group;	

/* 消息队列句柄 */
extern QueueHandle_t 	g_queue_esp8266;





/* 任务 */ 
extern void app_task_init			(void* pvParameters);   
extern void app_task_key			(void* pvParameters); 
extern void app_task_dht			(void* pvParameters); 
extern void app_task_usart			(void* pvParameters);  



/* 变量 */
extern volatile float g_temp;
extern volatile float g_humi;
extern volatile int g_flag_in;
extern uint8_t tmp;
extern volatile int g_door;

/* 函数 */
extern void dgb_printf_safe(const char *format, ...);

extern SemaphoreHandle_t g_sem_binary;


#endif

