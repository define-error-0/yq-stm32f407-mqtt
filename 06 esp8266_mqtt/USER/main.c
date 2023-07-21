#include "includes.h"

/*------------------------------------任务空间句柄初始化----------------------------------------*/
/*任务控制块指针*/
static TaskHandle_t app_task_init_handle      = NULL;			
static TaskHandle_t g_app_task_key_handle     = NULL;
static TaskHandle_t g_app_task_mqtt_handle 	  = NULL;
static TaskHandle_t g_app_task_esp8266_handle = NULL;
static TaskHandle_t g_app_task_monitor_handle = NULL;
static TaskHandle_t g_app_task_oled_handle    = NULL;
static TaskHandle_t g_app_task_steer_handle   = NULL;
static TaskHandle_t g_app_task_key16_handle   = NULL;
static TaskHandle_t g_app_task_finger_handle  = NULL;
static TaskHandle_t g_app_task_flash_handle   = NULL; 
 
/* 任务1:初始化 */ 
static void app_task_init(void* pvParameters);  

/* 任务2:按键 */  
static void app_task_key(void* pvParameters); 

/* 任务3:mqtt控制反馈 */  
static void app_task_mqtt(void* pvParameters); 

/* 任务4:无线WiFi模块-esp8266*/  
static void app_task_esp8266(void* pvParameters); 

/* 任务5:监控任务*/  
static void app_task_monitor(void* pvParameters); 

/* 任务6:OLED任务*/  
static void app_task_oled(void* pvParameters); 

/* 任务7:舵机门任务*/  
static void app_task_steer(void* pvParameters); 

/* 任务8:矩阵键盘4x4任务*/  
static void app_task_key16(void* pvParameters); 

/* 任务9:指纹模块任务*/  
static void app_task_finger(void* pvParameters); 

/* 任务10:flash读写任务*/  
static void app_task_flash(void* pvParameters); 


/*-----------------------------------------全局变量初始化-----------------------------------------*/
/* 互斥型信号量句柄 */
static SemaphoreHandle_t g_mutex_printf;

/* 事件标志组句柄 */
EventGroupHandle_t g_event_group;	

/* 消息队列句柄 */
QueueHandle_t g_queue_esp8266;			

/*信号量*/
SemaphoreHandle_t g_sem_binary;						

static volatile uint32_t g_esp8266_init=0;			//初始化变量
volatile float g_temp=0.0;							//温度
volatile float g_humi=0.0;							//湿度变量
volatile int g_door=0;								//门状态
volatile int g_flag_connect_server=0;				//连接服务器变量
volatile int g_flag_in=0;							//登录模式
volatile int g_flag_password_re=0;					//修改密码
volatile int g_flag_password_look=0;				//查看密码
uint8_t tmp=0;										//临时键盘数据
char  password[18]={0};							    //键盘登录密码
char flash_buf[18]={0};							    //flash的接收buf


/*----------------------------------------其他文件变量声明----------------------------------------*/
/*图像BMP及字库*/
extern unsigned char BMP1[];
extern unsigned char BMP2[];
extern unsigned char BMP3[];
extern unsigned char BMP4[];
extern unsigned char BMP5[];
extern unsigned char BMP6[];
extern unsigned char BMP7[];
extern const unsigned char F6x8[][6];
extern const unsigned char F8X16[];
extern char Hzk[][32];							

/*-------------------------------------------宏定义-----------------------------------------------*/
#define PASSWORD_INIT 				0  				//密码初始化
#define DEBUG_dgb_printf_safe_EN	1  				//开启打印开关
#define PASSWORD_ADDR 				0x000000		//密码位置
#define PASSWORD_PRIMARY  			"123456#"		//原初密码
#define PASSWORD_PASSWORD_LOOK      1				//复位显示密码开关	
#define STREE_OPEN                  1               //舵机开关
#define STREE_LED_OPEN				0				//灯光开关
#define KEY_EXIT					1				//按键定时中断开关	
#define BEER						1				//蜂鸣器开关		安静操作
//configUSE_TIMERS 									/*定时器开关*/
//configSUPPORT_DYNAMIC_ALLOCATION 


/*-----------------------------------------重定向函数---------------------------------------------*/
void dgb_printf_safe(const char *format, ...)
{
#if DEBUG_dgb_printf_safe_EN	

	va_list args;
	va_start(args, format);
	
	/* 获取互斥信号量 */
	xSemaphoreTake(g_mutex_printf,portMAX_DELAY);
	
	vprintf(format, args);
			
	/* 释放互斥信号量 */
	xSemaphoreGive(g_mutex_printf);	

	va_end(args);
#else
	(void)0;										//防止错误
#endif
}

/*------------------------------------------主程序------------------------------------------------*/
int main(void)
{
		
	/* 建立信号量 */
	vSemaphoreCreateBinary(g_sem_binary);
	
	/* 设置系统中断优先级分组4 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	/* 系统定时器中断频率为configTICK_RATE_HZ */
	SysTick_Config(SystemCoreClock/configTICK_RATE_HZ);										
	
	/* 初始化串口1 */
	usart1_init(9600);    

	/* 创建app_task_init任务 */
	xTaskCreate((TaskFunction_t )app_task_init,  		/* 任务入口函数 */
			  (const char*    )"app_task_init",			/* 任务名字 */
			  (uint16_t       )512,  					/* 任务栈大小 */
			  (void*          )NULL,					/* 任务入口函数参数 */
			  (UBaseType_t    )5, 						/* 任务的优先级 */
			  (TaskHandle_t*  )&app_task_init_handle);	/* 任务控制块指针 */ 

	
	/* 开启任务调度 */
	vTaskStartScheduler(); 
	
	while(1);
}


/*----------------------------------------任务创建程序--------------------------------------------*/
static void app_task_init(void* pvParameters)
{
	/* led初始化 */
	led_init();
	
	#if	BEER
	/* 蜂鸣器初始化 */	
	beep_init();
	#endif
	
	/* 按键初始化 */
	key_init();
	
	/*初始化OLED*/
	OLED_Init();
	
	/*初始化温湿度*/
	dht11_init();
	
	/*pwm初始化*/
	sg_init();
	
	/*初始化键盘*/
	key_board_init();
	
	/*初始化温湿度*/
	dht11_init();
	
	/*FLASH初始化*/
	Flash_config();
	
	/*开启看门狗 */
	IWDG_Start(); 

	/* 创建互斥锁 */	  
	g_mutex_printf=xSemaphoreCreateMutex();
				  
	/* 创建事件标志组 */
	g_event_group=xEventGroupCreate();

	/* 创建消息队列 */
	g_queue_esp8266=xQueueCreate(3, sizeof(g_esp8266_rx_buf));	
	
	/* 创建app_task_key任务 */		  
	xTaskCreate((TaskFunction_t )app_task_key,  		/* 任务入口函数 */
			  (const char*    )"app_task_key",			/* 任务名字 */
			  (uint16_t       )128,  					/* 任务栈大小 */
			  (void*          )NULL,					/* 任务入口函数参数 */
			  (UBaseType_t    )5, 						/* 任务的优先级 */
			  (TaskHandle_t*  )&g_app_task_key_handle);	/* 任务控制块指针 */
			  
	/* 创建app_task_mqtt任务 */		  
	xTaskCreate((TaskFunction_t )app_task_mqtt,  		/* 任务入口函数 */
			  (const char*    )"app_task_mqtt",		/* 任务名字 */
			  (uint16_t       )512,  					/* 任务栈大小 */
			  (void*          )NULL,					/* 任务入口函数参数 */
			  (UBaseType_t    )5, 						/* 任务的优先级 */
			  (TaskHandle_t*  )&g_app_task_mqtt_handle);	/* 任务控制块指针 */	

	/* 创建app_task_oled任务 */		  
	xTaskCreate((TaskFunction_t )app_task_oled,  		/* 任务入口函数 */
			  (const char*    )"app_task_oled",			/* 任务名字 */
			  (uint16_t       )128,  						/* 任务栈大小 */
			  (void*          )NULL,						/* 任务入口函数参数 */
			  (UBaseType_t    )5, 							/* 任务的优先级 */
			  (TaskHandle_t*  )&g_app_task_oled_handle);		/* 任务控制块指针 */			  	  
			  
	/* 创建app_task_esp8266任务 */		  
	xTaskCreate((TaskFunction_t )app_task_esp8266,  		/* 任务入口函数 */
			  (const char*    )"app_task_esp8266",		/* 任务名字 */
			  (uint16_t       )1024,  					/* 任务栈大小 */
			  (void*          )NULL,					/* 任务入口函数参数 */
			  (UBaseType_t    )5, 						/* 任务的优先级 */
			  (TaskHandle_t*  )&g_app_task_esp8266_handle);	/* 任务控制块指针 */				  
			  
	/* 创建app_task_monitor任务 */		  
	xTaskCreate((TaskFunction_t )app_task_monitor,  		/* 任务入口函数 */
			  (const char*    )"app_task_monitor",			/* 任务名字 */
			  (uint16_t       )512, 						/* 任务栈大小 */
			  (void*          )NULL,						/* 任务入口函数参数 */
			  (UBaseType_t    )5, 							/* 任务的优先级 */
			  (TaskHandle_t*  )&g_app_task_monitor_handle);		/* 任务控制块指针 */	

	/* 创建app_task_steer和喂狗任务 */		  
	xTaskCreate((TaskFunction_t )app_task_steer,  		/* 任务入口函数 */
			  (const char*    )"app_task_steer",			/* 任务名字 */
			  (uint16_t       )128,  						/* 任务栈大小 */
			  (void*          )NULL,						/* 任务入口函数参数 */
			  (UBaseType_t    )10, 							/* 任务的优先级 */			//因为还有喂狗，所以优先级高一点，一行代码不用开新任务
			  (TaskHandle_t*  )&g_app_task_steer_handle);		/* 任务控制块指针 */	
	  		  
	/* 创建app_task_key16任务 */		  
	xTaskCreate((TaskFunction_t )app_task_key16,  		/* 任务入口函数 */
			  (const char*    )"app_task_key16",			/* 任务名字 */
			  (uint16_t       )128,  						/* 任务栈大小 */
			  (void*          )NULL,						/* 任务入口函数参数 */
			  (UBaseType_t    )5, 							/* 任务的优先级 */
			  (TaskHandle_t*  )&g_app_task_key16_handle);		/* 任务控制块指针 */	
			  
	/* 创建app_task_finger任务 */		  
	xTaskCreate((TaskFunction_t )app_task_finger,  		/* 任务入口函数 */
			  (const char*    )"app_task_finger",			/* 任务名字 */
			  (uint16_t       )128,  						/* 任务栈大小 */
			  (void*          )NULL,						/* 任务入口函数参数 */
			  (UBaseType_t    )5, 							/* 任务的优先级 */
			  (TaskHandle_t*  )&g_app_task_finger_handle);		/* 任务控制块指针 */	
	
	/* 创建app_task_flash任务 */		  
	xTaskCreate((TaskFunction_t )app_task_flash,  		/* 任务入口函数 */
			  (const char*    )"app_task_flash",			/* 任务名字 */
			  (uint16_t       )128,  						/* 任务栈大小 */
			  (void*          )NULL,						/* 任务入口函数参数 */
			  (UBaseType_t    )5, 							/* 任务的优先级 */
			  (TaskHandle_t*  )&g_app_task_flash_handle);		/* 任务控制块指针 */	
			  
	vTaskDelete(NULL);		  
}   



/*----------------------------------------任务函数定义--------------------------------------------*/
static void app_task_key(void* pvParameters)
{
	EventBits_t EventValue=0;								//键值
	
	dgb_printf_safe("app_task_key create success\r\n");		

	for(;;)
	{
		#if KEY_EXIT	
		//等待事件组中的相应事件位，或同步
		EventValue=xEventGroupWaitBits((EventGroupHandle_t	)g_event_group,		//确保按键按下同时进行
									   (EventBits_t			)0x0F,
									   (BaseType_t			)pdTRUE,				
									   (BaseType_t			)pdFALSE,
									   (TickType_t			)portMAX_DELAY);
		//延时消抖
		vTaskDelay(50);		
		if(EventValue & EVENT_GROUP_KEY1_DOWN)
		{
			//禁止EXTI0触发中断
			NVIC_DisableIRQ(EXTI0_IRQn);					//确保中断执行不被打断
			
			//确认是按下
			if(PAin(0) == 0)
			{
				dgb_printf_safe("S1 Press\r\n");
				#if	BEER
				beep_on();  PFout(9)=0;delay_ms(100);		//按下有效时提醒
				beep_off(); PFout(9)=1;
				#endif
				//xSemaphoreTake(g_sem_binary,portMAX_DELAY);
				g_flag_in = 0;								//修改门状态和登录方式
				g_door=0;
				//xSemaphoreGive(g_sem_binary);
				//等待按键释放
				while(PAin(0)==0)
					vTaskDelay(20);	
				
			}
				
			//允许EXTI0触发中断
			NVIC_EnableIRQ(EXTI0_IRQn);						//解除中断保护
		}
		
		if(EventValue & EVENT_GROUP_KEY2_DOWN)
		{
			//禁止EXTI2触发中断
			NVIC_DisableIRQ(EXTI2_IRQn);

				
			if(PEin(2) == 0)
			{
				dgb_printf_safe("S2 Press\r\n");
				#if	BEER
				beep_on();  PFout(9)=0;delay_ms(100);		//按下有效时提醒
				beep_off(); PFout(9)=1;
				#endif
				//xSemaphoreTake(g_sem_binary,portMAX_DELAY);
				g_flag_in = 1;
				PAout(4)=1;
				//g_door=0;
				//xSemaphoreGive(g_sem_binary);
				//等待按键释放
				while(PEin(2)==0)
					vTaskDelay(20);
			}

			//允许EXTI2触发中断
			NVIC_EnableIRQ(EXTI2_IRQn);	
		}	
		
		if(EventValue & EVENT_GROUP_KEY3_DOWN)
		{
			//禁止EXTI3触发中断
			NVIC_DisableIRQ(EXTI3_IRQn);
			
				
			if(PEin(3) == 0)	
			{
				dgb_printf_safe("S3 Press\r\n");
				#if	BEER
				beep_on();  PFout(9)=0;delay_ms(100);		//按下有效时提醒
				beep_off(); PFout(9)=1;
				#endif
				//xSemaphoreTake(g_sem_binary,portMAX_DELAY);
				g_flag_in = 2;
				g_flag_password_re=1;
				//g_door=1;
				//xSemaphoreGive(g_sem_binary);	
				//等待按键释放
				while(PEin(3)==0)
					vTaskDelay(20);
			}
				
			//允许EXTI3触发中断
			NVIC_EnableIRQ(EXTI3_IRQn);	
		}
		
		if(EventValue & EVENT_GROUP_KEY4_DOWN)
		{
			//禁止EXTI4触发中断
			NVIC_DisableIRQ(EXTI4_IRQn);
				
			if(PEin(4) == 0)	
			{
				dgb_printf_safe("S4 Press\r\n");
				#if	BEER
				beep_on();  PFout(9)=0;delay_ms(100);		//按下有效时提醒
				beep_off(); PFout(9)=1;
				#endif				
				//xSemaphoreTake(g_sem_binary,portMAX_DELAY);
				g_flag_in = 3;
				g_flag_password_look=1;
				//xSemaphoreGive(g_sem_binary);
				//等待按键释放
				while(PEin(4)==0)
					vTaskDelay(20);	
			}
			//允许EXTI4触发中断
			NVIC_EnableIRQ(EXTI4_IRQn);	
		}
		#else
			vTaskDelay(1000);	
		#endif
	}
} 

static void app_task_mqtt(void* pvParameters)
{
	uint32_t 	delay_1s_cnt=0;									//时间变量
	uint8_t		buf[5]={20,05,56,8,20};							//接收数组
	
	dgb_printf_safe("app_task_mqtt create success\r\n");		//显示连接状态
	
	dgb_printf_safe("app_task_mqtt suspend\r\n");

	vTaskSuspend(NULL);
	
	dgb_printf_safe("app_task_mqtt resume\r\n");
	
	vTaskDelay(1000);
	
	for(;;)
	{
		//发送心跳包
		mqtt_send_heart();							//防止离线
		
		//上报设备状态
		mqtt_report_devices_status();				//向上汇报
		
		delay_ms(1000);								//一秒一次
		
		delay_1s_cnt++;							
		
			
		if(delay_1s_cnt >= 3 )						//三秒一次
		{	
			delay_1s_cnt=0;

			if(0 == dht11_read(buf))				//检验和正确
			{
			
				g_temp = (float)buf[2]+(float)buf[3]/10;		//读取数组内容，修改全局变量温度和湿度，因为没有其他位置赋值，不需要上锁
				g_humi = (float)buf[0]+(float)buf[1]/10;
			
				printf("Temperature=%.1f Humidity=%.1f\r\n",g_temp,g_humi);
			}

		}
		 
	}
}

static void app_task_monitor(void* pvParameters)
{
	uint32_t esp8266_rx_cnt=0;
	
	BaseType_t xReturn = pdFALSE;					//初始化消息队列
	
	dgb_printf_safe("app_task_monitor create success \r\n");
	
	for(;;)
	{	
		esp8266_rx_cnt = g_esp8266_rx_cnt;
		
		delay_ms(10);
		
		/* n毫秒后，发现g_esp8266_rx_cnt没有变化，则认为接收数据结束 */
		if(g_esp8266_init && esp8266_rx_cnt && (esp8266_rx_cnt == g_esp8266_rx_cnt))
		{
			/* 发送消息，如果队列满了，超时时间为1000个节拍，如果1000个节拍都发送失败，函数直接返回 */
			xReturn = xQueueSend(g_queue_esp8266,(void *)g_esp8266_rx_buf,1000);		
			
			if (xReturn != pdPASS)
				dgb_printf_safe("[app_task_monitor] xQueueSend g_queue_esp8266 error code is %d\r\n", xReturn);
			
			g_esp8266_rx_cnt=0;
			memset((void *)g_esp8266_rx_buf,0,sizeof(g_esp8266_rx_buf));
		
		}	
	}
}

static void app_task_esp8266(void* pvParameters)
{
	
	/*看个人阿里云平台，如果上传数据也被捕获，那么就需要修改判断逻辑，因为id，可能随机，也可能固定*/
	
	uint8_t buf[512];								//初始化接收空间
	BaseType_t xReturn = pdFALSE;					//消息队列初始化
	uint32_t i;										//循环变量初始化
	
	dgb_printf_safe("app_task_esp8266 create success\r\n");
	
	while(esp8266_mqtt_init())
	{
		dgb_printf_safe("esp8266_mqtt_init ...");
		
		delay_ms(1000);
	}
	
	#if BEER
	//蜂鸣器嘀两声，D1灯闪烁两次，示意连接成功
	beep_on();  PFout(9)=0;delay_ms(100);
	beep_off(); PFout(9)=1;delay_ms(100);	
	beep_on();  PFout(9)=0;delay_ms(100);
	beep_off(); PFout(9)=1;delay_ms(100);
	#endif
	
	//g_flag_connect_server=1;
	
	printf("esp8266 connect aliyun with mqtt success\r\n");	
	
	vTaskResume(g_app_task_mqtt_handle);
	
	g_esp8266_init=1;
	
	for(;;)
	{	
		xReturn = xQueueReceive(g_queue_esp8266,	/* 消息队列的句柄 */
								buf,				/* 得到的消息内容 */
								portMAX_DELAY); 	/* 等待时间一直等 */
		if (xReturn != pdPASS)
		{
			dgb_printf_safe("[app_task_esp8266] xQueueReceive error code is %d\r\n", xReturn);
			continue;
		}	

		for(i=0;i<sizeof(buf);i++)
		{
			//判断的关键字符是否为 1"
			//核心数据，即{"switch_led_1":1}中的“1”
			if((buf[i]==0x31) && (buf[i+1]==0x22))
			{
					//判断控制变量
					if( buf[i+3]=='1' )
						PFout(9)=0;//控制灯亮
					else
						PFout(9)=1;//控制灯灭
			}	

			//判断的关键字符是否为 2"
			//核心数据，即{"switch_led_2":1}中的“1”
			if((buf[i]==0x32) && (buf[i+1]==0x22))
			{
					//判断控制变量
					if( buf[i+3]=='1' )
						PFout(10)=0;//控制灯亮
					else
						PFout(10)=1;//控制灯灭
			}

			//判断的关键字符是否为 3"
			//核心数据，即{"switch_led_3":1}中的“1”
			if(buf[i]==0x33 && buf[i+1]==0x22)
			{
					//判断控制变量
					if( buf[i+3]=='1' )
						PEout(13)=0;//控制灯亮
					else
						PEout(13)=1;//控制灯灭
			}	

			//判断的关键字符是否为 4"
			//核心数据，即{"switch_led_4":1}中的“1”
			if(buf[i]==0x34 && buf[i+1]==0x22)
			{
					//判断控制变量
					if( buf[i+3]=='1' )
						PEout(14)=0;//控制灯亮
					else
						PEout(14)=1;//控制灯灭
			}	

			
			//判断的关键字符是否为 5"
			//核心数据，即{"switch_door_5":1}中的“1”
			if(buf[i]==0x35 && buf[i+1]==0x22)
			{
					//判断控制变量
					if( buf[i+3]=='1')
						//TIM_SetCompare2(TIM4,(uint32_t)(2.5*180/20));	
						g_door=1;
					else
						//TIM_SetCompare2(TIM4,(uint32_t)(0));		
						g_door=0;
			}	

		}

	}
}

static void app_task_oled(void* pvParameters)
{
	int tmp_flag=0;								//上一次的状态
	int i = 0;									//左右滚动
	
	OLED_Clear();					
	OLED_DrawBMP(0,0,56,8,BMP4);				//初始化界面
	OLED_DrawBMP(64,0,120,8,BMP5);
	for(;;)
	{
		if(tmp_flag!=g_flag_in)					//模式切换的时候才清屏
		{
			if(tmp_flag==0)
				OLED_WR_Byte(0x2E,OLED_CMD);    //关闭滚动
			vTaskDelay(10);
			tmp_flag=g_flag_in;					//更新状态
			OLED_Clear();						//清屏
			vTaskDelay(10);
			if(g_flag_in==0)
			{
				i=!i;						//切换左右滚动
				OLED_DrawBMP(0,0,56,8,BMP4);		
				OLED_DrawBMP(64,0,120,8,BMP5);
				vTaskDelay(10);
			}
		}

		if(g_flag_in==0)					//模式0
		{
			if(i==1)
			{
				//从左到右移动
				OLED_WR_Byte(0x2E,OLED_CMD);        //关闭滚动
				OLED_WR_Byte(0x26,OLED_CMD);        //水平向左或者右滚动 26/27
				OLED_WR_Byte(0x00,OLED_CMD);        //虚拟字节
				OLED_WR_Byte(0x00,OLED_CMD);        //起始页 0
				OLED_WR_Byte(0x07,OLED_CMD);        //滚动时间间隔
				OLED_WR_Byte(0x07,OLED_CMD);        //终止页 7
				OLED_WR_Byte(0x00,OLED_CMD);        //虚拟字节
				OLED_WR_Byte(0xFF,OLED_CMD);        //虚拟字节
				OLED_WR_Byte(0x2F,OLED_CMD);        //开启滚动	
			}
			else
			{
						//从右到左移动
				OLED_WR_Byte(0x2E,OLED_CMD);        //关闭滚动
				OLED_WR_Byte(0x27,OLED_CMD);        //水平向左或者右滚动 26/27
				OLED_WR_Byte(0x00,OLED_CMD);        //虚拟字节
				OLED_WR_Byte(0x00,OLED_CMD);        //起始页 0
				OLED_WR_Byte(0x07,OLED_CMD);        //滚动时间间隔
				OLED_WR_Byte(0x07,OLED_CMD);        //终止页 7
				OLED_WR_Byte(0x00,OLED_CMD);        //虚拟字节
				OLED_WR_Byte(0xFF,OLED_CMD);        //虚拟字节
				OLED_WR_Byte(0x2F,OLED_CMD);        //开启滚动
			
			
			}
			vTaskDelay(100);
		}
		

		else if(g_flag_in==1)				//模式1
		{
			OLED_DrawBMP(32,0,95,8,BMP6);
			
		}

		else if(g_flag_in==2)				//模式2
		{
			OLED_DrawBMP(32,0,96,8,BMP7);
			
		}

		else if(g_flag_in==3)				//模式3
		{
			OLED_DrawBMP(0,0,128,8,BMP1);
			vTaskDelay(100);
			OLED_DrawBMP(0,0,128,8,BMP2);
							
		}
		vTaskDelay(100);
		OLED_WR_Byte(0x2E,OLED_CMD);        //关闭滚动	也可以在上面的滚动加一个flag,只执行一次，但是多了一个变量没必要,不加的话，就会出现图像异常平移.
	}
}


static void app_task_steer(void* pvParameters)
{
	//sg_angle(0);
	int i =0;                  					 //喂狗时间
	for(;;)										//通过标志位来控制舵机
	{
		
#if STREE_OPEN
	
		if(g_door==1)
		{
			sg_angle(0);
			
	#if STREE_LED_OPEN 
			PFout(9)=0;
			PFout(10)=0;
			PEout(13)=0;
			PEout(14)=0;
			//printf("%d\n",g_door);
	#endif
		}
		else if(g_door==0)
		{
			sg_angle(180);
	#if STREE_LED_OPEN 		
			PFout(9)=1;
			PFout(10)=1;
			PEout(13)=1;
			PEout(14)=1;
	#endif
			//printf("%d\n",g_door);			
		}
#endif
		
	delay_ms(100);

		if(i==2)
		{
			
			NVIC_DisableIRQ(EXTI0_IRQn);		//禁止EXTI0触发中断
			
			IWDG_ReloadCounter();				//喂狗不能中断，而且要快速
			
			NVIC_EnableIRQ(EXTI0_IRQn);			//允许EXTI0触发中断
			
			i=0;								//重新计数
		}
		i++;
	}
}


static void app_task_key16(void* pvParameters)
{
	char data_buf[18]={0};						//临时保存得到按下键值存入数据数组
	int i=0;	
	printf("Please enter a password with 18 characters or less\r\n");	//数组下标
	for(;;)
	{

		tmp = get_key_board();					//获取返回值char数据
		
		if(g_flag_password_re==0)	
		{
			
			if(tmp != 'N')						//不为N才有效
			{
				#if BEER
				beep_on();  PFout(9)=0;delay_ms(100);
				beep_off(); PFout(9)=1;
				#endif
				printf("%c\n",tmp);
				data_buf[i]=tmp;				//插入数组
				i++;
				if(tmp=='#')
				{
					i=0;
					printf("%s\n\r",data_buf);
					if(strcmp(data_buf,password)==0)
					{
						g_door=1;								//设置门状态
						printf("CXNB\r\n");
					}
					if(strcmp(data_buf,(const char*)flash_buf)==0)
					{
						g_door=1;
						printf("CXNB\r\n");
					}					
					memset(data_buf,0,sizeof(data_buf));		//清空,进行下一次输入操作
				}
			}
		}
		vTaskDelay(200);	
	}	
}

static void app_task_finger(void* pvParameters)
{
	int32_t rt;							//函数返回值接收
	uint32_t key_sta;					//键值控制功能选择
	uint16_t id=1;						//用户数
	uint32_t timeout=0;					//超时处理变量
	uint16_t user_total;				//获取用户数
	uint8_t switch_flag=0;				//开关标志位
	
	for(;;)
	{
		if(g_flag_in==1)
		{

			delay_ms(1000);
			
			dgb_printf_safe("指纹登录\r\n");
			
			/* 跟电容指纹模块进行握手 */
			while(SFM_ACK_SUCCESS!=sfm_init(115200))
			{
				
				delay_ms(500);
			
				dgb_printf_safe("电容指纹模块连接中 ...\r\n");		
			}
			
			dgb_printf_safe("电容指纹模块已连接上\r\n");
			
			/* 成功握手，蜂鸣器嘀一声示意 */
			#if BEER
			beep_on();delay_ms(50);beep_off();
			#endif
			/* 获取用户总数 */
			sfm_get_user_total(&user_total);
			
			dgb_printf_safe("电容指纹模块用户总数 = %d \r\n",user_total);
			
			for(;;)
			{
				key_sta=key_sta_get(tmp);
				
				/* 添加指纹 */
				if(key_sta & 0x01)
				{
					dgb_printf_safe("\r\n\r\n=====================================\r\n\r\n");
					dgb_printf_safe("执行添加指纹操作,请将手指放到指纹模块触摸感应区\r\n");
					timeout=0;
					
					/* 显示蓝色 */
					sfm_ctrl_led(0x06,0x06,0x32);
					
					while((sfm_touch_check()!=SFM_ACK_SUCCESS) && (timeout<10))
					{
						timeout++;
					}
					
					if(timeout>=10)
					{
						dgb_printf_safe("没有检测到手指，请重新操作!\r\n");
						
						/* 恢复光圈全亮->全灭，周期2秒 */
						sfm_ctrl_led(0x00,0x07,0xC8);
						
						continue;
					}
					
					dgb_printf_safe("检测到手指，现在开始添加指纹...\r\n");
					
					/* 获取未使用的用户id */
					rt = sfm_get_unused_id(&id);
					
					if(rt != SFM_ACK_SUCCESS)
					{
						dgb_printf_safe("获取未使用的用户id %s\r\n",sfm_error_code(rt));
					
						/* 恢复光圈全亮->全灭，周期2秒 */
						sfm_ctrl_led(0x00,0x07,0xC8);				
					
						continue;
					
					}
					
					dgb_printf_safe("将使用的用户id为%d\r\n",id);	
					
					rt=sfm_reg_user(id);
					
					if(rt == SFM_ACK_SUCCESS)
					{
						/* 成功:光圈显示绿色 */
						sfm_ctrl_led(0x05,0x05,0x32);			
						delay_ms(1000);
						
						/* 成功，蜂鸣器嘀一声示意 */
						#if BEER
						beep_on();delay_ms(50);beep_off();					
						#endif
					}
					else
					{
						/* 失败:光圈显示红色 */
						sfm_ctrl_led(0x03,0x03,0x32);
						
						delay_ms(1000);			
					}
					
					
					dgb_printf_safe("添加指纹 %s\r\n",sfm_error_code(rt));
					
					/* 恢复光圈全亮->全灭，周期2秒 */
					sfm_ctrl_led(0x00,0x07,0xC8);		
				}
				
				/* 刷指纹 */
				if(key_sta & 0x02)
				{
					dgb_printf_safe("\r\n\r\n=====================================\r\n\r\n");
					dgb_printf_safe("执行刷指纹操作,请将手指放到指纹模块触摸感应区\r\n");
					timeout=0;
					
					/* 显示蓝色 */
					sfm_ctrl_led(0x06,0x06,0x32);
					
					while((sfm_touch_check()!=SFM_ACK_SUCCESS) && (timeout<10))
					{
						timeout++;
					}
					
					if(timeout>=10)
					{
						dgb_printf_safe("没有检测到手指，请重新操作!\r\n");
						
						/* 恢复光圈全亮->全灭，周期2秒 */
						sfm_ctrl_led(0x00,0x07,0xC8);
						
						continue;
					}
					
					dgb_printf_safe("检测到手指，现在开始刷指纹...\r\n");
					
					rt=sfm_compare_users(&id);
					
					if(rt == SFM_ACK_SUCCESS)
					{
						/* 成功:光圈显示绿色 */
						sfm_ctrl_led(0x05,0x05,0x32);
						
						delay_ms(1000);
						
						/* 成功，蜂鸣器嘀一声示意 */
						#if BEER
						beep_on();delay_ms(50);beep_off();	
						#endif
						switch_flag=1;
						break;
					}
					else
					{
						/* 失败:光圈显示红色 */
						sfm_ctrl_led(0x03,0x03,0x32);
						
						delay_ms(1000);			
					}
					
					/* 若id为0，则比对不成功的*/
					dgb_printf_safe("刷指纹 %s 识别到id=%d\r\n",sfm_error_code(rt),id);
					
					/* 恢复光圈全亮->全灭，周期2秒 */
					sfm_ctrl_led(0x00,0x07,0xC8);
					
				}
				
				/* 获取用户总数 */
				if(key_sta & 0x04)
				{
					printf("\r\n\r\n=====================================\r\n\r\n");
			
					rt=sfm_get_user_total(&user_total);
			
					dgb_printf_safe("电容指纹模块用户总数 %s %d \r\n",sfm_error_code(rt),user_total);
				
					if(rt == SFM_ACK_SUCCESS)
					{
						/* 成功，蜂鸣器嘀一声示意 */
						#if BEER
						beep_on();delay_ms(50);beep_off();				
						#endif
					}		
				}	

				
				/* 删除所有指纹 */
				if(key_sta & 0x08)
				{
					printf("\r\n\r\n=====================================\r\n\r\n");

					rt = sfm_del_user_all();

					dgb_printf_safe("删除所有用户 %s\r\n",sfm_error_code(rt));
					
					if(rt == SFM_ACK_SUCCESS)
					{
						/* 成功，蜂鸣器嘀一声示意 */
						#if BEER
						beep_on();delay_ms(50);beep_off();
						#endif
					}			
				}
				vTaskDelay(100);				
				
			}
			if(switch_flag==1)
			{
				g_flag_in = 0;							//退出指纹模式
				
				/* 恢复光圈全亮->全灭，周期2秒 */
				sfm_ctrl_led(0x00,0x07,0xC8);		
				
				PAout(4)=0;								//继电器掉电
				g_door=1;								//开门
			}
			
		}
	
		vTaskDelay(100);
	}

}

static void app_task_flash(void* pvParameters)
{
	uint8_t  data=0;												//接收数据
	uint16_t W25ID=0;												//设备ID
	int i=0;														//初始化数组下标
	uint8_t data_buf[18]={0};										//初始化接收数据空间
	
	/*可自定义初始密码*/	
#if PASSWORD_INIT 
	
	//先擦除再写
	Flash_SectorErase(PASSWORD_ADDR);
	delay_ms(500);
	//写入内容
	Flash_PageProgram(PASSWORD_ADDR, 10,(uint8_t *)PASSWORD_PRIMARY);
	
#endif	
	
	/*可自定义初始密码*/	
	
	W25ID = Flash_ReadID();											//获取设备ID
	printf("W25ID=%x\r\n",W25ID);
	delay_ms(500);	
	Flash_ReadData(PASSWORD_ADDR, 10,(uint8_t *)flash_buf);				//获取密码
	
#if PASSWORD_PASSWORD_LOOK 	
	
	printf("rdatabuf=%s\r\n",flash_buf);							//实用时候修改宏定义即可
	
#endif	
	
	delay_ms(500);
	memcpy(password,flash_buf,sizeof(flash_buf));					//清空接收密码数组,准备查看新密码
	for(;;)
	{
		if(g_flag_password_re==1)
		{
			printf("The password change mode is displayed\r\n");	//进入修改模式
			
			for(;;)
			{
				data=tmp; 											//获取返回值char键盘数据
			
				if(tmp != 'N')
				{
					#if BEER
					beep_on();delay_ms(100);beep_off();
					#endif
					printf("change : %c\n",data);	
					data_buf[i]=data;								//接收键盘的临时数据,一次循环加延时后,数据变成'N',所以没影响.同时屏蔽本来的密码输入即可.
					i++;
					if(data=='#')
					{
						printf("%s\n",data_buf);					//接收完毕，打印查看
						i=0;									
						//先擦除再写
						Flash_SectorErase(PASSWORD_ADDR);				
						delay_ms(500);
						//写入内容
						Flash_PageProgram(PASSWORD_ADDR,sizeof(data_buf),data_buf);
						memset(data_buf,0,sizeof(data_buf));		//清空等待下一次修改
						g_flag_password_look=1;						//开启密码查看
						g_flag_password_re=0;						//关闭修改密码模式
						printf("Please enter a password with 18 characters or less\r\n");
						break;
					}
					data='N';
					
				}
				delay_ms(200);
			}
			printf("Password changed successfully\r\n");			//打印修改成功
		}
		if(g_flag_password_look==1)
		{
			printf("Enter the password viewing mode\r\n");			//进入查看模式
			W25ID = Flash_ReadID();
			printf("W25ID=%x\r\n",W25ID);
			delay_ms(500);
			Flash_ReadData(PASSWORD_ADDR,sizeof(data_buf),(uint8_t *)flash_buf);
			printf("rdatabuf=%s\r\n",flash_buf);
			g_flag_password_look=0;
			memcpy(password,flash_buf,sizeof(flash_buf));			//清空接收密码数组,准备查看新密码
		}
		delay_ms(1000);
		
	}

}

/*-----------------------------------------------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}


void vApplicationTickHook( void )
{

}
