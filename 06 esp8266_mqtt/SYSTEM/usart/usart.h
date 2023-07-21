#ifndef __USART_H
#define __USART_H

extern volatile uint8_t  g_usart1_rx_buf[512];
extern volatile uint32_t g_usart1_rx_cnt;
extern volatile uint32_t g_usart1_rx_end;

extern volatile uint8_t  g_usart2_rx_buf[512];
extern volatile uint32_t g_usart2_rx_cnt;
extern volatile uint32_t g_usart2_rx_end;

extern volatile uint8_t  g_usart3_rx_buf[512];
extern volatile uint32_t g_usart3_rx_cnt;
extern volatile uint32_t g_usart3_rx_end;

extern void usart1_init(uint32_t baud);
extern void usart2_init(uint32_t baud);
extern void usart3_init(uint32_t baud);

extern void usart3_send_str(char *str);
extern void usart3_send_bytes(uint8_t *buf,uint32_t len);



#endif


