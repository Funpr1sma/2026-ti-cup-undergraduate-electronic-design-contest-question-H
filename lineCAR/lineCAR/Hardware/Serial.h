#ifndef HARDWARE_SERIAL_H_
#define HARDWARE_SERIAL_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * 初始化UART0串口软件模块。
 *
 * UART0硬件由SYSCFG_DL_init()初始化：
 * PA10 = TX，PA11 = RX，115200，8N1。
 *
 * Serial_Init()会为PA11/RX重新启用内部上拉，因此不连接USB转串口时
 * 串口输入仍保持稳定高电平。发送采用有限等待，串口异常不能无限阻塞
 * 按键扫描和小车控制。
 *
 * 本模块使用RX环形缓冲区，能够连续接收
 * VOFA+发送的整行ASCII命令。
 */
void Serial_Init(void);

/* 有限等待发送一个字节；超时则丢弃，绝不无限阻塞控制程序。 */
void Serial_SendByte(uint8_t data);

/* 发送以'\0'结尾的字符串。 */
void Serial_SendString(const char *text);

/*
 * 尝试从UART0 RX环形缓冲区读取一个字节。
 *
 * 返回true：成功读取，结果写入*data；
 * 返回false：当前没有可读字节。
 */
bool Serial_TryReadByte(uint8_t *data);

/* 返回RX环形缓冲区溢出次数。 */
uint32_t Serial_GetRxOverflowCount(void);

/* 返回UART0 RX中断进入次数。 */
uint32_t Serial_GetRxIrqCount(void);

/* 返回因UART TX长时间不可用而丢弃的字节数。 */
uint32_t Serial_GetTxDropCount(void);

#endif /* HARDWARE_SERIAL_H_ */
