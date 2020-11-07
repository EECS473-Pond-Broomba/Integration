/*
 * uart_printf.h
 *
 *  Created on: Oct 17, 2020
 *      Author: rishgoel
 */

#ifndef INC_UART_PRINTF_H_
#define INC_UART_PRINTF_H_

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
//#include "stm32f4xx_hal.h"
#include "main.h"

void vprint(const char *fmt, va_list argp)
{
	char temp[500];
	if(vsprintf(temp, fmt, argp))
	{
		HAL_UART_Transmit(&huart2, (uint8_t*) temp, strlen(temp), 1000);
	}
}

void uart_printf(const char *fmt, ...)
{
	va_list argp;
	va_start(argp, fmt);
	vprint(fmt, argp);
	va_end(argp);
}



#endif /* INC_UART_PRINTF_H_ */
