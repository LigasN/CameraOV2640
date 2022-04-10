/*
 * uart_printf.c
 *
 *  Created on: Apr 10, 2022
 *      Author: norbe
 */

#include "uart_printf.h"
#include "usart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

void uprintf(const char* fmt, ...)
{
    va_list argp;
    va_start(argp, fmt);
    char string[200];
    if (0 < vsprintf(string, fmt, argp)) // build string
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)string, strlen(string),
                          0xffffff); // send message via UART
    }
    va_end(argp);
}
