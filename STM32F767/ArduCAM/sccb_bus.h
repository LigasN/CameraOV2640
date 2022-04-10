
#ifndef _SCCB_BUS_H_
#define _SCCB_BUS_H_

#include "main.h"

#define I2C_TIM 1

#define SCL_Pin GPIO_Pin_8 // Was GPIO_Pin_10 for other board
#define SDA_Pin GPIO_Pin_9 // Was GPIO_Pin_11 for other board

#define SCCB_SIC_H() GPIO_SetBits(GPIOB, SCL_Pin)   // SCL H
#define SCCB_SIC_L() GPIO_ResetBits(GPIOB, SCL_Pin) // SCL H
#define SCCB_SID_H() GPIO_SetBits(GPIOB, SDA_Pin)   // SDA	H
#define SCCB_SID_L() GPIO_ResetBits(GPIOB, SDA_Pin) // SDA	H

#define SCCB_DATA_IN                                                           \
    {                                                                          \
        GPIOB->MODER &= 0XFF3FFFFF;                                            \
        GPIOB->MODER |= 0X00000000;                                            \
    }
#define SCCB_DATA_OUT                                                          \
    {                                                                          \
        GPIOB->MODER &= 0XFF3FFFFF;                                            \
        GPIOB->MODER |= 0X00400000;                                            \
    }
#define SCCB_SID_STATE GPIO_ReadInputDataBit(GPIOB, SDA_Pin)

void sccb_bus_init(void);
void sccb_bus_start(void);
void sccb_bus_stop(void);
void sccb_bus_send_noack(void);
void sccb_bus_send_ack(void);
uint8_t sccb_bus_write_byte(uint8_t data);
uint8_t sccb_bus_read_byte(void);

#endif /* _SCCB_BUS_H_ */

/******************* (C) COPYRIGHT 2015 WangBao *****END OF FILE****/
