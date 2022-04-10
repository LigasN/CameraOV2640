#include "sccb_bus.h"
uint32_t ntime;

/**
 * @brief  init i2c bus
 * @param  None
 * @retval None
 */
void sccb_bus_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStructure.Pin   = SDA_Pin;
    GPIO_InitStructure.Mode  = GPIO_MODE_AF_OD;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
    HAL_GPIO_WritePin(GPIOB, SDA_Pin, GPIO_PIN_SET);
    GPIO_InitStructure.Pin  = SCL_Pin;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
    HAL_GPIO_WritePin(GPIOB, SCL_Pin, GPIO_PIN_SET);
    SCCB_DATA_OUT;
}

void sccb_bus_start(void)
{
    SCCB_SID_H();
    delay_us(I2C_TIM);
    SCCB_SIC_H();
    delay_us(I2C_TIM);
    SCCB_SID_L();
    delay_us(I2C_TIM);
    SCCB_SIC_L();
    delay_us(I2C_TIM);
}

void sccb_bus_stop(void)
{
    SCCB_SID_L();
    delay_us(I2C_TIM);
    SCCB_SIC_H();
    delay_us(I2C_TIM);
    SCCB_SID_H();
    delay_us(I2C_TIM);
}

void sccb_bus_send_noack(void)
{
    SCCB_SID_H();
    delay_us(I2C_TIM);
    SCCB_SIC_H();
    delay_us(I2C_TIM);
    SCCB_SIC_L();
    delay_us(I2C_TIM);
    SCCB_SID_L();
    delay_us(I2C_TIM);
}

void sccb_bus_send_ack(void)
{
    SCCB_SID_L();
    delay_us(I2C_TIM);
    SCCB_SIC_L();
    delay_us(I2C_TIM);
    SCCB_SIC_H();
    delay_us(I2C_TIM);
    SCCB_SIC_L();
    delay_us(I2C_TIM);
    SCCB_SID_L();
    delay_us(I2C_TIM);
}

uint8_t sccb_bus_write_byte(uint8_t data)
{
    uint32_t i;
    uint8_t tem;

    for (i = 0; i < 8; i++)
    {
        if ((data << i) & 0x80)
        {
            SCCB_SID_H();
        }
        else
        {
            SCCB_SID_L();
        }
        delay_us(I2C_TIM);
        SCCB_SIC_H();
        delay_us(I2C_TIM);
        SCCB_SIC_L();
    }
    SCCB_DATA_IN;
    delay_us(I2C_TIM);
    SCCB_SIC_H();
    delay_us(I2C_TIM);
    if (SCCB_SID_STATE)
    {
        tem = 0;
    }
    else
    {
        tem = 1;
    }

    SCCB_SIC_L();
    delay_us(I2C_TIM);
    SCCB_DATA_OUT;
    return tem;
}

uint8_t sccb_bus_read_byte(void)
{
    uint32_t i;
    uint8_t read = 0;

    SCCB_DATA_IN;
    delay_us(I2C_TIM);
    for (i = 8; i > 0; i--)
    {
        delay_us(I2C_TIM);
        SCCB_SIC_H();
        delay_us(I2C_TIM);
        read = read << 1;
        if (SCCB_SID_STATE)
        {
            read += 1;
        }
        SCCB_SIC_L();
        delay_us(I2C_TIM);
    }
    SCCB_DATA_OUT;
    return read;
}
