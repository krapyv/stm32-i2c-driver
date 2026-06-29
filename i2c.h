#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include "stm32f411.h"

typedef enum
{
    I2C_CHANNEL_1 = 1, // SCL1 - PB6, SDA1 = PB7
    I2C_CHANNEL_2,     // SCL2 - PB10, SDA2 - PB3
    I2C_CHANNEL_3,     // SCL3 - PA8, SDA3 - PB4
    I2C_CHANNEL_MAX
} I2C_Channel_t;

typedef enum
{
    I2C_SB_MASK = (1 << 0),
    I2C_ADDR_MASK = (1 << 1),
    I2C_TXE_MASK = (1 << 7),
    I2C_RXNE_MASK = (1 << 6),
    I2C_BTF_MASK = (1 << 2),

} I2C_Bit_Masks_t;

typedef enum
{
    I2C_OK = 0,
    I2C_ERROR = 1
} I2C_Status_t;

typedef struct
{
    I2C_Channel_t channel;
    GPIO_RegDef_t *scl_port;
    GPIO_RegDef_t *sda_port;
    uint8_t scl_pin;
    uint8_t sda_pin;

    I2C_RegDef_t *Instance;
} I2C_HandleTypeDef;

void I2C_Init(I2C_HandleTypeDef *hi2c);
I2C_Status_t I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint8_t slave_addr, uint8_t *data, uint8_t length);
I2C_Status_t I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint8_t slave_addr, uint8_t *data, uint8_t length);
I2C_Status_t I2C_Master_Transmit_Receive(I2C_HandleTypeDef *hi2c, uint8_t slave_addr, uint8_t *pSend, uint8_t *pReceive, uint8_t send_length, uint8_t receive_length);

#endif