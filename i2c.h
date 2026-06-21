#ifndef I2C_H
#define I2C_H

typedef enum
{
    I2C_CHANNEL_1 = 1, // SCL1 - PA6, SDA1 = PB7
    I2C_CHANNEL_2,     // SCL2 - PB10, SDA2 - PB3
    I2C_CHANNEL_3,     // SCL3 - PB8, SDA3 - PB3
    I2C_CHANNEL_MAX
} I2C_Channel_t;

typedef struct
{
    I2C_Channel_t channel;
} I2C_HandleTypeDef;

void I2C_init(void);

#endif