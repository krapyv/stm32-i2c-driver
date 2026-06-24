#ifndef I2C_H
#define I2C_H

typedef enum
{
    I2C_CHANNEL_1 = 1, // SCL1 - PB6, SDA1 = PB7
    I2C_CHANNEL_2,     // SCL2 - PB10, SDA2 - PB3
    I2C_CHANNEL_3,     // SCL3 - PA8, SDA3 - PB4
    I2C_CHANNEL_MAX
} I2C_Channel_t;

typedef struct
{
    I2C_Channel_t channel;
    GPIO_RegDef_t *scl_port;
    GPIO_RegDef_t *sda_port;
    uint8_t scl_pin;
    uint8_t sda_pin;

    I2C_RegDef_t *Instance;
} I2C_HandleTypeDef;

void I2C_init(I2C_HandleTypeDef *hi2c);
void I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint8_t slave_addr, uint8_t *data, uint8_t length);
void I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint8_t slave_addr, uint8_t *data, uint8_t length);
void I2C_Master_Transmit_Receive(I2C_HandleTypeDef *hi2c, uint8_t slave_addr, uint8_t *pSend, uint8_t *pReceive, uint8_t send_length, uint8_t receive_length);

#endif