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

typedef enum
{
    I2C_STATE_IDLE = 0,
    I2C_STATE_ERROR,
    I2C_STATE_START_PENDING,
    I2C_STATE_TX_ADDR,
    I2C_STATE_RX_ADDR,
    I2C_STATE_FINISHING,
    I2C_STATE_DONE
} I2C_State_t;

typedef enum
{
    I2C_TX = 0,
    I2C_RX,
    I2C_TX_RX
} I2C_Mode_t;

typedef enum
{
    I2C_TX_RX_WRITE = 0,
    I2C_TX_RX_READ
} I2C_TX_RX_Phase_t;

typedef enum
{
    I2C_ERROR_NONE = 0,
    I2C_ERROR_BERR = (1 << 8),
    I2C_ERROR_ARLO = (1 << 9),
    I2C_ERROR_AF = (1 << 10),
} I2C_Error_Code_t;

typedef struct
{
    I2C_Channel_t channel;
    GPIO_RegDef_t *scl_port;
    GPIO_RegDef_t *sda_port;
    uint8_t scl_pin;
    uint8_t sda_pin;
    uint8_t slave_add;

    volatile I2C_State_t state;
    volatile uint8_t index;
    volatile I2C_Mode_t mode;
    volatile uint16_t error_code;
    volatile I2C_TX_RX_Phase_t phase;

    volatile uint8_t *pTxBuffPtr;
    volatile uint8_t TxLength;

    volatile uint8_t *pRxBuffPtr;
    volatile uint8_t RxLength;

    uint32_t sb_hits;
    uint32_t stop_hits;
    uint32_t start_pending_hits;

    I2C_RegDef_t *Instance;
} I2C_HandleTypeDef;

// get "a link" of the variable that has been declared globally in another file in the project scope
extern I2C_HandleTypeDef hi2c;

void I2C_Reinit(void);
void I2C_Init(void);
I2C_Status_t I2C_Master_Transmit(uint8_t slave_addr, uint8_t *data, uint8_t length);
I2C_Status_t I2C_Master_Receive(uint8_t slave_addr, uint8_t *data, uint8_t length);
I2C_Status_t I2C_Master_Transmit_Receive(uint8_t slave_addr, uint8_t *pSend, uint8_t *pReceive, uint8_t send_length, uint8_t receive_length);

void I2C_Process(void);
#endif