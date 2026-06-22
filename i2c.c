#include <stdint.h>
#include "i2c.h"
#include "stm32f411.h"

static uint8_t I2C_Validate_Pins(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->channel == I2C_CHANNEL_1)
    {
        if (hi2c->scl_port == GPIOB && hi2c->sda_port == GPIOB)
        {
            if ((hi2c->scl_pin == 6 || hi2c->scl_pin == 8) && (hi2c->sda_pin == 7 || hi2c->sda_pin == 9))
            {
                return 1;
            }
        }
    }
    else if (hi2c->channel == I2C_CHANNEL_2)
    {
        if (hi2c->scl_port == GPIOB && hi2c->sda_port == GPIOB)
        {
            if ((hi2c->scl_pin == 10) && (hi2c->sda_pin == 3 || hi2c->sda_pin == 9))
            {
                return 1;
            }
        }
    }
    else if (hi2c->channel == I2C_CHANNEL_3)
    {
        if (hi2c->scl_port == GPIOA && hi2c->sda_port == GPIOB)
        {
            if ((hi2c->scl_pin == 8) && (hi2c->sda_pin == 4 || hi2c->sda_pin == 8))
            {
                return 1;
            }
        }
    }

    return 0;
}

void I2C_init(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->channel >= I2C_CHANNEL_MAX)
    {
        return;
    }

    if (!I2C_Validate_Pins(hi2c))
    {
        return;
    }

    switch (hi2c->channel)
    {
    case I2C_CHANNEL_1:
        // enable GPIOB clock
        RCC->AHB1ENR |= (1 << 1);

        // enable I2C for channel 1
        RCC->APB1ENR |= (1 << 21);

        break;
    case I2C_CHANNEL_2:
        // enable GPIOB clock
        RCC->AHB1ENR |= (1 << 1);

        // enable I2C for channel 2
        RCC->APB1ENR |= (1 << 22);

        break;
    case I2C_CHANNEL_3:
        // enable GPIOA and GPIOB clocks
        RCC->AHB1ENR |= (1 << 1) | (1 << 0);

        // enable I2C for channel 3
        RCC->APB1ENR |= (1 << 23);

        break;
    default:
        return;
    }

    /* ----- set the MODER for SCL pin and SDA pin to Alternate function ----- */
    // value: 10

    // one pin takes 2 bits, so (2 * pin_number):
    // e.g. pin 6 takes 2*6 = 12 => 13:12
    // pin 7 takes 2*7 = 14 => 15:14
    // pin 0 takes 2*0 = 0 => 1:0

    // clear the MODER bits
    // 11 = 0x3
    hi2c->scl_port->MODER &= ~(0x3 << (2 * hi2c->scl_pin));
    hi2c->sda_port->MODER &= ~(0x3 << (2 * hi2c->sda_pin));

    // set the MODER bits
    // 10 = 0x2
    hi2c->scl_port->MODER |= (0x2 << (2 * hi2c->scl_pin));
    hi2c->sda_port->MODER |= (0x2 << (2 * hi2c->sda_pin));

    /* ----- set the AFRL for SCL pin and SDA pin to AF4 ----- */
    // AF4 value: 0100

    // AFRL/AFRH uses 4 bits per 1 pin

    if (hi2c->scl_pin >= 8)
    {
        // pins 8-15 are set in AFRH

        // clear the AFRH bits
        // 1111 = 2^3 + 2^2 + 2^1 + 2^0 = 8 + 4 + 2 + 1 = 15 = 0xF
        hi2c->scl_port->AFRH &= ~(0xF << (4 * (hi2c->scl_pin - 8))); // scl_pin - 8 to prevent overflow of 32 bit register

        // set the bits to AF4
        hi2c->scl_port->AFRH |= (GPIO_AF4 << (4 * (hi2c->scl_pin - 8)));
    }
    else
    {
        // pins 0-7 are set in AFRL

        // clear the AFRL bits
        // 1111 = 2^3 + 2^2 + 2^1 + 2^0 = 8 + 4 + 2 + 1 = 15 = 0xF
        hi2c->scl_port->AFRL &= ~(0xF << (4 * hi2c->scl_pin));

        // set the bits to AF4
        hi2c->scl_port->AFRL |= (GPIO_AF4 << (4 * hi2c->scl_pin));
    }

    if (hi2c->sda_pin >= 8)
    {
        // pins 8-15 are set in AFRH

        // clear the AFRH bits
        // 1111 = 2^3 + 2^2 + 2^1 + 2^0 = 8 + 4 + 2 + 1 = 15 = 0xF
        hi2c->sda_port->AFRH &= ~(0xF << (4 * (hi2c->sda_pin - 8))); // sda_pin - 8 to prevent overflow of 32 bit register

        // set the bits to AF4
        hi2c->sda_port->AFRH |= (GPIO_AF4 << (4 * (hi2c->sda_pin - 8)));
    }
    else
    {
        // pins 0-7 are set in AFRL

        // clear the AFRL bits
        // 1111 = 2^3 + 2^2 + 2^1 + 2^0 = 8 + 4 + 2 + 1 = 15 = 0xF
        hi2c->sda_port->AFRL &= ~(0xF << (4 * hi2c->sda_pin));

        // set the bits to AF4
        hi2c->sda_port->AFRL |= (GPIO_AF4 << (4 * hi2c->sda_pin));
    }

    /* ----- set the OTYPER for SCL pin and SDA pin to Open-drain ----- */
    // value 1

    hi2c->scl_port->OTYPER |= (1 << hi2c->scl_pin);
    hi2c->sda_port->OTYPER |= (1 << hi2c->sda_pin);

    I2C_RegDef_t *I2C;
    /* ----- I2C Configuration ----- */
    if (hi2c->channel == 1)
    {
        I2C = I2C1;
    }
    else if (hi2c->channel == 2)
    {
        I2C = I2C2;
    }
    else
    {
        I2C = I2C3;
    }

    // set FREQ[5:0] (Peripheral clock frequency) in CR2
    // clear the bits
    // 111111 = 2^5 + 2^4 + 2^3 + 2^2 + 2^1 + 2^0 = 32 + 16 + 8 + 4 + 2 + 1 = 48 + 15 = 63 = 0x3F
    I2C->CR2 &= ~(0x3F << 0);

    // set the bits
    // 16 MHz = 010000 = 0x10
    I2C->CR2 |= (0x10 << 0);
}