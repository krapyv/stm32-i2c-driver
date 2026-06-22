#include <stdint.h>
#include <stdio.h>
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
    if ((hi2c->channel >= I2C_CHANNEL_MAX) || !I2C_Validate_Pins(hi2c))
    {
        return;
    }

    /* ----- I2C Choosing ----- */
    if (hi2c->channel == 1)
    {
        hi2c->Instance = I2C1;
    }
    else if (hi2c->channel == 2)
    {
        hi2c->Instance = I2C2;
    }
    else
    {
        hi2c->Instance = I2C3;
    }
    /* ----- I2C Choosing ----- */

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

    /* ----- I2C CCR configuration ----- */

    // declare a local I2C var to simplify the naming
    I2C_RegDef_t *I2C = hi2c->Instance;

    // set the bit 15 to 0 (Sm mode I2C)
    I2C->CCR &= ~(1 << 15);

    // set the CCR [11:0] to 80 = 0x50, since CCR = T_high / T_pclk1
    // where T_high = 5000 ns, T_pclk1 = 62.5 ns (1 / 16 000 000)

    // clear the range 11:0
    // 111111111111 = 0xFF
    I2C->CCR &= ~(0xFFF << 0);

    // set the range to 0x50
    I2C->CCR |= (0x50 << 0);

    // set FREQ[5:0] (Peripheral clock frequency) in CR2
    // clear the bits
    // 111111 = 2^5 + 2^4 + 2^3 + 2^2 + 2^1 + 2^0 = 32 + 16 + 8 + 4 + 2 + 1 = 48 + 15 = 63 = 0x3F
    I2C->CR2 &= ~(0x3F << 0);

    // set the bits
    // 16 MHz = 010000 = 0x10
    I2C->CR2 |= (0x10 << 0);

    /* ----- TRISE configuration ----- */
    // set the bits 5:0 to 17 = 0x11
    // 1000 ns / 62.5 + 1 = 16 + 1 = 17

    // clear the range
    // 111111 = 2^5 + 2^4 + 2^3 + 2^2 + 2^1 + 2^0 = 32 + 16 + 8 + 4 + 2 + 1 = 40 + 20 + 3 = 63 = 0x3F
    I2C->TRISE &= ~(0x3F << 0);

    // set the range to 0x11
    I2C->TRISE |= (0x11 << 0);

    // ----- CR1 configuration ----- //

    // set bit 0 PE (Peripheral enabled) to 1

    I2C->CR1 |= (1 << 0);

    // set bit 10 ACK (Acknowledge enable) to 1
    // since this bit is set and cleared by software and cleared by hardware when PE = 0
    // it is enabled after PE became 1
    I2C->CR1 |= (1 << 10);
}

void I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint8_t slave_addr, uint8_t *data, uint8_t length)
{
    if (hi2c == NULL || hi2c->Instance == NULL || data == NULL || length == 0)
    {
        return;
    }

    // declare a local I2C var to simplify the naming
    I2C_RegDef_t *I2C = hi2c->Instance;

    // start the transaction by issuing START (set to 1)
    I2C->CR1 |= (1 << 8);
    // START is cleared by hardware when start is sent

    while (!(I2C->SR1 & (1 << 0)))
        ; // polling the SB (Start bit)
    // when exit, the bit is set, start condition generated

    // write the slave address to the DR register (it will start sending the bits to the slave and clear the SB bit)
    // 7 bit slave address + W (Write) bit 0
    I2C->DR = (slave_addr << 1) & ~0x01; // 0x01 = 2^0 = 1 = (1 << 0)

    // polling the ADDR (Address sent) bit
    // the bit is set after the ACK of the byte
    while (!(I2C->SR1 & (1 << 1)))
    {
        uint32_t sr1_snap = I2C->SR1;

        // check the bit 10 Acknowledge failure (the peripheral is disconnected, powered off or missing)
        // the bit 9 ARLO (Arbitration lost) (the another master seized the bus)
        // the bit 8 BERR (Bus error) (an external glitch happened)
        if (sr1_snap & ((1 << 10) | (1 << 9) | (1 << 8)))
        {
            // clear the error flags by writing 0 to them
            I2C->SR1 &= ~((1 << 10) | (1 << 9) | (1 << 8));

            // generate the STOP condition to release the physical lines safely
            I2C->CR1 |= (1 << 9);

            return;
        }
    }
}