#include <stdint.h>
#include <stdio.h>
#include "i2c.h"
#include "stm32f411.h"
#include "core_cm4.h"
#include "systick.h"

void I2C_Reinit(void)
{
    // explicitly disable PE (bit 0 CR1) to prevet undefined hardware behavior
    // if the PE is already 1 after reboot or SWRST recovery
    hi2c.Instance->CR1 &= ~(1 << 0);

    // set the bit 15 to 0 (Sm mode I2C)
    hi2c.Instance->CCR &= ~(1 << 15);

    // set the CCR [11:0] to 80 = 0x50, since CCR = T_high / T_pclk1
    // where T_high = 5000 ns, T_pclk1 = 62.5 ns (1 / 16 000 000)

    // clear the range 11:0
    // 111111111111 = 0xFF
    hi2c.Instance->CCR &= ~(0xFFF << 0);

    // set the range to 0x50
    hi2c.Instance->CCR |= (0x50 << 0);

    // set FREQ[5:0] (Peripheral clock frequency) in CR2
    // clear the bits
    // 111111 = 2^5 + 2^4 + 2^3 + 2^2 + 2^1 + 2^0 = 32 + 16 + 8 + 4 + 2 + 1 = 48 + 15 = 63 = 0x3F
    hi2c.Instance->CR2 &= ~(0x3F << 0);

    // set the bits
    // 16 MHz = 010000 = 0x10
    hi2c.Instance->CR2 |= (0x10 << 0);

    /* ----- TRISE configuration ----- */
    // set the bits 5:0 to 17 = 0x11
    // 1000 ns / 62.5 + 1 = 16 + 1 = 17

    // clear the range
    // 111111 = 2^5 + 2^4 + 2^3 + 2^2 + 2^1 + 2^0 = 32 + 16 + 8 + 4 + 2 + 1 = 40 + 20 + 3 = 63 = 0x3F
    hi2c.Instance->TRISE &= ~(0x3F << 0);

    // set the range to 0x11
    hi2c.Instance->TRISE |= (0x11 << 0);

    // ----- CR1 configuration ----- //

    // set bit 0 PE (Peripheral enabled) to 1

    hi2c.Instance->CR1 |= (1 << 0);

    // set bit 10 ACK (Acknowledge enable) to 1
    // since this bit is set and cleared by software and cleared by hardware when PE = 0
    // it is enabled after PE became 1
    hi2c.Instance->CR1 |= (1 << 10);

    /* ----- Interrupts enablement (CR2) -----*/

    // set ITERREN (Error interrupt enable) Bit 8 to 1
    hi2c.Instance->CR2 |= (1 << 8);

    // set ITEVTEN (Event interrupt enable) Bit 9 to 1
    hi2c.Instance->CR2 |= (1 << 9);
}

// Events Interrupt Handler (priority 38 - cannot be preempted by I2C1_ER)
void I2C1_EV_IRQHandler(void)
{
    // if the state is ERROR, return
    if ((hi2c.state == I2C_STATE_ERROR) || (hi2c.state == I2C_STATE_IDLE) || (hi2c.state == I2C_STATE_DONE) || (hi2c.state == I2C_STATE_FINISHING))
    {
        return;
    }

    uint16_t sr1_snapshot = hi2c.Instance->SR1;
    uint16_t cr1_snapshot = hi2c.Instance->CR1;
    uint16_t cr2_snapshot = hi2c.Instance->CR2;
    uint32_t dummy = 0;

    // SB bit 0 in SR1 caused interrupt
    if (sr1_snapshot & (1 << 0))
    {
        hi2c.sb_hits++;
        // clear the SB by reading SR1 (already done by creating snapshot)
        // following by writing the DR register with the slave's address (7 bits) + W (0)/R (1) bit
        switch (hi2c.mode)
        {
        case I2C_TX:
            // use W (0), so clear the bit 0th
            hi2c.Instance->DR = (hi2c.slave_add << 1) & ~(0x01);
            hi2c.state = I2C_STATE_TX_ADDR;
            break;
        case I2C_RX:
            // use R (1), so set the bit 0th to 1
            hi2c.Instance->DR = (hi2c.slave_add << 1) | 0x01;
            hi2c.state = I2C_STATE_RX_ADDR;
            break;
        case I2C_TX_RX:
            if (hi2c.phase == I2C_TX_RX_WRITE)
            {
                // use W (0), so clear the bit 0th
                hi2c.Instance->DR = (hi2c.slave_add << 1) & ~(0x01);
                hi2c.state = I2C_STATE_TX_ADDR;
            }
            else
            {
                // use R (1), so set the bit 0th to 1
                hi2c.Instance->DR = (hi2c.slave_add << 1) | 0x01;
                hi2c.state = I2C_STATE_RX_ADDR;
            }
            break;
        }
    }
    // ADDR bit 1 in SR1 caused interrupt
    else if (sr1_snapshot & (1 << 1))
    {
        switch (hi2c.state)
        {
        // transmit phase
        case I2C_STATE_TX_ADDR:
            // after the ADDR is set, clear it
            // read SR2, since SR1 has already been read
            dummy = hi2c.Instance->SR2;
            (void)dummy;

            // enable ITBUFEN
            hi2c.Instance->CR2 |= (1 << 10);
            break;

        case I2C_STATE_RX_ADDR:
            // N = 1 case
            if (hi2c.RxLength == 1)
            {
                // need to clear the ACK before ADDR clear

                // clear the ACK
                hi2c.Instance->CR1 &= ~(1 << 10);

                // clear the ADDR by reading SR2 (SR1 has already been read)
                dummy = hi2c.Instance->SR2;
                (void)dummy;

                // then set the STOP to 1, because since we have 1 byte to get, we need the STOP immediately after it
                hi2c.Instance->CR1 |= (1 << 9);

                // enable ITBUFEN
                hi2c.Instance->CR2 |= (1 << 10);
            }
            // N = 2 case
            else if (hi2c.RxLength == 2)
            {
                // need to set POS = 1 and ACK = 0 before ADDR clear

                // first of all, set POS (bit 11 in CR1) to 1
                // POS = 1 - ACK bit controls the (N)ACK of the next byte with is received in the shift register
                hi2c.Instance->CR1 |= (1 << 11);

                // clear the ACK (bit 10 in CR1)
                // to prepare the NACK pulse for the last second byte
                hi2c.Instance->CR1 &= ~(1 << 10);

                // ADDR clear
                dummy = hi2c.Instance->SR2;
                (void)dummy;
            }
            // N >= 3 case
            else
            {
                // after the ADDR is set, clear it
                // read SR2, since SR1 has already been read
                dummy = hi2c.Instance->SR2;
                (void)dummy;

                // enable ITBUFEN
                hi2c.Instance->CR2 |= (1 << 10);
            }

            break;
        }
    }
    // BTF bit 2 in SR1 caused interrupt
    // we are checking if the BTF is set
    else if (sr1_snapshot & (1 << 2))
    {
        switch (hi2c.mode)
        {
        case I2C_TX:
            // we have sent bytes from 0 to TxLength - 1, so TxLength bytes
            if (hi2c.index == hi2c.TxLength)
            {
                hi2c.stop_hits++;
                // issue STOP
                hi2c.Instance->CR1 |= (1 << 9);

                hi2c.state = I2C_STATE_FINISHING;
            }
            else
            {
                if (hi2c.index == hi2c.TxLength - 1)
                {
                    // disable ITBUFEN
                    hi2c.Instance->CR2 &= ~(1 << 10);
                }
                hi2c.Instance->DR = hi2c.pTxBuffPtr[hi2c.index++];
            }
            break;

        case I2C_RX:
            // N = 1: no BTF is possible (shift register never fills with nothing behind it)
            // N = 2 case
            if (hi2c.RxLength == 2)
            {
                // after BTF is read, we have the byte 1 in the DR, the byte 2 in the shift register

                // issue STOP
                hi2c.Instance->CR1 |= (1 << 9);

                // read both bytes (after the byte 1 is read, the byte 2 immediately drops from shift register to the DR)
                hi2c.pRxBuffPtr[0] = hi2c.Instance->DR; // byte 1
                hi2c.pRxBuffPtr[1] = hi2c.Instance->DR; // byte 2

                // after both bytes have been read, clear POS
                hi2c.Instance->CR1 &= ~(1 << 11);

                // after all bytes have been read, re-enable the ACK bit in CR1
                hi2c.Instance->CR1 |= (1 << 10);

                hi2c.state = I2C_STATE_FINISHING;
            }
            else if (hi2c.RxLength >= 3)
            {
                if (hi2c.index == hi2c.RxLength - 3)
                {
                    // if BTF is fired, byte N - 2 in DR, byte N - 1 in shift register

                    // clear ACK bit 10 in CR1
                    hi2c.Instance->CR1 &= ~(1 << 10);

                    // read data N-2
                    hi2c.pRxBuffPtr[hi2c.index] = hi2c.Instance->DR;
                    hi2c.index++;

                    break;
                }

                else if (hi2c.index == hi2c.RxLength - 2)
                {
                    // if BTF is fired, byte N - 1 in DR, byte N in shift register

                    // set STOP high (we have all the bytes we need)
                    hi2c.Instance->CR1 |= (1 << 9);

                    // read bytes N - 1 and N
                    hi2c.pRxBuffPtr[hi2c.index++] = hi2c.Instance->DR; // N - 1

                    hi2c.pRxBuffPtr[hi2c.index] = hi2c.Instance->DR; // N

                    // after all bytes have been read, re-enable the ACK bit in CR1
                    hi2c.Instance->CR1 |= (1 << 10);

                    hi2c.state = I2C_STATE_FINISHING;
                }
                else
                {
                    // fallback
                    // the byte is in DR, the byte is in the shift register, but it is not tail end indexes

                    // read the byte in DR
                    hi2c.pRxBuffPtr[hi2c.index] = hi2c.Instance->DR;

                    hi2c.index++;
                }
            }
            break;

        case I2C_TX_RX:
            if (hi2c.phase == I2C_TX_RX_WRITE)
            {
                // enable ITBUFEN
                if (hi2c.RxLength == 1 || hi2c.RxLength >= 3)
                {
                    hi2c.Instance->CR2 |= (1 << 10);
                }
                // repeated start
                hi2c.Instance->CR1 |= (1 << 8);
                // reset index
                hi2c.index = 0;

                // flip the phase value
                hi2c.phase = I2C_TX_RX_READ;
            }
            else if (hi2c.phase == I2C_TX_RX_READ)
            {
                // N = 1: no BTF is possible (shift register never fills with nothing behind it)
                // N = 2 case
                if (hi2c.RxLength == 2)
                {
                    // after BTF is read, we have the byte 1 in the DR, the byte 2 in the shift register

                    // issue STOP
                    hi2c.Instance->CR1 |= (1 << 9);

                    // read both bytes (after the byte 1 is read, the byte 2 immediately drops from shift register to the DR)
                    hi2c.pRxBuffPtr[0] = hi2c.Instance->DR; // byte 1
                    hi2c.pRxBuffPtr[1] = hi2c.Instance->DR; // byte 2

                    // after both bytes have been read, clear POS
                    hi2c.Instance->CR1 &= ~(1 << 11);

                    // after all bytes have been read, re-enable the ACK bit in CR1
                    hi2c.Instance->CR1 |= (1 << 10);

                    hi2c.state = I2C_STATE_FINISHING;
                }
                else if (hi2c.RxLength >= 3)
                {
                    if (hi2c.index == hi2c.RxLength - 3)
                    {
                        // if BTF is fired, byte N - 2 in DR, byte N - 1 in shift register

                        // clear ACK bit 10 in CR1
                        hi2c.Instance->CR1 &= ~(1 << 10);

                        // read data N-2
                        hi2c.pRxBuffPtr[hi2c.index] = hi2c.Instance->DR;
                        hi2c.index++;

                        break;
                    }

                    else if (hi2c.index == hi2c.RxLength - 2)
                    {
                        // if BTF is fired, byte N - 1 in DR, byte N in shift register

                        // set STOP high (we have all the bytes we need)
                        hi2c.Instance->CR1 |= (1 << 9);

                        // read bytes N - 1 and N
                        hi2c.pRxBuffPtr[hi2c.index++] = hi2c.Instance->DR; // N - 1

                        hi2c.pRxBuffPtr[hi2c.index] = hi2c.Instance->DR; // N

                        // after all bytes have been read, re-enable the ACK bit in CR1
                        hi2c.Instance->CR1 |= (1 << 10);

                        hi2c.state = I2C_STATE_FINISHING;
                    }

                    else
                    {
                        // fallback
                        // the byte is in DR, the byte is in the shift register, but it is not tail end indexes

                        // read the byte in DR
                        hi2c.pRxBuffPtr[hi2c.index] = hi2c.Instance->DR;

                        hi2c.index++;
                    }
                }
            }

            break;
        }
    }

    // RxNE bit 6 in SR1 caused interrupt
    else if (sr1_snapshot & (1 << 6))
    {

        if (hi2c.RxLength == 1)
        {
            // stop is already set after the clearing of the ADDR

            // read the byte
            hi2c.pRxBuffPtr[0] = hi2c.Instance->DR;

            // enabled ACK
            hi2c.Instance->CR1 |= (1 << 10);

            // ITBUFEN is not disabled

            hi2c.state = I2C_STATE_FINISHING;
        }

        // RxLength == 2 does not use RxE

        // RxLength >= 3
        else if (hi2c.RxLength >= 3)
        {
            if (hi2c.index < hi2c.RxLength - 2)
            {
                hi2c.pRxBuffPtr[hi2c.index++] = hi2c.Instance->DR;

                if (hi2c.index == hi2c.RxLength - 3)
                {
                    // disable ITBUFEN, so the last two bytes are BTF-only
                    hi2c.Instance->CR2 &= ~(1 << 10);
                }
            }
        }
    }
    // TxE bit 7 in SR1 caused interrupt
    else if (sr1_snapshot & (1 << 7))
    {
        if (hi2c.index < hi2c.TxLength)
        {
            if (hi2c.index == hi2c.TxLength - 1)
            {
                // disable ITBUFEN
                hi2c.Instance->CR2 &= ~(1 << 10);
            }
            hi2c.Instance->DR = hi2c.pTxBuffPtr[hi2c.index++];
        }
    }

    return;
}

// Error Interrupts Handler (priority 39 - can be preempted by I2C1_EV)
void I2C1_ER_IRQHandler(void)
{
    // disable interrupts
    __disable_irq();

    // populate the error_code

    // make snapshot of current state of SR1
    uint16_t sr1_snap = hi2c.Instance->SR1;

    uint16_t sr1_errors_snap = (sr1_snap & ((uint16_t)0xF << 8));

    // isolate the errors bits 11:8 and assigning to the error_code
    // 0b00001111 = 2^3 + 2^2 + 2^1 + 2^0 = 8 + 4 + 2 + 1 = 15 = 0xF
    hi2c.error_code = sr1_errors_snap;

    // error code checking branches
    if (hi2c.error_code & I2C_ERROR_BERR)
    {
        // needs SWRST hard reset

        // since I immediately enable SWRST, there is no explicit SR1 error flag clearing
        // the SR1 flags cleared immediately by SWRST

        // set SWRST to 1
        hi2c.Instance->CR1 |= (1 << 15);

        // clear the pending EV interrupt
        // I2C1_EV has a position of 31
        // one ICPR has 32 Interrupt-related bits (0...31)

        // with NVIC the write-1-to-clear is the tool that allows to prevent race conditions if another interrupt fires mid-execution
        NVIC->ICPR[0] = (1 << 31);
    }
    else
    {
        if (hi2c.error_code & I2C_ERROR_ARLO)
        {
            // needs to wait until the current transmission is over because the another master seized the bus

            // clear the SR1 ARLO error flag
            // a single write to prevent late-incoming bits to SR1 from overwriting
            // bits 13 and 5 are reserved bits, write 0 to them
            hi2c.Instance->SR1 = ~((1 << 13) | (1 << 9) | (1 << 5));

            // no stop issuing since the microcontroller is no longer a master
        }
        if (hi2c.error_code & I2C_ERROR_AF)
        {
            // needs to issue STOP and then try the communication again

            // make snapshot of current CR1
            uint16_t cr1_snap = hi2c.Instance->CR1;
            uint32_t cr1_modified = cr1_snap;

            // clear the SR1 AF error flag
            // ~(1 << 10) = 11111011....
            hi2c.Instance->SR1 = ~((1 << 13) | (1 << 10) | (1 << 5));

            // check whether there was no ARLO error
            // if there is an ARLO error, that means that the microcontroler stopped being a master of the lines, it is now a target that just listens to the bus and waits when the controller that seized the bus concludes the communication.
            // AF recovery sequence however issues STOP. the MCU in target mode just releases the lines letting them float as a STOP sequence. so if there are both ARLO and AF errors simultaneously, the stop issuing in the AF Recovery sequence disappears
            if (!(sr1_errors_snap & I2C_ERROR_ARLO))
            {
                // issue STOP condition
                // set the STOP bit to 1
                cr1_modified |= (1 << 9);
            }

            // set the ACK bit to 1
            cr1_modified |= (1 << 10);
            // clear the POS bit
            cr1_modified &= ~(1 << 11);

            // direct write of the modified CR1 to I2C->CR1
            hi2c.Instance->CR1 = cr1_modified;
        }
    }

    // set the state to ERROR
    hi2c.state = I2C_STATE_ERROR;

    // enable interrupts
    __enable_irq();

    return;
}

static uint8_t I2C_Validate_Pins()
{
    if (hi2c.channel == I2C_CHANNEL_1)
    {
        if (hi2c.scl_port == GPIOB && hi2c.sda_port == GPIOB)
        {
            if ((hi2c.scl_pin == 6 || hi2c.scl_pin == 8) && (hi2c.sda_pin == 7 || hi2c.sda_pin == 9))
            {
                return 1;
            }
        }
    }
    else if (hi2c.channel == I2C_CHANNEL_2)
    {
        if (hi2c.scl_port == GPIOB && hi2c.sda_port == GPIOB)
        {
            if ((hi2c.scl_pin == 10) && (hi2c.sda_pin == 3 || hi2c.sda_pin == 9))
            {
                return 1;
            }
        }
    }
    else if (hi2c.channel == I2C_CHANNEL_3)
    {
        if (hi2c.scl_port == GPIOA && hi2c.sda_port == GPIOB)
        {
            if ((hi2c.scl_pin == 8) && (hi2c.sda_pin == 4 || hi2c.sda_pin == 8))
            {
                return 1;
            }
        }
    }

    return 0;
}

// TODO: timeout mechanism via hi2c->get_tick_ms function pointer
// curretly spins indefinitely if hardware becomes unresponsive
// must be implemented before production use

void I2C_Init()
{
    if ((hi2c.channel >= I2C_CHANNEL_MAX) || !I2C_Validate_Pins())
    {
        return;
    }

    /* ----- I2C Choosing ----- */
    if (hi2c.channel == 1)
    {
        hi2c.Instance = I2C1;
    }
    else if (hi2c.channel == 2)
    {
        hi2c.Instance = I2C2;
    }
    else
    {
        hi2c.Instance = I2C3;
    }
    /* ----- I2C Choosing ----- */

    switch (hi2c.channel)
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
    hi2c.scl_port->MODER &= ~(0x3 << (2 * hi2c.scl_pin));
    hi2c.sda_port->MODER &= ~(0x3 << (2 * hi2c.sda_pin));

    // set the MODER bits
    // 10 = 0x2
    hi2c.scl_port->MODER |= (0x2 << (2 * hi2c.scl_pin));
    hi2c.sda_port->MODER |= (0x2 << (2 * hi2c.sda_pin));

    /* ----- set the AFRL for SCL pin and SDA pin to AF4 ----- */
    // AF4 value: 0100

    // AFRL/AFRH uses 4 bits per 1 pin

    if (hi2c.scl_pin >= 8)
    {
        // pins 8-15 are set in AFRH

        // clear the AFRH bits
        // 1111 = 2^3 + 2^2 + 2^1 + 2^0 = 8 + 4 + 2 + 1 = 15 = 0xF
        hi2c.scl_port->AFRH &= ~(0xF << (4 * (hi2c.scl_pin - 8))); // scl_pin - 8 to prevent overflow of 32 bit register

        // set the bits to AF4
        hi2c.scl_port->AFRH |= (GPIO_AF4 << (4 * (hi2c.scl_pin - 8)));
    }
    else
    {
        // pins 0-7 are set in AFRL

        // clear the AFRL bits
        // 1111 = 2^3 + 2^2 + 2^1 + 2^0 = 8 + 4 + 2 + 1 = 15 = 0xF
        hi2c.scl_port->AFRL &= ~(0xF << (4 * hi2c.scl_pin));

        // set the bits to AF4
        hi2c.scl_port->AFRL |= (GPIO_AF4 << (4 * hi2c.scl_pin));
    }

    if (hi2c.sda_pin >= 8)
    {
        // pins 8-15 are set in AFRH

        // clear the AFRH bits
        // 1111 = 2^3 + 2^2 + 2^1 + 2^0 = 8 + 4 + 2 + 1 = 15 = 0xF
        hi2c.sda_port->AFRH &= ~(0xF << (4 * (hi2c.sda_pin - 8))); // sda_pin - 8 to prevent overflow of 32 bit register

        // set the bits to AF4
        hi2c.sda_port->AFRH |= (GPIO_AF4 << (4 * (hi2c.sda_pin - 8)));
    }
    else
    {
        // pins 0-7 are set in AFRL

        // clear the AFRL bits
        // 1111 = 2^3 + 2^2 + 2^1 + 2^0 = 8 + 4 + 2 + 1 = 15 = 0xF
        hi2c.sda_port->AFRL &= ~(0xF << (4 * hi2c.sda_pin));

        // set the bits to AF4
        hi2c.sda_port->AFRL |= (GPIO_AF4 << (4 * hi2c.sda_pin));
    }

    /* ----- set the OTYPER for SCL pin and SDA pin to Open-drain ----- */
    // value 1

    hi2c.scl_port->OTYPER |= (1 << hi2c.scl_pin);
    hi2c.sda_port->OTYPER |= (1 << hi2c.sda_pin);

    /* ----- explicitly clear the PUPDR for SCL and SDA pins ----- */
    // since PUPDR give 2 bits per pin, we are using (2 * SCL pin number), (2 * SDA pin number)

    // 11 = 0x3
    hi2c.scl_port->PUPDR &= ~(0x3 << (2 * hi2c.scl_pin));
    hi2c.sda_port->PUPDR &= ~(0x3 << (2 * hi2c.sda_pin));

    /* ----- I2C CCR configuration ----- */

    // declare a local I2C var to simplify the naming
    I2C_RegDef_t *I2C = hi2c.Instance;

    // explicitly disable PE (bit 0 CR1) to prevet undefined hardware behavior
    // if the PE is already 1 after reboot or SWRST recovery
    I2C->CR1 &= ~(1 << 0);

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

    /* ----- Interrupts enablement (NVIC) ----- */

    // I2C_EV has position 31 in the vector table
    // ISER[0] pos 31
    NVIC->ISER[0] = (1 << 31);

    // I2C_ER has position 32 in the vector table
    // ISER[1] pos 1
    NVIC->ISER[1] = (1 << 0);

    // set priorities
    // EV = 5, ER = 6
    // range: 0 (highest), 15 (lowest)
    NVIC->IPR[31] = (5U << 4);
    NVIC->IPR[32] = (6U << 4);

    /* ----- Interrupts enablement (CR2) -----*/

    // set ITERREN (Error interrupt enable) Bit 8 to 1
    I2C->CR2 |= (1 << 8);

    // set ITEVTEN (Event interrupt enable) Bit 9 to 1
    I2C->CR2 |= (1 << 9);

    // the ITBUFEN will be enabled dynamically when we are actively ready to transmit or receive bytes
}

uint8_t I2C_PollHardwareBusy(I2C_RegDef_t *I2C)
{
    uint32_t start = SysTick_GetTick();

    while ((SysTick_GetTick() - start) <= 4)
    {
        uint32_t sr1_snap = I2C->SR1;
        uint32_t sr2_snap = I2C->SR2;
        // firstly, check the error flags
        if (sr1_snap & ((1 << 10) | (1 << 9) | (1 << 8)))
        {
            // if there are errors, clear the error flags by writing 0 to them
            I2C->SR1 &= ~((1 << 10) | (1 << 9) | (1 << 8));

            // generate STOP condition to gracefuly and safely release the lines
            I2C->CR1 |= (1 << 9);

            hi2c.error_code = I2C_ERROR_BERR;
            hi2c.state = I2C_STATE_ERROR;

            return 1;
        }

        if (!(sr2_snap & (1 << 1)))
        {
            // if the BUSY bit is 0, exit the loop
            return 0;
        }
    }

    // if timeout is over and the function does not exit
    // the bus is stuck

    __disable_irq();
    hi2c.error_code = I2C_ERROR_BERR;
    hi2c.Instance->CR1 |= (1 << 15);
    hi2c.state = I2C_STATE_ERROR;
    __enable_irq();

    return 1;
}

I2C_Status_t I2C_Master_Transmit(uint8_t slave_addr, uint8_t *data, uint8_t length)
{
    if (hi2c.Instance == NULL || data == NULL || length == 0)
    {
        return I2C_ERROR;
    }

    hi2c.slave_add = slave_addr;
    hi2c.pTxBuffPtr = data;
    hi2c.TxLength = length;
    hi2c.pRxBuffPtr = NULL;
    hi2c.RxLength = 0;
    hi2c.mode = I2C_TX;
    hi2c.index = 0;
    hi2c.error_code = 0;

    // wait until the bus is completely idle (BUSY = 0)
    if (I2C_PollHardwareBusy(hi2c.Instance))
    {
        return I2C_ERROR;
    }

    // hi2c.state = I2C_STATE_TX_ADDR;
    hi2c.state = I2C_STATE_START_PENDING;
    // start the transaction by issuing START (set to 1)
    hi2c.Instance->CR1 |= (1 << 8);
    // START is cleared by hardware when start is sent

    return I2C_OK;
}

I2C_Status_t I2C_Master_Receive(uint8_t slave_addr, uint8_t *data, uint8_t length)
{
    if (hi2c.Instance == NULL || data == NULL || length == 0)
    {
        return I2C_ERROR;
    }

    hi2c.slave_add = slave_addr;
    hi2c.pRxBuffPtr = data;
    hi2c.RxLength = length;
    hi2c.TxLength = 0;
    hi2c.pTxBuffPtr = NULL;
    hi2c.mode = I2C_RX;
    hi2c.index = 0;
    hi2c.error_code = 0;

    // wait until the bus is completely idle (BUSY = 0)
    if (I2C_PollHardwareBusy(hi2c.Instance))
    {
        return I2C_ERROR;
    }

    hi2c.state = I2C_STATE_RX_ADDR;
    // Master issues the Start
    hi2c.Instance->CR1 |= (1 << 8);
    // hardware clears it after the start is sent

    return I2C_OK;
}

I2C_Status_t I2C_Master_Transmit_Receive(uint8_t slave_addr, uint8_t *pSend, uint8_t *pReceive, uint8_t send_length, uint8_t receive_length)
{
    if (hi2c.Instance == NULL || pSend == NULL || pReceive == NULL || send_length == 0 || receive_length == 0)
    {
        return I2C_ERROR;
    }

    hi2c.slave_add = slave_addr;
    hi2c.pTxBuffPtr = pSend;
    hi2c.TxLength = send_length;
    hi2c.pRxBuffPtr = pReceive;
    hi2c.RxLength = receive_length;
    hi2c.mode = I2C_TX_RX;
    hi2c.phase = I2C_TX_RX_WRITE;
    hi2c.index = 0;
    hi2c.error_code = 0;

    // wait until the bus is completely idle (BUSY == 0)
    if (I2C_PollHardwareBusy(hi2c.Instance))
    {
        return I2C_ERROR;
    }

    hi2c.state = I2C_STATE_TX_ADDR;
    // Master issues the Start condition
    hi2c.Instance->CR1 |= (1 << 8);
    // the hardware clears this bit when the start is sent

    return I2C_OK;
}

void I2C_PollStopConfirmation()
{
    uint32_t start = SysTick_GetTick();
    uint8_t success = 0;

    while ((SysTick_GetTick() - start) <= 2)
    {
        // BUSY bit 1 polling
        if (!(hi2c.Instance->SR2 & (1 << 1)))
        {
            // if it is 0, the STOP is detected, the lines are settled
            success = 1;
            break;
        }
    }

    if (success)
    {
        hi2c.state = I2C_STATE_DONE;
    }
    else
    {
        // in reality it is a software-detected STOP-confirmation timeout, not a real BERR
        hi2c.error_code = I2C_ERROR_BERR;
        hi2c.state = I2C_STATE_ERROR;
    }

    return;
}

void I2C_Process(void)
{
    // Firstly read index, then state
    uint8_t current_index = hi2c.index;

    I2C_State_t current_state = hi2c.state;

    if (current_state == I2C_STATE_FINISHING)
    {
        I2C_PollStopConfirmation();
    }

    else if (current_state == I2C_STATE_ERROR)
    {
        if (hi2c.error_code & I2C_ERROR_BERR)
        {
            // the SWRST is already 1 and not yet cleared

            // we need to check if the lines are released and the bus is free
            // for that, we need to check GPIO_IDR values for the pins
            uint32_t bus_is_free = 0;

            uint32_t sda_val = 0;
            uint32_t scl_val = 0;

            uint32_t start = SysTick_GetTick();

            while ((SysTick_GetTick() - start) <= 2)
            {
                sda_val = hi2c.sda_port->IDR & (1 << hi2c.sda_pin);
                scl_val = hi2c.scl_port->IDR & (1 << hi2c.scl_pin);

                // if the lines both are 1, then the bus is naturally free
                if (sda_val && scl_val)
                {
                    bus_is_free = 1;
                    break;
                }
            }

            // the waiting IDR loop exits and bus is still not free
            if (!bus_is_free)
            {
                // gpio clock-banging

                // first of all, change the MODER for SCL and SDA lines

                // 1.  SCL to General purpose output mode (01)

                uint8_t bus_released = 0;

                // clear the bits (each pin has 2 bits)
                // 0b0011 = 2^1 + 2^0 = 3 = 0x3
                hi2c.scl_port->MODER &= ~(0x3 << (hi2c.scl_pin * 2));

                // set the bits
                // 01 = 2^0 = 1 = 0x1
                hi2c.scl_port->MODER |= (0x1 << (hi2c.scl_pin * 2));

                // make sure the SCL has OTYPER at Open-Drain
                hi2c.scl_port->OTYPER |= (1 << hi2c.scl_pin);

                // 2. SDA to Input (00)
                // we just want to read the true state of the wire
                hi2c.sda_port->MODER &= ~(0x3 << (hi2c.sda_pin * 2));

                // make sure the SCL line is high
                hi2c.scl_port->BSRR = (1 << hi2c.scl_pin);
                // BSRR (Bit Set/Reset Register) allows to change the state of individual pins atomically (in a single CPU instruction cycle) without using a read-modify-write operation

                // a small delay to give time electrical changes to occur on the bus
                for (volatile uint32_t i = 0; i < 50; i++)
                    ;

                // manually toggle SCL via GPIO to walk the slave's internal shift counter forward uint it releases SDA
                for (uint8_t i = 0; i < 9; i++)
                {
                    // check if the SDA is not released
                    if (hi2c.sda_port->IDR & (1 << hi2c.sda_pin))
                    {
                        // if released, exit early
                        bus_released = 1;
                        break;
                    }

                    // pull the SCL line low to step forward in the slave's internal state machine
                    hi2c.scl_port->BSRR = (1 << (hi2c.scl_pin + 16));

                    for (volatile uint32_t i = 0; i < 50; i++)
                        ;

                    // release the SCL line high
                    hi2c.scl_port->BSRR = (1 << hi2c.scl_pin);

                    for (volatile uint32_t i = 0; i < 50; i++)
                        ;
                }

                if (bus_released)
                {
                    // generate stop
                    // STOP sequence: SCL goes low, SDA goes low, SCL goes high, SDA goes high

                    // first of all, configure the SDA pin as General Purpose Output (01)

                    // clear the bits (a pin has 2 bits in MODER)
                    // 0b0011 = 0x3
                    hi2c.sda_port->MODER &= ~(0x3 << (hi2c.sda_pin * 2));

                    // set the bits
                    // 0b0001 = 0x1
                    hi2c.sda_port->MODER |= (0x1 << (hi2c.sda_pin * 2));

                    // make sure that SDA pin is Open-Drain
                    hi2c.sda_port->OTYPER |= (1 << hi2c.sda_pin);

                    /* ----- SEQUENCE -----*/
                    // SCL goes low
                    hi2c.scl_port->BSRR = (1 << (hi2c.scl_pin + 16));
                    for (volatile uint32_t i = 0; i < 50; i++)
                        ;

                    // SDA goes low
                    hi2c.sda_port->BSRR = (1 << (hi2c.sda_pin + 16));
                    for (volatile uint32_t i = 0; i < 50; i++)
                        ;

                    // SCL goes high
                    hi2c.scl_port->BSRR = (1 << hi2c.scl_pin);
                    for (volatile uint32_t i = 0; i < 50; i++)
                        ;

                    // SDA goes high
                    hi2c.sda_port->BSRR = (1 << hi2c.sda_pin);
                    for (volatile uint32_t i = 0; i < 50; i++)
                        ;
                }

                // return the pins to AF
                /* ----- set the MODER for SCL pin and SDA pin to Alternate function ----- */
                // value: 10

                // one pin takes 2 bits, so (2 * pin_number):
                // e.g. pin 6 takes 2*6 = 12 => 13:12
                // pin 7 takes 2*7 = 14 => 15:14
                // pin 0 takes 2*0 = 0 => 1:0

                // clear the MODER bits
                // 11 = 0x3
                hi2c.scl_port->MODER &= ~(0x3 << (2 * hi2c.scl_pin));
                hi2c.sda_port->MODER &= ~(0x3 << (2 * hi2c.sda_pin));

                // set the MODER bits
                // 10 = 0x2
                hi2c.scl_port->MODER |= (0x2 << (2 * hi2c.scl_pin));
                hi2c.sda_port->MODER |= (0x2 << (2 * hi2c.sda_pin));

                /* ----- set the AFRL for SCL pin and SDA pin to AF4 ----- */
                // AF4 value: 0100

                // AFRL/AFRH uses 4 bits per 1 pin

                if (hi2c.scl_pin >= 8)
                {
                    // pins 8-15 are set in AFRH

                    // clear the AFRH bits
                    // 1111 = 2^3 + 2^2 + 2^1 + 2^0 = 8 + 4 + 2 + 1 = 15 = 0xF
                    hi2c.scl_port->AFRH &= ~(0xF << (4 * (hi2c.scl_pin - 8))); // scl_pin - 8 to prevent overflow of 32 bit register

                    // set the bits to AF4
                    hi2c.scl_port->AFRH |= (GPIO_AF4 << (4 * (hi2c.scl_pin - 8)));
                }
                else
                {
                    // pins 0-7 are set in AFRL

                    // clear the AFRL bits
                    // 1111 = 2^3 + 2^2 + 2^1 + 2^0 = 8 + 4 + 2 + 1 = 15 = 0xF
                    hi2c.scl_port->AFRL &= ~(0xF << (4 * hi2c.scl_pin));

                    // set the bits to AF4
                    hi2c.scl_port->AFRL |= (GPIO_AF4 << (4 * hi2c.scl_pin));
                }

                if (hi2c.sda_pin >= 8)
                {
                    // pins 8-15 are set in AFRH

                    // clear the AFRH bits
                    // 1111 = 2^3 + 2^2 + 2^1 + 2^0 = 8 + 4 + 2 + 1 = 15 = 0xF
                    hi2c.sda_port->AFRH &= ~(0xF << (4 * (hi2c.sda_pin - 8))); // sda_pin - 8 to prevent overflow of 32 bit register

                    // set the bits to AF4
                    hi2c.sda_port->AFRH |= (GPIO_AF4 << (4 * (hi2c.sda_pin - 8)));
                }
                else
                {
                    // pins 0-7 are set in AFRL

                    // clear the AFRL bits
                    // 1111 = 2^3 + 2^2 + 2^1 + 2^0 = 8 + 4 + 2 + 1 = 15 = 0xF
                    hi2c.sda_port->AFRL &= ~(0xF << (4 * hi2c.sda_pin));

                    // set the bits to AF4
                    hi2c.sda_port->AFRL |= (GPIO_AF4 << (4 * hi2c.sda_pin));
                }
            }

            // clear the SWRST bit to move the peripheral out of reset
            hi2c.Instance->CR1 &= ~(1 << 15);

            // reinitialize the I2C peripheral
            I2C_Reinit();

            hi2c.state = I2C_STATE_IDLE;
            hi2c.error_code = I2C_ERROR_NONE;
            return;
        }
        else if (hi2c.error_code & (I2C_ERROR_AF | I2C_ERROR_ARLO))
        {
            // TODO: add SysTick timeout, SWRST escalation
            hi2c.state = I2C_STATE_IDLE;
            hi2c.error_code = I2C_ERROR_NONE;
            return;
        }
    }
}