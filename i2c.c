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

// TODO: timeout mechanism via hi2c->get_tick_ms function pointer
// curretly spins indefinitely if hardware becomes unresponsive
// must be implemented before production use

void I2C_Init(I2C_HandleTypeDef *hi2c)
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

    /* ----- explicitly clear the PUPDR for SCL and SDA pins ----- */
    // since PUPDR give 2 bits per pin, we are using (2 * SCL pin number), (2 * SDA pin number)

    // 11 = 0x3
    hi2c->scl_port->PUPDR &= ~(0x3 << (2 * hi2c->scl_pin));
    hi2c->sda_port->PUPDR &= ~(0x3 << (2 * hi2c->sda_pin));

    /* ----- I2C CCR configuration ----- */

    // declare a local I2C var to simplify the naming
    I2C_RegDef_t *I2C = hi2c->Instance;

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
}

static uint8_t I2C_PollHardwareFlags(I2C_RegDef_t *I2C, I2C_Bit_Masks_t bit_mask)
{
    while (1)
    {
        uint32_t sr1_snap = I2C->SR1;

        // firstly, check the error flags
        if (sr1_snap & ((1 << 11) | (1 << 10) | (1 << 9) | (1 << 8)))
        {
            uint32_t cr1_snap = I2C->CR1;

            sr1_snap &= ~((1 << 11) | (1 << 10) | (1 << 9) | (1 << 8));

            // if there are errors, clear the error flags by writing 0 to them
            I2C->SR1 = sr1_snap;

            uint32_t cr1_modified = cr1_snap;

            // set the STOP bit to 1
            cr1_modified |= (1 << 9);
            // set the ACK bit to 1
            cr1_modified |= (1 << 10);
            // clear the POS bit
            cr1_modified &= ~(1 << 11);

            // direct write of the modified CR1 to I2C->CR1
            I2C->CR1 = cr1_modified;

            return 1;
        }

        // secondly, check the bit
        if (sr1_snap & bit_mask)
        {
            // if the bit is set, exit the loop
            return 0;
        }
    }
}

uint8_t I2C_PollHardwareBusy(I2C_RegDef_t *I2C)
{
    while (1)
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

            return 1;
        }

        if (!(sr2_snap & (1 << 1)))
        {
            // if the BUSY bit is 0, exit the loop
            return 0;
        }
    }
}

I2C_Status_t I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint8_t slave_addr, uint8_t *data, uint8_t length)
{
    if (hi2c == NULL || hi2c->Instance == NULL || data == NULL || length == 0)
    {
        return I2C_ERROR;
    }

    // declare a local I2C var to simplify the naming
    I2C_RegDef_t *I2C = hi2c->Instance;

    // wait until the bus is completely idle (BUSY = 0)
    if (I2C_PollHardwareBusy(I2C))
    {
        return I2C_ERROR;
    }

    // start the transaction by issuing START (set to 1)
    I2C->CR1 |= (1 << 8);
    // START is cleared by hardware when start is sent

    // polling the SB (Start bit)
    // when exit, the bit is set, start condition generated
    if (I2C_PollHardwareFlags(I2C, I2C_SB_MASK))
    {
        return I2C_ERROR;
    }

    // SB is cleared by reading SR1 (done in the poll above) + writing DR
    // SR1 read is already consumed by the polling loop - writing DR now completes the clear

    // write the slave address to the DR register (it will start sending the bits to the slave and clear the SB bit)
    // 7 bit slave address + W (Write) bit 0
    I2C->DR = (slave_addr << 1) & ~0x01; // 0x01 = 2^0 = 1 = (1 << 0)

    // polling the ADDR (Address sent) bit
    // the bit is set after the ACK of the byte
    if (I2C_PollHardwareFlags(I2C, I2C_ADDR_MASK))
    {
        return I2C_ERROR;
    }

    // read the SR1 and SR2 to clear the ADDR bit in the SR1
    // SR1 read is already consumed by the polling loop above - reading SR2 now completes the clear
    uint32_t dummy = I2C->SR2;
    (void)dummy; // preventing the compiler warnings of an unused variable

    // ensure we are the Master (MSL, bit 0 = 1) and Transmitter (TRA, bit 2 = 1)
    // 0b0101 = 2^2 + 2^0 = 4 + 1 = 5 = 0x05
    if ((dummy & ((1 << 2) | (1 << 0))) != 0x05)
    {
        // if we enter this if body, the hardware is desynchronized
        // force STOP
        I2C->CR1 |= (1 << 9);
        return I2C_ERROR;
    }

    for (uint8_t i = 0; i < length; i++)
    {
        // polling the TxE Transmitters Data register empty
        // goes high after every byte leaves the DR
        if (I2C_PollHardwareFlags(I2C, I2C_TXE_MASK))
        {
            return I2C_ERROR;
        }

        I2C->DR = data[i];
    }

    // before sending the STOP condition
    // we need to make sure the byte has left the shift register, so we are not going to corrupt the last byte mid-sending
    // polling the Byte transfer finished
    if (I2C_PollHardwareFlags(I2C, I2C_BTF_MASK))
    {
        return I2C_ERROR;
    }

    // issuing the Stop generation
    I2C->CR1 |= (1 << 9);

    return I2C_OK;
}

I2C_Status_t I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint8_t slave_addr, uint8_t *data, uint8_t length)
{
    if (hi2c == NULL || data == NULL || length == 0)
    {
        return I2C_ERROR;
    }

    // declare a local I2C var to simplify the naming
    I2C_RegDef_t *I2C = hi2c->Instance;

    // wait until the bus is completely idle (BUSY = 0)
    if (I2C_PollHardwareBusy(I2C))
    {
        return I2C_ERROR;
    }

    // Master issues the Start
    I2C->CR1 |= (1 << 8);
    // hardware clears it after the start is sent

    // poll the SB bit 0 in SR1
    if (I2C_PollHardwareFlags(I2C, I2C_SB_MASK))
    {
        return I2C_ERROR;
    }

    // clear the SB bit 0 in SR1 by readimg the SR1 and then writing the DR with the slave's address
    // SR1 read is consumed in the loop above, so the DR write completes a clear

    // write a peripheral address to the DR
    // since we want to read data bytes from the peripheral, we are setting the bit 0 (R) as 0x01 = 1
    I2C->DR = (slave_addr << 1) | (0x01 << 0);

    // poll the ADDR
    // hardware sets it to 1 after the ACK of the byte, so after the peripheral has acknowledged its address
    if (I2C_PollHardwareFlags(I2C, I2C_ADDR_MASK))
    {
        return I2C_ERROR;
    }

    /* ----- ONE-BYTE SCENARIO ----- */
    if (length == 1)
    {
        // we have already polled ADDR

        // clear the ACK (Bit 10 CR1)
        I2C->CR1 &= ~(1 << 10);
    }
    /* ----- ONE-BYTE SCENARIO ----- */

    // read the SR1 and SR2 to clear the ADDR bit in the SR1
    // SR1 read is consumed in the ADDR polling loop, so SR2 read completes a clear
    uint32_t dummy = I2C->SR2;
    (void)dummy; // preventing the compiler warnings of an unused variable

    // ensure we are the Master (MSL, bit 0 = 1) and Receiver (TRA, bit 2 = 0)
    // 0b0001 = 2^0 = 1 = 0x01
    if ((I2C->SR2 & ((1 << 2) | (1 << 0))) != 0x01)
    {
        // if we enter this if body, the hardware is desynchronized
        // force STOP
        I2C->CR1 |= (1 << 9);
        return I2C_ERROR;
    }

    /* ----- ONE-BYTE SCENARIO ----- */
    if (length == 1)
    {
        // set STOP condition
        I2C->CR1 |= (1 << 9);

        // wait for a byte from the peripheral (poll RxNE Bit 6)
        if (I2C_PollHardwareFlags(I2C, I2C_RXNE_MASK))
        {
            return I2C_ERROR;
        }

        *data = I2C->DR;

        // enable ACK
        I2C->CR1 |= (1 << 10);

        // the first and simultaneously the last byte of data has been received, so exit the function
        return I2C_OK;
    }
    /* ----- ONE-BYTE SCENARIO ----- */

    for (uint8_t i = 0; i < length; i++)
    {
        // check if we are about to handle the second-to-last byte
        if (i == length - 2)
        {
            // if so, we need to clear ACK bit in CR1 to prevent the peripheral to send us unwanted bytes
            I2C->CR1 &= ~(1 << 10);
        }

        // wait for a byte from the peripheral (poll RxNE Bit 6)
        if (I2C_PollHardwareFlags(I2C, I2C_RXNE_MASK))
        {
            return I2C_ERROR;
        }

        data[i] = I2C->DR;

        // check if we have handled the second-to-last byte
        if (i == length - 2)
        {
            // if so, the last byte is already clocking in, so we need to issue the STOP condition
            I2C->CR1 |= (1 << 9);
        }
    }

    // after the data was received and the STOP issue was issued, re-enble the ACK
    I2C->CR1 |= (1 << 10);

    return I2C_OK;
}

I2C_Status_t I2C_Master_Transmit_Receive(I2C_HandleTypeDef *hi2c, uint8_t slave_addr, uint8_t *pSend, uint8_t *pReceive, uint8_t send_length, uint8_t receive_length)
{
    if (hi2c == NULL || pSend == NULL || pReceive == NULL || send_length == 0 || receive_length == 0)
    {
        return I2C_ERROR;
    }

    // declare a local I2C var to simplify the naming
    I2C_RegDef_t *I2C = hi2c->Instance;

    // wait until the bus is completely idle (BUSY == 0)
    if (I2C_PollHardwareBusy(I2C))
    {
        return I2C_ERROR;
    }

    // Master issues the Start condition
    I2C->CR1 |= (1 << 8);
    // the hardware clears this bit when the start is sent

    if (I2C_PollHardwareFlags(I2C, I2C_SB_MASK))
    {
        return I2C_ERROR;
    }

    // clear the SB bit by reading the SR1 and writing the DR register
    // SR1 read was done in a polling loop above, so the next DR writing completes the clear

    /* ----- WRITE PART ----- */

    // write the slave's address (7 bits) + the bit 0 of 0 (Writing) (the Master sending bytes over line to the slave)
    I2C->DR = (slave_addr << 1) & ~(0x01 << 0);

    // poll the ADDR bit 1 in SR1
    // it is set by hardware when the peripheral acknowledges its address

    // to eliminate cases when there is firstly an error, and then the ADDR bit hits, so the polling loop exits without handling the error
    // check the error flags first and handle them, then check for the ADDR bit

    if (I2C_PollHardwareFlags(I2C, I2C_ADDR_MASK))
    {
        return I2C_ERROR;
    }

    // clear the ADDR bit
    // SR1 read was done in the poll loop above, so only SR2 read remained to complete the clear
    uint32_t dummy = I2C->SR2;
    (void)dummy;

    // ensure we are the Master (MSL, bit 0 = 1) and Transmitter (TRA, bit 2 = 1)
    // 0b0101 = 2^2 + 2^0 = 4 + 1 = 5 = 0x05
    if ((dummy & ((1 << 2) | (1 << 0))) != 0x05)
    {
        // if we enter this if body, the hardware is desynchronized
        // force STOP
        I2C->CR1 |= (1 << 9);
        return I2C_ERROR;
    }

    // send the data
    for (uint8_t i = 0; i < send_length; i++)
    {
        // poll the TxE Data register empty (transmitters)
        if (I2C_PollHardwareFlags(I2C, I2C_TXE_MASK))
        {
            return I2C_ERROR;
        }

        I2C->DR = pSend[i];
    }

    // poll the BTF bit 2 in SR1
    // to prevent corrupting the byte that could be still in the shift registers
    if (I2C_PollHardwareFlags(I2C, I2C_BTF_MASK))
    {
        return I2C_ERROR;
    }

    /* ----- WRITE PART ----- */

    /* ----- READ PART ----- */
    // generate the Repeated start
    // by setting the START bit 8 in CR1 to 1
    I2C->CR1 |= (1 << 8);

    // poll the SB bit 0
    if (I2C_PollHardwareFlags(I2C, I2C_SB_MASK))
    {
        return I2C_ERROR;
    }

    // clear the SB bit 0 by reading the SR1 and writing to the DR
    // SR1 read is consumed in the loop above, so the DR writing completes a clear

    // write to the DR a slave's address (7 bits) + last bit 0 as 1 (R) becasue the Master wants to read the data from the slave
    I2C->DR = (slave_addr << 1) | (0x01 << 0);

    // poll the ADDR
    if (I2C_PollHardwareFlags(I2C, I2C_ADDR_MASK))
    {
        return I2C_ERROR;
    }

    /* ---------- ONE-BYTE SCENARIO ---------- */
    if (receive_length == 1)
    {
        // immediately after the ADDR bit hits, clear the ACK bit in the CR1 to prevent the peripheral from eventual sending of unwanted bytes
        I2C->CR1 &= ~(1 << 10);
    }
    /* ---------- ONE-BYTE SCENARIO ---------- */

    /* ---------- TWO-BYTES SCENARIO ---------- */
    if (receive_length == 2)
    {
        // first of all, set POS (bit 11 in CR1) to 1
        // POS = 1 - ACK bit controls the (N)ACK of the next byte with is received in the shift register
        I2C->CR1 |= (1 << 11);

        // clear the ACK (bit 10 in CR1)
        // to prepare the NACK pulse for the last second byte
        I2C->CR1 &= ~(1 << 10);
    }
    /* ---------- TWO-BYTES SCENARIO ---------- */

    /* ---------- SHARED ADDR CLEARING ---------- */
    // clear the ADDR bit 1 in SR1
    dummy = I2C->SR1;
    dummy = I2C->SR2;

    // ensure we are the Master (MSL, bit 0 = 1) and Receiver (TRA, bit 2 = 0)
    // 0b0001 = 2^0 = 1 = 0x01
    if ((dummy & ((1 << 2) | (1 << 0))) != 0x01)
    {
        uint32_t cr1_snap = I2C->CR1;

        cr1_snap |= (1 << 9);
        cr1_snap |= (1 << 10);
        cr1_snap &= ~(1 << 11);
        // if we enter this if body, the hardware is desynchronized
        // force STOP
        I2C->CR1 = cr1_snap;

        return I2C_ERROR;
    }

    /* ---------- SHARED ADDR CLEARING ---------- */

    /* ---------- ONE-BYTE SCENARIO ---------- */
    if (receive_length == 1)
    {
        // generate the STOP condition to prevent the peripheral from sending unwanted bytes after the one we are asking for
        I2C->CR1 |= (1 << 9);

        // poll the RxNE
        // exits when the byte arrives
        if (I2C_PollHardwareFlags(I2C, I2C_RXNE_MASK))
        {
            return I2C_ERROR;
        }

        // once the byte arrives, read it to our variable
        *pReceive = I2C->DR;
    }

    /* ---------- TWO-BYTES SCENARIO ---------- */
    else if (receive_length == 2)
    {
        // poll BTF - we need to make sure that both bytes arrived: the first one is in the DR, the second is in the shift register
        if (I2C_PollHardwareFlags(I2C, I2C_BTF_MASK))
        {
            return I2C_ERROR;
        }

        // after the BTF is set, issue the STOP condition to prevent fetching of the third byte
        I2C->CR1 |= (1 << 9);

        // read the data
        for (uint8_t i = 0; i < receive_length; i++)
        {
            // poll RxNE
            if (I2C_PollHardwareFlags(I2C, I2C_RXNE_MASK))
            {
                return I2C_ERROR;
            }
            pReceive[i] = I2C->DR;
        }
    }
    /* ---------- TWO-BYTES SCENARIO ---------- */

    /* ---------- MULTI-BYTE SCENARIO ----------*/
    else if (receive_length >= 3)
    {

        // read the data
        for (uint8_t i = 0; i < receive_length; i++)
        {
            // check if the current byte is the second-to-last
            // to clear the ACK bit in CR1
            if (i == receive_length - 2)
            {
                I2C->CR1 &= ~(1 << 10);
            }

            // poll the RxNE
            // exits when the byte arrives
            if (I2C_PollHardwareFlags(I2C, I2C_RXNE_MASK))
            {
                return I2C_ERROR;
            }

            // once the byte arrived, read it to the array
            pReceive[i] = I2C->DR;

            // check if the current byte is the second-to-last
            // to generate the STOP condition in CR1 after the last byte
            if (i == receive_length - 2)
            {
                I2C->CR1 |= (1 << 9);
            }
        }
    }
    /* ---------- MULTI-BYTE SCENARIO ----------*/

    // after both bytes have been read, clear POS
    I2C->CR1 &= ~(1 << 11);

    // after all bytes have been read, re-enable the ACK bit in CR1
    I2C->CR1 |= (1 << 10);

    /* ----- READ PART ----- */

    return I2C_OK;
}