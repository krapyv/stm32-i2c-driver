# STM32F411 I2C Interrupt-Driven Non-blocking Driver

An I2C driver that supports three types of transaction: transmit, receive and transmit-receive, using EV and ER interrupt handlers driven by SB, ADDR, TXE, RXNE, BTF, BERR, ARLO and AF flags in non-blocking way. Error recovery isolates each fault - BERR, ARLO, AF - and triggers a Software Reset (SWRST) to return the peripheral to a clean register state.
This documents the reusable driver layer only. Application-layer usage with BMP280 is covered in a separate project README.

## Project Structure

```text
reusable_drivers/
├── core/   # ARM Cortex-M4 and STM32F411 register definitions
    ├── core_cm4.h      # Register layout definitions for NVIC and SysTick architectures
    └── stm32f411.h     # Memory boundaries and register definitions for AHB/APB peripherals
└── periph/     # Portable peripheral drivers
    ├── i2c.c      # Register-level I2C peripheral driver: EV/ER ISR logic, transaction state machine, and SWRST recovery
    └── i2c.h      # I2C control and state enums, handler struct declaration, extern variable declaration, and function headers
```

## Public API

### I2C_Master_Transmit
Queues a transmit transaction and returns immediately.
Completion must be polled via I2C_Process() or checked against I2C_STATE_DONE.

I2C_Process() must be called from the main loop to handle state transitions and
retry logic.

Parameters:
- slave_addr    7-bit device address (driver shifts left internally)
- data          Pointer to transmit buffer. Must remain valid until STATE_DONE
- length        Number of bytes to transmit

Returns: I2C_OK if transaction was queued, I2C_ERROR if bus is busy or the driver is in a fault state. Check hi2c.state to distinguish.

### I2C_Master_Receive
Queues a receive transaction and returns immediately.
Completion must be polled via I2C_Process() or checked against I2C_STATE_DONE.

I2C_Process() must be called from the main loop to handle state transitions and
retry logic.

Parameters:
- slave_addr    7-bit device address (driver shifts left internally)
- data          Pointer to receive buffer. Must remain valid until STATE_DONE
- length        Number of bytes to receive

Returns: I2C_OK if transaction was queued, I2C_ERROR if bus is busy or the drive
r is in a fault state. Check hi2c.state to distinguish.

### I2C_Master_Transmit_Receive
Queues a transmit-receive transaction and returns immediately.
Completion must be polled via I2C_Process() or checked against I2C_STATE_DONE.

I2C_Process() must be called from the main loop to handle state transitions and retry logic.

Parameters:
- slave_addr        7-bit device address (driver shifts left internally)
- pSend             Pointer to transmit buffer. Must remain valid until STATE_DONE
- pReceive          Pointer to receive buffer. Must remain valid until STATE_DONE
- send_length       Number of bytes to transmit
- receive_length    Number of bytes to receive

Returns: I2C_OK if transaction was queued, I2C_ERROR if bus is busy or the drive
r is in a fault state. Check hi2c.state to distinguish.

## Low-level Architectural Decisions

### State machine
The state machine has 7 different states:
1. I2C_STATE_IDLE:
The default state of the I2C.

2. I2C_STATE_ERROR:
The I2C bus experienced an error (BERR, ARLO or AF - can be distinguished by looking at the `error_code` value).

BERR error requires the SWRST recovery sequence that handled by I2C_Process().
For the sake of the simplicity, AF and ARLO errors are handled in I2C_Process by changing hi2c.state to I2C_STATE_IDLE and hi2c.error_code to I2C_ERROR_NONE.

3. I2C_STATE_START_PENDING:
The state represents "a START condition was issued but has not been detected on the bus yet". 
Is set along the both normal and repeated START generations (CR1 START bit (bit 8)). 

4. I2C_STATE_TX_ADDR:
The state represents the hardware reality after the START was detected on the bus and slave's address was sent over the lines with W last bit.
Used in ADDR state machine's branch to handle the Transmit ADDR bit interrupt.

5. I2C_STATE_RX_ADDR:
The state represents the hardware reality after the START was detected on the bus and slave's address was sent over the lines with R last bit.
Used in ADDR state machine's branch to handle the Receive ADDR bit interrupt.

6. I2C_STATE_FINISHING:
The state represents the period between the CR1 STOP bit (bit 8) setting and the moment the STOP is detected on the lines. 
Handled by I2C_PollStopConfirmation(), that uses the polling mechanism - SR2 BUSY bit (bit 1) and 2ms SysTick timeout, after the call from  I2C_Process(). 
The I2C_PollStopConfirmation, in case of timeout hit, sets I2C_ERROR_BERR as a software sentinel, not a hardware flag. Then the recovery path in I2C_Process() will trigger a full SWRST sequence - the same path as a real hardware bus error.

7. I2C_STATE_DONE:
The state used to show the transaction finished successfully, the bus is free and the peripheral is ready for the next transaction.

### N=1/2/3 receive paths

ADDR interrupt:

N=1:
The ACK should be cleared before the ADDR clear, since the first and simultaneously the last byte is going to be clocking in immediately after the ADDR clearing. If ACK would be cleared after the ADDR, then the byte is going to be ACKed and the peripheral would send the second unwanted byte as well.
The CR2 ITBUFEN bit (bit 10) is enabled as the last since the incoming byte needs to trigger RXNE interrupt.

N=2:
The POS should be set and the ACK should be cleared before ADDR clear.
CR1 POS bit (bit 11) is used to make the ACK bit to control the (N)ACK of the next byte which is received in the shift register. Since the ACK is cleared along with the POS setting, the second (the last) byte is going to be NACKed.
Without the POS = 1, the ACK = 0 would NACK the first byte of data.

N=3:
Just clear ADDR (read of SR1 and SR2) and enable ITBUFEN.

BTF interrupt:

N=1: no BTF is possible since the BTF requires both DR and shift register to be full simultaneously. With only one byte requested, the shift register never fills - BTF cannot fire.

N=2: 

The CR1 STOP bit (bit 9) is set, since both wanted bytes are received (the first one is in the DR, the second one is in the Shift Register).
Read both bytes (after the byte 2 is read, the byte 1 immediately drops to the Shift Register).
Clear CR1 POS bit (bit 11) and re-enable the CR1 ACK bit (bit 10) for the next transaction.

N=3:
Has three branches:
1. index = RxLength - 3:

The second to last data byte handling. 
Byte N - 2 in DR, byte N - 1 in Shift register.

The ACK bit is cleared to NACK the byte N-1, the byte N-2 is read from the DR. The byte N-1 moves immediately to the DR.

2. index = RxLength - 2:
The last byte handling.

Byte N-1 in DR, byte N in shift register.

Set the CR1 STOP bit (bit 9), since all the bytes have been received.
Consequently read the bytes N-1 and N. Re-enable the ACK bit in CR1. Set the hi2c.state to I2C_STATE_FINISHING.

3. Fallback:
The first byte is in DR, the second byte is in the shift register, but they are not tail-end indexes (the firmware is one step behind the hardware).

Read the byte from the DR, to clear BTF and let the state machine to fall to simple RXNE.

RXNE interrupt:

The N=2 does not use RXNE since its ADDR branch does not enable CR2 ITBUFEN bit (bit 10).

N=1:
The only requested byte is in the DR. Read it and re-enable ACK.

N=3: 
Used only for bytes before the tail-end (index < RxLength - 2).

Read the DR byte. If it is the last byte before the tail-end ones (index = RxLength - 3) - disable ITBUFEN, so the last two bytes are BTF-only.

## Known Limitations

### AF and ARLO errors handling:

At the moment, the AF and ARLO errors are handled in I2C_Process() just by setting hi2c.state to STATE_IDLE and clearing hi2c.error_code to ERROR_NONE.

There is a clear way to make it better: to add SysTick timeout because:
1. The ARLO error is caused by the master losing the bus control during the transaction, so we have to wait some time and check whether the bus is free, not immediately try to seize control over the bus and cause a new collision.
2. The AF error is caused by the peripheral not acknowledging the address, so we need to wait some time and try again (if we assume the slave could have been not responsive).

Both errors should use SWRST recovery path (the same as the BERR) if waiting is not helpful.

### The BTF Repeated Start race

The repeated-start implementation in this driver has an inherent window between write-phase BTF handling and genuine SB confirmation where the ISR guard must correctly reject reentry via state rather than phase.
Since the BTF flag is cleared by hardware once START or STOP condition is detected on the bus and in I2C_TX_RX case we are not explicitly clearing the BTF, but just issuing the REPEATED START after the ISR exit, the BTF flag is still set.
Since the BTF is still set when the ISR exits, the pending interrupt flag in NVIC has not been cleared, causing immediate re-entry.
The ISR immediately re-enters itself and hits the BTF once again for the same stale BTF flag.
The EV ISR guard checks hi2c.state == I2C_STATE_START_PENDING at entry and discards all re-entries while the repeated START is pending on the bus (observed in GDB: races on approximately 9 of every 19 repeated-start cycles).
The I2C_Master_* functions that issue START have I2C_PollHardwareBusy that takes up to 4 ms of time, so there are almost no races at all.
The ISR issuing START has no time-buffer at all.
The issue is just frequent, but cosmetic - no side effects if the guard is present.

hi2c.start_pending_hits is a diagnostic counter that accumulates stale re-entries, allowing the race frequency to be observed at runtime without affecting transaction correctness.

## Next Steps & Learning Roadmap

While the basic Interrupt-driven architecture is already done, there are several improvements that could be made in the future:

1. DMA-driven architecture:

Now the CPU is involved only during interrupts. With DMA, the CPU is removed from the data path entirely.

2. Proper AF/ARLO recovery with SysTick timeout:

The AF/ARLO recovery currently resets to IDLE without bus-free verification on retry logic.

3. SWRST path for STOP confirmation timeout: 

Not critical for the test usage, but essential for the production-grade driver.
