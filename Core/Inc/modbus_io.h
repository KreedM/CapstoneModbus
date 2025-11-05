#ifndef MODBUS_IO_H
#define MODBUS_IO_H

#include <stdint.h>
#include "stm32u5xx_hal.h"

#define MODBUS_IO_BUFFER_SIZE 256

// Enables UART RXNE interrupt and disables UART TXE. Prescales timer to match baud rate, sets it to one pulse mode and configures CC1 & CC2 to character wait times
void modbus_io_init(
	UART_HandleTypeDef* _modbus_io_huart, uint32_t modbus_io_huart_freq, 	// UART kernel clock frequency, without prescaler
	TIM_HandleTypeDef* _modbus_io_htim, uint32_t modbus_io_tim_freq		 	// Timer clock frequency
);

// Make sure buffers are at most/least size MODBUS_IO_BUFFER_SIZE!
uint32_t modbus_io_write(uint8_t *data, uint32_t len); 	// Returns number of bytes that'll be transmitted

uint32_t modbus_io_read(uint8_t *buffer);				// Returns number of bytes that are read into buffer

// Handlers clear any requisite flags
void modbus_io_tc_handler(void);

void modbus_io_rx_ne_handler(void);

void modbus_io_1_5_char_handler(void);

void modbus_io_3_5_char_handler(void);

#endif
