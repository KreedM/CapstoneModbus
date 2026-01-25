#ifndef DEBUG_H
#define	DEBUG_H

#include <stdint.h>
#include "stm32c0xx.h"

#define DEBUG_WRITE_SIZE 2048 						// Actual characters allowed is -1
#define DEBUG_READ_SIZE  2048

void debug_init(UART_HandleTypeDef* _debug_huart); 	// Doesn't initialize peripheral or GPIOs. Make sure the pointed to handle doesn't go out of scope!

uint32_t debug_write(uint8_t *data, uint32_t len);

uint32_t debug_read(uint8_t *buffer); 				// Reads substrings separated by LF

void debug_handler(void);							// Call this in USARTx_IRQHandler()

#endif
