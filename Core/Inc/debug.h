#ifndef DEBUG_H
#define	DEBUG_H

#include <stdint.h>
#include "stm32u5xx_hal.h"

#define DEBUG_WRITE_SIZE 2048 						// Actual characters allowed is -1
#define DEBUG_READ_SIZE  2048

void debug_init(UART_HandleTypeDef* _debug_huart); 	// Doesn't initialize peripheral or GPIOs. Make sure the pointed to handle doesn't go out of scope!

// Implement these in USARTx_IRQHandler()
void debug_tx_e_handler(void);
uint32_t debug_write(uint8_t *data, uint32_t len);

void debug_rx_ne_handler(void);
uint32_t debug_read(uint8_t *buffer); 				// Reads substrings separated by LF

#endif

