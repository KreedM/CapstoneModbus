#include "debug.h"

static volatile uint32_t debug_write_head = 0, debug_write_tail = 0;
static volatile uint8_t  debug_write_buffer[DEBUG_WRITE_SIZE];

static volatile uint32_t debug_read_head = 0, debug_read_tail = 0;
static volatile uint8_t  debug_read_buffer[DEBUG_READ_SIZE];

static volatile uint32_t line_feeds = 0;

static volatile UART_HandleTypeDef* debug_huart;
void debug_init(UART_HandleTypeDef* _debug_huart) {
	debug_huart = _debug_huart;

	__HAL_UART_DISABLE_IT(debug_huart, UART_IT_TXE);
	__HAL_UART_ENABLE_IT(debug_huart, UART_IT_RXNE);
}

static uint32_t count, next_debug_write_head;
uint32_t debug_write(uint8_t *data, uint32_t len) {
    if((len == 0) || (len >= DEBUG_WRITE_SIZE))
        return 0;

    if(debug_write_head == DEBUG_WRITE_SIZE - 1)
        next_debug_write_head = 0;
    else
        next_debug_write_head = debug_write_head + 1;

    if(next_debug_write_head == debug_write_tail)
        return 0;

    count = 0;

    do {
        debug_write_buffer[debug_write_head] = data[count];

        debug_write_head = next_debug_write_head;

        if(next_debug_write_head == DEBUG_WRITE_SIZE - 1)
            next_debug_write_head = 0;
        else
            ++next_debug_write_head;

        ++count;
    } while((count < len) && (next_debug_write_head != debug_write_tail));

    __HAL_UART_ENABLE_IT(debug_huart, UART_IT_TXE);

    return count;
}

void debug_tx_e_handler(void) {
	while(__HAL_UART_GET_FLAG(debug_huart, UART_FLAG_TXE)) { // If TXFIFO is enabled, flag indicates FIFO isn't full
    	if(debug_write_head == debug_write_tail) {
    		__HAL_UART_DISABLE_IT(debug_huart, UART_IT_TXE);

    		break;
    	}

    	debug_huart->Instance->TDR = debug_write_buffer[debug_write_tail];

        if(debug_write_tail == DEBUG_WRITE_SIZE - 1)
            debug_write_tail = 0;
        else
            ++debug_write_tail;
    }
}

uint32_t debug_read(uint8_t *buffer) {
    if(line_feeds == 0)
        return 0;

    count = 0;

    while(debug_read_head != debug_read_tail) {
        *buffer = debug_read_buffer[debug_read_tail];

        if(debug_read_tail == DEBUG_READ_SIZE - 1)
            debug_read_tail = 0;
        else
            ++debug_read_tail;

        ++count;

        if(*buffer == '\n') {
        	__HAL_UART_ENABLE_IT(debug_huart, UART_IT_RXNE);

            break;
        }

        ++buffer;
    }

    --line_feeds;

    return count;
}

static volatile uint32_t next_debug_read_head;
static volatile uint8_t  character;
void debug_rx_ne_handler(void) {
    if(debug_read_head == DEBUG_READ_SIZE - 1)
        next_debug_read_head = 0;
    else
        next_debug_read_head = debug_read_head + 1;

    while(__HAL_UART_GET_FLAG(debug_huart, UART_FLAG_RXNE)) {
    	if(next_debug_read_head == debug_read_tail) {
    		__HAL_UART_DISABLE_IT(debug_huart, UART_IT_RXNE);

    		break;
    	}

        if(
			__HAL_UART_GET_FLAG(debug_huart, UART_FLAG_PE) ||
			__HAL_UART_GET_FLAG(debug_huart, UART_FLAG_FE) ||
			__HAL_UART_GET_FLAG(debug_huart, UART_FLAG_NE)
		) {
        	__HAL_UART_CLEAR_FLAG(debug_huart, UART_FLAG_PE);
        	__HAL_UART_CLEAR_FLAG(debug_huart, UART_FLAG_FE);
        	__HAL_UART_CLEAR_FLAG(debug_huart, UART_FLAG_NE);

            debug_huart->Instance->RDR; // Read and clear anomalous byte
        }
        else {
            character = debug_huart->Instance->RDR;
            if(character == '\n')
                ++line_feeds;

            debug_read_buffer[debug_read_head] = character;

            debug_read_head = next_debug_read_head;

            if(next_debug_read_head == DEBUG_READ_SIZE - 1)
                next_debug_read_head = 0;
            else
                ++next_debug_read_head;
        }
    }

    if(__HAL_UART_GET_FLAG(debug_huart, UART_FLAG_ORE))
        __HAL_UART_CLEAR_FLAG(debug_huart, UART_FLAG_ORE);
}
