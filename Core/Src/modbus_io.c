#include "modbus_io.h"
#include <stdbool.h>
#include <string.h>

static volatile UART_HandleTypeDef* modbus_io_huart;
static volatile TIM_HandleTypeDef*  modbus_io_htim;

static volatile bool frame_new = true, frame_end = true;

static volatile uint32_t modbus_io_transmit_head = 0, modbus_io_transmit_size = 0;
static volatile uint8_t modbus_io_transmit_buffer[MODBUS_IO_BUFFER_SIZE];

static volatile uint32_t modbus_io_receive_size = 0;
static volatile uint8_t modbus_io_receive_buffer[MODBUS_IO_BUFFER_SIZE];

static volatile uint32_t modbus_io_read_size = 0;
static volatile uint8_t modbus_io_read_buffer[MODBUS_IO_BUFFER_SIZE];

void modbus_io_init(UART_HandleTypeDef* _modbus_io_huart, uint32_t modbus_io_huart_freq, TIM_HandleTypeDef* _modbus_io_htim, uint32_t modbus_io_tim_freq) {
	modbus_io_huart = _modbus_io_huart;

	switch(modbus_io_huart->Init.ClockPrescaler) {
		case UART_PRESCALER_DIV1: 	modbus_io_huart_freq /= 1; break;
		case UART_PRESCALER_DIV2: 	modbus_io_huart_freq /= 2; break;
		case UART_PRESCALER_DIV4: 	modbus_io_huart_freq /= 4; break;
		case UART_PRESCALER_DIV6: 	modbus_io_huart_freq /= 6; break;
		case UART_PRESCALER_DIV8: 	modbus_io_huart_freq /= 8; break;
		case UART_PRESCALER_DIV10: 	modbus_io_huart_freq /= 10; break;
		case UART_PRESCALER_DIV12: 	modbus_io_huart_freq /= 12; break;
		case UART_PRESCALER_DIV16: 	modbus_io_huart_freq /= 16; break;
		case UART_PRESCALER_DIV32: 	modbus_io_huart_freq /= 32; break;
		case UART_PRESCALER_DIV64: 	modbus_io_huart_freq /= 64; break;
		case UART_PRESCALER_DIV128: modbus_io_huart_freq /= 128; break;
		case UART_PRESCALER_DIV256:	modbus_io_huart_freq /= 256; break;
	}

	__HAL_UART_DISABLE_IT(_modbus_io_huart, UART_IT_TC);
	__HAL_UART_ENABLE_IT(_modbus_io_huart, UART_IT_RXNE);

	modbus_io_htim = _modbus_io_htim;

	modbus_io_htim->Instance->PSC = ((float)modbus_io_tim_freq / modbus_io_huart_freq) * modbus_io_huart->Instance->BRR;

	float bits_per_frame = 1 + (modbus_io_huart->Init.Parity > 0);
	switch(modbus_io_huart->Init.WordLength) {
		case UART_WORDLENGTH_7B: bits_per_frame += 7; break;
		case UART_WORDLENGTH_8B: bits_per_frame += 8; break;
		case UART_WORDLENGTH_9B: bits_per_frame += 9; break;
	}
	switch(modbus_io_huart->Init.StopBits) {
		case UART_STOPBITS_0_5: bits_per_frame += 0.5f; break;
		case UART_STOPBITS_1: 	bits_per_frame += 1; break;
		case UART_STOPBITS_1_5: bits_per_frame += 1.5f; break;
		case UART_STOPBITS_2:	bits_per_frame += 2; break;
	}

	modbus_io_htim->Instance->CCR1 = bits_per_frame * 3/2;	// 1.5 character times
	modbus_io_htim->Instance->CCR2 = bits_per_frame * 7/2;	// 3.5 character times
	modbus_io_htim->Instance->ARR = bits_per_frame * 7/2; 	// Just needs to be >= CCR2, stops earlier too

	__HAL_TIM_ENABLE_IT(_modbus_io_htim, TIM_IT_CC1);
	__HAL_TIM_ENABLE_IT(_modbus_io_htim, TIM_IT_CC2);

	modbus_io_htim->Instance->EGR |= TIM_EGR_UG; // Update registers to configured values
	modbus_io_htim->Instance->SR &= ~TIM_SR_UIF; // Necessary after update generation
}

void restart_timer(void) {
	modbus_io_htim->Instance->CR1 &= ~TIM_CR1_CEN;

	frame_new = false;
	frame_end = false;

	modbus_io_htim->Instance->CNT = 0;
	modbus_io_htim->Instance->CR1 |= TIM_CR1_CEN;
}

uint32_t modbus_io_write(uint8_t *data, uint32_t len) {
	if(len == 0)
		return 0;
	else if(len >= MODBUS_IO_BUFFER_SIZE)
		len = MODBUS_IO_BUFFER_SIZE;

	memcpy((void*)modbus_io_transmit_buffer, (void*)data, len);

	modbus_io_transmit_head = 0;
	modbus_io_transmit_size = len;

	if(frame_end)
		__HAL_UART_ENABLE_IT(modbus_io_huart, UART_IT_TC);

	return len;
}

void modbus_io_tc_handler(void) {
	restart_timer();

	modbus_io_huart->Instance->TDR = modbus_io_transmit_buffer[modbus_io_transmit_head++];

	if(modbus_io_transmit_head == modbus_io_transmit_size) {
		modbus_io_transmit_size = 0;
		modbus_io_transmit_head = 0;

		__HAL_UART_DISABLE_IT(modbus_io_huart, UART_IT_TC);
	}
}

uint32_t modbus_io_read(uint8_t *buffer) {
	if(modbus_io_read_size == 0)
		return 0;

	memcpy((void*)buffer, (void*)modbus_io_read_buffer, modbus_io_read_size);

	uint32_t count = modbus_io_read_size;

	modbus_io_read_size = 0;

	return count;
}

void modbus_io_rx_ne_handler(void) {
	if(frame_new)
		modbus_io_receive_size = 0;

	restart_timer();

	uint8_t rdr = modbus_io_huart->Instance->RDR;
	if(modbus_io_receive_size < MODBUS_IO_BUFFER_SIZE)
		modbus_io_receive_buffer[modbus_io_receive_size++] = rdr;

	if(
		__HAL_UART_GET_FLAG(modbus_io_huart, UART_FLAG_PE) ||
		__HAL_UART_GET_FLAG(modbus_io_huart, UART_FLAG_FE) ||
		__HAL_UART_GET_FLAG(modbus_io_huart, UART_FLAG_NE)
	) {
		__HAL_UART_CLEAR_FLAG(modbus_io_huart, UART_FLAG_PE);
		__HAL_UART_CLEAR_FLAG(modbus_io_huart, UART_FLAG_FE);
		__HAL_UART_CLEAR_FLAG(modbus_io_huart, UART_FLAG_NE);
	}

    if(__HAL_UART_GET_FLAG(modbus_io_huart, UART_FLAG_ORE))
        __HAL_UART_CLEAR_FLAG(modbus_io_huart, UART_FLAG_ORE);
}

void modbus_io_1_5_char_handler(void) {
	frame_new = true;

	__HAL_TIM_CLEAR_FLAG(modbus_io_htim, TIM_FLAG_CC1);
}

void modbus_io_3_5_char_handler(void) {
	frame_end = true;

	if(modbus_io_transmit_size > 0)							// Shouldn't ever happen since device should wait for frame end, process message, then reply
		__HAL_UART_ENABLE_IT(modbus_io_huart, UART_IT_TC);

	if(modbus_io_receive_size > 0) {						// Separate read buffer is used just in case application takes a while to read, avoids race conditions
		memcpy((void*)modbus_io_read_buffer, (void*)modbus_io_receive_buffer, modbus_io_receive_size);

		modbus_io_read_size = modbus_io_receive_size;

		modbus_io_receive_size = 0;
	}

	__HAL_TIM_CLEAR_FLAG(modbus_io_htim, TIM_FLAG_CC2);
}
