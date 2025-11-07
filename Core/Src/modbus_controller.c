#include "modbus_controller.h"
#include "modbus_io.h"
#include "debug.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static uint8_t modbus_controller_address;

static uint32_t modbus_controller_read_buffer_size;
static uint8_t modbus_controller_read_buffer[MODBUS_IO_BUFFER_SIZE];

static uint32_t modbus_controller_write_buffer_size;
static uint8_t modbus_controller_write_buffer[MODBUS_IO_BUFFER_SIZE];

static uint16_t modbus_holding_registers[MODBUS_CONTROLLER_HOLDING_REGISTERS_SIZE];

static uint8_t coils[256] = {0};
static uint8_t discrete_inputs[256] = {0};
static uint16_t holding_registers[128] = {0};
static uint16_t input_registers[128] = {0};

void modbus_controller_init(void) {
	holding_registers[0] = 0x1234;
	holding_registers[1] = 0x5678;
	input_registers[0] = 0xABCD;
	input_registers[1] = 0xEF01;
	coils[0] = 1;
	coils[10] = 1;
	discrete_inputs[5] = 1;
	modbus_controller_address = 0x42;

	for(uint32_t i = 0; i < MODBUS_CONTROLLER_HOLDING_REGISTERS_SIZE; ++i)
		modbus_holding_registers[i] = i;
}

void process_modbus_message(void);

uint16_t calculate_CRC(uint8_t *data, uint32_t length);

char echo[2048];
void modbus_controller_tick(void) {
	modbus_controller_read_buffer_size = modbus_io_read(modbus_controller_read_buffer);

	if(modbus_controller_read_buffer_size < MODBUS_MIN_MESSAGE_BYTES)
		return;

	for (uint32_t i = 0; i < modbus_controller_read_buffer_size; ++i)
		sprintf(&echo[i * 3], "%02X ", modbus_controller_read_buffer[i]);

	echo[modbus_controller_read_buffer_size * 3] = '\r';
	echo[modbus_controller_read_buffer_size * 3 + 1] = '\n';
	echo[modbus_controller_read_buffer_size * 3 + 2] = '\0';

	debug_write((uint8_t*)echo, strlen(echo));

	if(modbus_controller_read_buffer[MODBUS_ADDRESS_INDEX] != modbus_controller_address)
		return;

	uint16_t crc = (modbus_controller_read_buffer[modbus_controller_read_buffer_size - MODBUS_CRC_BYTES + 1] << 8) |
			   	   (modbus_controller_read_buffer[modbus_controller_read_buffer_size - MODBUS_CRC_BYTES]);

	if(crc != calculate_CRC(modbus_controller_read_buffer, modbus_controller_read_buffer_size - MODBUS_CRC_BYTES))
		return;

	process_modbus_message();
}

void modbus_controller_write(void) {	// Appends CRC before transmitting
	uint16_t crc = calculate_CRC(modbus_controller_write_buffer, modbus_controller_write_buffer_size);

	modbus_controller_write_buffer[modbus_controller_write_buffer_size++] = crc & 0xFF;
	modbus_controller_write_buffer[modbus_controller_write_buffer_size++] = crc >> 8;

	modbus_io_write(modbus_controller_write_buffer, modbus_controller_write_buffer_size);
}

void modbus_controller_exception(uint8_t exception) {	// Sets MSB of function code, appends exception code as data
	modbus_controller_write_buffer[MODBUS_FUNCTION_INDEX] |= 0x80;

	modbus_controller_write_buffer[MODBUS_EXCEPTION_INDEX] = exception;

	modbus_controller_write_buffer_size = MODBUS_MIN_MESSAGE_BYTES - MODBUS_CRC_BYTES + 1;
}

uint16_t calculate_CRC(uint8_t *data, uint32_t length) {
    uint8_t temp;
    uint16_t crc = 0xFFFF;  						// Initialize CRC register

    for (uint32_t i = 0; i < length; ++i)
	{
    	temp = data[i] ^ crc;  						// XOR byte with low byte of CRC
		crc >>= 8;                                  // Shift CRC right 8 bits
		crc ^= crc_table[temp];                     // XOR with table lookup
	}

    return crc;
}

void process_read_holding_registers(void);

void process_write_multiple_registers(void);

void process_modbus_message(void) {
	modbus_controller_write_buffer[MODBUS_ADDRESS_INDEX] = modbus_controller_read_buffer[MODBUS_ADDRESS_INDEX];
	modbus_controller_write_buffer[MODBUS_FUNCTION_INDEX] = modbus_controller_read_buffer[MODBUS_FUNCTION_INDEX];

	modbus_controller_write_buffer_size = MODBUS_MIN_MESSAGE_BYTES - MODBUS_CRC_BYTES;

	switch(modbus_controller_write_buffer[MODBUS_FUNCTION_INDEX]) {
		case MODBUS_READ_HOLDING_REGISTERS: process_read_holding_registers(); break;
		case MODBUS_WRITE_MULTIPLE_REGISTERS: process_write_multiple_registers(); break;
		default: modbus_controller_exception(MODBUS_ILLEGAL_FUNCTION); modbus_controller_write();
	}
}

void process_read_holding_registers() {
	if((modbus_controller_read_buffer_size - MODBUS_CRC_BYTES) < (MODBUS_QUANTITY_OF_REGISTERS_INDEX + 2)) // Not enough fields in message
		return;

	uint16_t starting_address = (modbus_controller_read_buffer[MODBUS_STARTING_ADDRESS_INDEX] << 8) |
								(modbus_controller_read_buffer[MODBUS_STARTING_ADDRESS_INDEX + 1]);

	if(starting_address >= MODBUS_CONTROLLER_HOLDING_REGISTERS_SIZE) {
		modbus_controller_exception(MODBUS_ILLEGAL_DATA_ADDRESS);

		modbus_controller_write();

		return;
	}

	uint16_t quantity_of_registers = (modbus_controller_read_buffer[MODBUS_QUANTITY_OF_REGISTERS_INDEX] << 8) |
									 (modbus_controller_read_buffer[MODBUS_QUANTITY_OF_REGISTERS_INDEX + 1]);

	if(
		(((uint32_t)starting_address) + quantity_of_registers) > MODBUS_CONTROLLER_HOLDING_REGISTERS_SIZE ||
		(((uint32_t)quantity_of_registers) << 1) > (MODBUS_IO_BUFFER_SIZE - MODBUS_MIN_MESSAGE_BYTES - 1) // IO can't send all registers
	) {
		modbus_controller_exception(MODBUS_ILLEGAL_DATA_VALUE);

		modbus_controller_write();

		return;
	}

	modbus_controller_write_buffer[MODBUS_READ_BYTE_COUNT_INDEX] = (quantity_of_registers << 1);
	modbus_controller_write_buffer_size = MODBUS_READ_BYTE_COUNT_INDEX + 1;

	for(uint32_t i = starting_address; i < (starting_address + quantity_of_registers); ++i) {
		modbus_controller_write_buffer[modbus_controller_write_buffer_size++] = modbus_holding_registers[i] >> 8;
		modbus_controller_write_buffer[modbus_controller_write_buffer_size++] = modbus_holding_registers[i] & 0xFF;
	}

	modbus_controller_write();
}

void process_write_multiple_registers() {
	if((modbus_controller_read_buffer_size - MODBUS_CRC_BYTES) < (MODBUS_WRITE_BYTE_COUNT_INDEX + 1)) // Not enough fields in message
		return;

	uint16_t starting_address = (modbus_controller_read_buffer[MODBUS_STARTING_ADDRESS_INDEX] << 8) |
								(modbus_controller_read_buffer[MODBUS_STARTING_ADDRESS_INDEX + 1]);

	if(starting_address >= MODBUS_CONTROLLER_HOLDING_REGISTERS_SIZE) {
		modbus_controller_exception(MODBUS_ILLEGAL_DATA_ADDRESS);

		modbus_controller_write();

		return;
	}

	uint16_t quantity_of_registers = (modbus_controller_read_buffer[MODBUS_QUANTITY_OF_REGISTERS_INDEX] << 8) |
									 (modbus_controller_read_buffer[MODBUS_QUANTITY_OF_REGISTERS_INDEX + 1]);

	if((((uint32_t)starting_address) + quantity_of_registers) > MODBUS_CONTROLLER_HOLDING_REGISTERS_SIZE) {
		modbus_controller_exception(MODBUS_ILLEGAL_DATA_VALUE);

		modbus_controller_write();

		return;
	}

	uint8_t byte_count = modbus_controller_read_buffer[MODBUS_WRITE_BYTE_COUNT_INDEX];

	if(
		(byte_count >> 1) != quantity_of_registers ||
		(modbus_controller_read_buffer_size - MODBUS_CRC_BYTES - MODBUS_WRITE_BYTE_COUNT_INDEX - 1) < byte_count
	) {
		modbus_controller_exception(MODBUS_ILLEGAL_DATA_VALUE);

		modbus_controller_write();

		return;
	}

	uint32_t data_index = MODBUS_WRITE_BYTE_COUNT_INDEX + 1;
	for(uint32_t i = starting_address; i < (starting_address + quantity_of_registers); ++i) {
		modbus_holding_registers[i] = modbus_controller_read_buffer[data_index++] << 8;
		modbus_holding_registers[i] |= modbus_controller_read_buffer[data_index++];
	}

	modbus_controller_write_buffer[MODBUS_STARTING_ADDRESS_INDEX] = modbus_controller_read_buffer[MODBUS_STARTING_ADDRESS_INDEX];
	modbus_controller_write_buffer[MODBUS_STARTING_ADDRESS_INDEX + 1] = modbus_controller_read_buffer[MODBUS_STARTING_ADDRESS_INDEX + 1];
	modbus_controller_write_buffer[MODBUS_QUANTITY_OF_REGISTERS_INDEX] = modbus_controller_read_buffer[MODBUS_QUANTITY_OF_REGISTERS_INDEX];
	modbus_controller_write_buffer[MODBUS_QUANTITY_OF_REGISTERS_INDEX + 1] = modbus_controller_read_buffer[MODBUS_QUANTITY_OF_REGISTERS_INDEX + 1];

	modbus_controller_write_buffer_size = MODBUS_QUANTITY_OF_REGISTERS_INDEX + 2;

	modbus_controller_write();
}

void process_modbus_message(){
	 uint8_t function_code = modbus_controller_buffer[MODBUS_FUNCTION_INDEX];

	    switch(function_code) {
	        case MODBUS_READ_COILS:
	            handle_read_coils();
	            break;

	        case MODBUS_READ_DISCRETE_INPUTS:
	            handle_read_discrete_inputs();
	            break;

	        case MODBUS_READ_HOLDING_REGISTERS:
	            handle_read_holding_registers();
	            break;

	        case MODBUS_READ_INPUT_REGISTERS:
	            handle_read_input_registers();
	            break;

	        case MODBUS_WRITE_SINGLE_COIL:
	            handle_write_single_coil();
	            break;

	        case MODBUS_WRITE_SINGLE_REGISTER:
	            handle_write_single_register();
	            break;

	        case MODBUS_DIAGNOSTICS:
	            handle_diagnostics();
	            break;

	        case MODBUS_GET_COMM_EVENT_COUNTER:
	            handle_get_comm_event_counter();
	            break;

	        case MODBUS_WRITE_MULTPLE_COILS:
	            handle_write_multiple_coils();
	            break;

	        case MODBUS_WRITE_MULTIPLE_REGISTERS:
	            handle_write_multiple_registers();
	            break;

	        case MODBUS_REPORT_SERVER_ID:
	            handle_report_server_id();
	            break;

	        case MODBUS_MASK_WRITE_REGISTER:
	            handle_mask_write_register();
	            break;

	        case MODBUS_READ_WRITE_MULTPLE_REGISTERS:
	            handle_read_write_multiple_registers();
	            break;

	        case MODBUS_READ_DEVICE_IDENTIFICATION_1:
	        case MODBUS_READ_DEVICE_IDENTIFICATION_2:
	            handle_read_device_identification();
	            break;

	        default:
	            // Unsupported function - ignore or send exception
	            break;
	    }
}


