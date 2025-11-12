#include "modbus_controller.h"
#include "modbus_io.h"
#include "debug.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static uint8_t m_c_address;

static uint32_t m_c_read_buffer_size;
static uint8_t m_c_read_buffer[MODBUS_IO_BUFFER_SIZE];

static uint32_t m_c_write_buffer_size;
static uint8_t m_c_write_buffer[MODBUS_IO_BUFFER_SIZE];

static uint8_t m_c_coils[MODBUS_CONTROLLER_COILS_BYTE_SIZE];
static uint8_t m_c_discrete_inputs[MODBUS_CONTROLLER_DISCRETE_INPUTS_BYTE_SIZE];
static uint16_t m_c_holding_registers[MODBUS_CONTROLLER_HOLDING_REGISTERS_SIZE];
static uint16_t m_c_input_registers[MODBUS_CONTROLLER_INPUT_REGISTERS_SIZE];

void modbus_controller_init(void) {
	m_c_address = 0x42;

	for(uint32_t i = 0; i < MODBUS_CONTROLLER_HOLDING_REGISTERS_SIZE; ++i)
		m_c_holding_registers[i] = i;

	for(uint32_t i = 0; i < MODBUS_CONTROLLER_INPUT_REGISTERS_SIZE; ++i)
			m_c_input_registers[i] = i;
}

void process_modbus_message(void);

uint16_t calculate_CRC(uint8_t *data, uint32_t length);

char echo[2048];
void modbus_controller_tick(void) {
	m_c_read_buffer_size = modbus_io_read(m_c_read_buffer);

	if(m_c_read_buffer_size < MODBUS_MIN_MESSAGE_BYTES)
		return;

	for (uint32_t i = 0; i < m_c_read_buffer_size; ++i)
		sprintf(&echo[i * 3], "%02X ", m_c_read_buffer[i]);

	echo[m_c_read_buffer_size * 3] = '\r';
	echo[m_c_read_buffer_size * 3 + 1] = '\n';
	echo[m_c_read_buffer_size * 3 + 2] = '\0';

	debug_write((uint8_t*)echo, strlen(echo));

	if(m_c_read_buffer[MODBUS_ADDRESS_INDEX] != m_c_address)
		return;

	uint16_t crc = (m_c_read_buffer[m_c_read_buffer_size - MODBUS_CRC_BYTES + 1] << 8) |
			   	   (m_c_read_buffer[m_c_read_buffer_size - MODBUS_CRC_BYTES]);

	if(crc != calculate_CRC(m_c_read_buffer, m_c_read_buffer_size - MODBUS_CRC_BYTES))
		return;

	process_modbus_message();
}

void modbus_controller_write(void) {	// Appends CRC before transmitting
	uint16_t crc = calculate_CRC(m_c_write_buffer, m_c_write_buffer_size);

	m_c_write_buffer[m_c_write_buffer_size++] = crc & 0xFF;
	m_c_write_buffer[m_c_write_buffer_size++] = crc >> 8;

	modbus_io_write(m_c_write_buffer, m_c_write_buffer_size);
}

void modbus_controller_exception(uint8_t exception) {	// Sets MSB of function code, appends exception code as data
	m_c_write_buffer[MODBUS_FUNCTION_INDEX] |= 0x80;

	m_c_write_buffer[MODBUS_EXCEPTION_INDEX] = exception;

	m_c_write_buffer_size = MODBUS_MIN_MESSAGE_BYTES - MODBUS_CRC_BYTES + 1;
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

void process_read_coils(void);
void process_read_discrete_inputs(void);
void process_read_holding_registers(void);
void process_read_input_registers(void);
void process_write_single_coil(void);
void process_write_single_register(void);
void process_write_multiple_coils(void);
void process_write_multiple_registers(void);

void process_modbus_message(void) {	// Appends device address and function to m_c_write_buffer, then processes function
	m_c_write_buffer[MODBUS_ADDRESS_INDEX] = m_c_read_buffer[MODBUS_ADDRESS_INDEX];
	m_c_write_buffer[MODBUS_FUNCTION_INDEX] = m_c_read_buffer[MODBUS_FUNCTION_INDEX];

	m_c_write_buffer_size = MODBUS_MIN_MESSAGE_BYTES - MODBUS_CRC_BYTES;

	switch(m_c_write_buffer[MODBUS_FUNCTION_INDEX]) {
		case MODBUS_READ_COILS:
			process_read_coils();
			break;
		case MODBUS_READ_DISCRETE_INPUTS:
			process_read_discrete_inputs();
			break;
		case MODBUS_READ_HOLDING_REGISTERS:
			process_read_holding_registers();
			break;
		case MODBUS_READ_INPUT_REGISTERS:
			process_read_input_registers();
			break;
		case MODBUS_WRITE_SINGLE_COIL:
			process_write_single_coil();
			break;
		case MODBUS_WRITE_SINGLE_REGISTER:
			process_write_single_register();
			break;
		case MODBUS_WRITE_MULTPLE_COILS:
			process_write_multiple_coils();
			break;
		case MODBUS_WRITE_MULTIPLE_REGISTERS:
			process_write_multiple_registers();
			break;
		default:
			modbus_controller_exception(MODBUS_ILLEGAL_FUNCTION);
			modbus_controller_write();
	}
}

// Function 0x01: Read Coils
void process_read_coils(void) {
	if((m_c_read_buffer_size - MODBUS_CRC_BYTES) < (MODBUS_QUANTITY_OF_COILS_INDEX + 2))	// +1 to include Lo portion of QoC, +1 for count up to and including index
		return;

	uint16_t starting_address = (m_c_read_buffer[MODBUS_STARTING_ADDRESS_INDEX] << 8) |
								(m_c_read_buffer[MODBUS_STARTING_ADDRESS_INDEX + 1]);

	if(starting_address >= (MODBUS_CONTROLLER_COILS_BYTE_SIZE << 3)) {
		modbus_controller_exception(MODBUS_ILLEGAL_DATA_ADDRESS);
		modbus_controller_write();
		return;
	}

	uint16_t quantity_of_coils = (m_c_read_buffer[MODBUS_QUANTITY_OF_COILS_INDEX] << 8) |
								 (m_c_read_buffer[MODBUS_QUANTITY_OF_COILS_INDEX + 1]);

	if(
		((((uint32_t)starting_address) + quantity_of_coils) > (MODBUS_CONTROLLER_COILS_BYTE_SIZE << 3)) ||
		(((((uint32_t)quantity_of_coils) + 7) >> 3) > 0xFF) ||												// Can only send back 255 bytes max
		(quantity_of_coils == 0)
	) {
		modbus_controller_exception(MODBUS_ILLEGAL_DATA_VALUE);
		modbus_controller_write();
		return;
	}

	uint8_t byte_count = (((uint32_t)quantity_of_coils) + 7) >> 3;

	m_c_write_buffer[MODBUS_READ_BYTE_COUNT_INDEX] = byte_count;
	m_c_write_buffer_size = MODBUS_READ_BYTE_COUNT_INDEX + 1;

	uint32_t coil_address = starting_address;
	for(uint8_t i = 0; i < byte_count; ++i) {
		uint8_t byte_value = 0;

		for(uint8_t bit = 0; bit < 8; ++bit) {
			if(m_c_coils[coil_address >> 3] & (1 << (coil_address & 0x07)))	// Get lower 3 bits for bit position
				byte_value |= (1 << bit);

			++coil_address;

			if((coil_address - starting_address) >= quantity_of_coils)
				break;
		}

		m_c_write_buffer[m_c_write_buffer_size++] = byte_value;
	}

	modbus_controller_write();
}

// Function 0x02: Read Discrete Inputs
void process_read_discrete_inputs(void) {	// Functionally same as process_read_coils
	if((m_c_read_buffer_size - MODBUS_CRC_BYTES) < (MODBUS_QUANTITY_OF_INPUTS_INDEX + 2))
		return;

	uint16_t starting_address = (m_c_read_buffer[MODBUS_STARTING_ADDRESS_INDEX] << 8) |
								(m_c_read_buffer[MODBUS_STARTING_ADDRESS_INDEX + 1]);

	if(starting_address >= (MODBUS_CONTROLLER_DISCRETE_INPUTS_BYTE_SIZE << 3)) {
		modbus_controller_exception(MODBUS_ILLEGAL_DATA_ADDRESS);
		modbus_controller_write();
		return;
	}

	uint16_t quantity_of_coils = (m_c_read_buffer[MODBUS_QUANTITY_OF_INPUTS_INDEX] << 8) |
								 (m_c_read_buffer[MODBUS_QUANTITY_OF_INPUTS_INDEX + 1]);

	if(
		((((uint32_t)starting_address) + quantity_of_coils) > (MODBUS_CONTROLLER_DISCRETE_INPUTS_BYTE_SIZE << 3)) ||
		(((((uint32_t)quantity_of_coils) + 7) >> 3) > 0xFF) ||
		(quantity_of_coils == 0)
	) {
		modbus_controller_exception(MODBUS_ILLEGAL_DATA_VALUE);
		modbus_controller_write();
		return;
	}

	uint8_t byte_count = (((uint32_t)quantity_of_coils) + 7) >> 3;

	m_c_write_buffer[MODBUS_READ_BYTE_COUNT_INDEX] = byte_count;
	m_c_write_buffer_size = MODBUS_READ_BYTE_COUNT_INDEX + 1;

	uint32_t coil_address = starting_address;
	for(uint8_t i = 0; i < byte_count; ++i) {
		uint8_t byte_value = 0;

		for(uint8_t bit = 0; bit < 8; ++bit) {
			if(m_c_discrete_inputs[coil_address >> 3] & (1 << (coil_address & 0x07)))
				byte_value |= (1 << bit);

			++coil_address;

			if((coil_address - starting_address) >= quantity_of_coils)
				break;
		}

		m_c_write_buffer[m_c_write_buffer_size++] = byte_value;
	}

	modbus_controller_write();
}

// Function 0x03: Read Holding Registers
void process_read_holding_registers(void) {
	if((m_c_read_buffer_size - MODBUS_CRC_BYTES) < (MODBUS_QUANTITY_OF_REGISTERS_INDEX + 2))
		return;

	uint16_t starting_address = (m_c_read_buffer[MODBUS_STARTING_ADDRESS_INDEX] << 8) |
								(m_c_read_buffer[MODBUS_STARTING_ADDRESS_INDEX + 1]);

	if(starting_address >= MODBUS_CONTROLLER_HOLDING_REGISTERS_SIZE) {
		modbus_controller_exception(MODBUS_ILLEGAL_DATA_ADDRESS);
		modbus_controller_write();
		return;
	}

	uint16_t quantity_of_registers = (m_c_read_buffer[MODBUS_QUANTITY_OF_REGISTERS_INDEX] << 8) |
									 (m_c_read_buffer[MODBUS_QUANTITY_OF_REGISTERS_INDEX + 1]);

	if(
		((((uint32_t)starting_address) + quantity_of_registers) > MODBUS_CONTROLLER_HOLDING_REGISTERS_SIZE) ||
		(quantity_of_registers > 0x7F) ||																		// Can only send back 254 bytes max
		(quantity_of_registers == 0)
	) {
		modbus_controller_exception(MODBUS_ILLEGAL_DATA_VALUE);
		modbus_controller_write();
		return;
	}

	m_c_write_buffer[MODBUS_READ_BYTE_COUNT_INDEX] = (quantity_of_registers << 1);
	m_c_write_buffer_size = MODBUS_READ_BYTE_COUNT_INDEX + 1;

	for(uint32_t i = starting_address; (i - starting_address) < quantity_of_registers; ++i) {
		m_c_write_buffer[m_c_write_buffer_size++] = m_c_holding_registers[i] >> 8;
		m_c_write_buffer[m_c_write_buffer_size++] = m_c_holding_registers[i] & 0xFF;
	}

	modbus_controller_write();
}

// Function 0x04: Read Input Registers
void process_read_input_registers(void) {	// Functionally same as process_read_holding_registers
	if((m_c_read_buffer_size - MODBUS_CRC_BYTES) < (MODBUS_QUANTITY_OF_REGISTERS_INDEX + 2))
		return;

	uint16_t starting_address = (m_c_read_buffer[MODBUS_STARTING_ADDRESS_INDEX] << 8) |
								(m_c_read_buffer[MODBUS_STARTING_ADDRESS_INDEX + 1]);

	if(starting_address >= MODBUS_CONTROLLER_INPUT_REGISTERS_SIZE) {
		modbus_controller_exception(MODBUS_ILLEGAL_DATA_ADDRESS);
		modbus_controller_write();
		return;
	}

	uint16_t quantity_of_registers = (m_c_read_buffer[MODBUS_QUANTITY_OF_REGISTERS_INDEX] << 8) |
									 (m_c_read_buffer[MODBUS_QUANTITY_OF_REGISTERS_INDEX + 1]);

	if(
		((((uint32_t)starting_address) + quantity_of_registers) > MODBUS_CONTROLLER_INPUT_REGISTERS_SIZE) ||
		(quantity_of_registers > 0x7F) ||
		(quantity_of_registers == 0)
	) {
		modbus_controller_exception(MODBUS_ILLEGAL_DATA_VALUE);
		modbus_controller_write();
		return;
	}

	m_c_write_buffer[MODBUS_READ_BYTE_COUNT_INDEX] = (quantity_of_registers << 1);
	m_c_write_buffer_size = MODBUS_READ_BYTE_COUNT_INDEX + 1;

	for(uint32_t i = starting_address; (i - starting_address) < quantity_of_registers; ++i) {
		m_c_write_buffer[m_c_write_buffer_size++] = m_c_input_registers[i] >> 8;
		m_c_write_buffer[m_c_write_buffer_size++] = m_c_input_registers[i] & 0xFF;
	}

	modbus_controller_write();
}

// Function 0x05: Write Single Coil
void process_write_single_coil(void) {
	if((m_c_read_buffer_size - MODBUS_CRC_BYTES) < (MODBUS_WRITE_DATA_INDEX + 2))
		return;

	uint16_t coil_address = (m_c_read_buffer[MODBUS_COIL_ADDRESS_INDEX] << 8) |
							(m_c_read_buffer[MODBUS_COIL_ADDRESS_INDEX + 1]);

	if(coil_address >= (MODBUS_CONTROLLER_COILS_BYTE_SIZE << 3)) {
		modbus_controller_exception(MODBUS_ILLEGAL_DATA_ADDRESS);
		modbus_controller_write();
		return;
	}

	uint16_t coil_value = (m_c_read_buffer[MODBUS_WRITE_DATA_INDEX] << 8) |
						  (m_c_read_buffer[MODBUS_WRITE_DATA_INDEX + 1]);

	if(coil_value == 0xFF00)
		m_c_coils[coil_address >> 3] |= (1 << (coil_address & 0x07));
	else if(coil_value == 0x0000)
		m_c_coils[coil_address >> 3] &= ~(1 << (coil_address & 0x07));
	else {
		modbus_controller_exception(MODBUS_ILLEGAL_DATA_VALUE);
		modbus_controller_write();
		return;
	}

	m_c_write_buffer[MODBUS_COIL_ADDRESS_INDEX] 	= m_c_read_buffer[MODBUS_COIL_ADDRESS_INDEX];
	m_c_write_buffer[MODBUS_COIL_ADDRESS_INDEX + 1] = m_c_read_buffer[MODBUS_COIL_ADDRESS_INDEX + 1];
	m_c_write_buffer[MODBUS_WRITE_DATA_INDEX] 		= m_c_read_buffer[MODBUS_WRITE_DATA_INDEX];
	m_c_write_buffer[MODBUS_WRITE_DATA_INDEX + 1]	= m_c_read_buffer[MODBUS_WRITE_DATA_INDEX + 1];

	m_c_write_buffer_size = MODBUS_WRITE_DATA_INDEX + 2;	// +1 to include Lo portion of write data, +1 for count up to and including index

	modbus_controller_write();
}

// Function 0x06: Write Single Register
void process_write_single_register(void) {
	if((m_c_read_buffer_size - MODBUS_CRC_BYTES) < (MODBUS_WRITE_DATA_INDEX + 2))
		return;

	uint16_t register_address = (m_c_read_buffer[MODBUS_REGISTER_ADDRESS_INDEX] << 8) |
								(m_c_read_buffer[MODBUS_REGISTER_ADDRESS_INDEX + 1]);

	if(register_address >= MODBUS_CONTROLLER_HOLDING_REGISTERS_SIZE) {
		modbus_controller_exception(MODBUS_ILLEGAL_DATA_ADDRESS);
		modbus_controller_write();
		return;
	}

	uint16_t register_value = (m_c_read_buffer[MODBUS_WRITE_DATA_INDEX] << 8) |
							  (m_c_read_buffer[MODBUS_WRITE_DATA_INDEX + 1]);

	m_c_holding_registers[register_address] = register_value;

	m_c_write_buffer[MODBUS_REGISTER_ADDRESS_INDEX] 	= m_c_read_buffer[MODBUS_REGISTER_ADDRESS_INDEX];
	m_c_write_buffer[MODBUS_REGISTER_ADDRESS_INDEX + 1] = m_c_read_buffer[MODBUS_REGISTER_ADDRESS_INDEX + 1];
	m_c_write_buffer[MODBUS_WRITE_DATA_INDEX] 			= m_c_read_buffer[MODBUS_WRITE_DATA_INDEX];
	m_c_write_buffer[MODBUS_WRITE_DATA_INDEX + 1] 		= m_c_read_buffer[MODBUS_WRITE_DATA_INDEX + 1];

	m_c_write_buffer_size = MODBUS_WRITE_DATA_INDEX + 2;

	modbus_controller_write();
}

// Function 0x0F: Write Multiple Coils
void process_write_multiple_coils(void) {
	if((m_c_read_buffer_size - MODBUS_CRC_BYTES) < (MODBUS_WRITE_BYTE_COUNT_INDEX + 1))
		return;

	uint16_t starting_address = (m_c_read_buffer[MODBUS_STARTING_ADDRESS_INDEX] << 8) |
								(m_c_read_buffer[MODBUS_STARTING_ADDRESS_INDEX + 1]);

	if(starting_address >= (MODBUS_CONTROLLER_COILS_BYTE_SIZE << 3)) {
		modbus_controller_exception(MODBUS_ILLEGAL_DATA_ADDRESS);
		modbus_controller_write();
		return;
	}

	uint16_t quantity_of_coils = (m_c_read_buffer[MODBUS_QUANTITY_OF_COILS_INDEX] << 8) |
								 (m_c_read_buffer[MODBUS_QUANTITY_OF_COILS_INDEX + 1]);

	if(
		((((uint32_t)starting_address) + quantity_of_coils) > (MODBUS_CONTROLLER_COILS_BYTE_SIZE << 3)) ||
		(quantity_of_coils == 0)
	) {
		modbus_controller_exception(MODBUS_ILLEGAL_DATA_VALUE);
		modbus_controller_write();
		return;
	}

	uint8_t byte_count = m_c_read_buffer[MODBUS_WRITE_BYTE_COUNT_INDEX];

	if(
		(byte_count != ((((uint32_t)quantity_of_coils) + 7) >> 3)) ||
		(m_c_read_buffer_size - MODBUS_CRC_BYTES - (MODBUS_WRITE_BYTE_COUNT_INDEX + 1)) < byte_count
	) {
		modbus_controller_exception(MODBUS_ILLEGAL_DATA_VALUE);
		modbus_controller_write();
		return;
	}

	uint32_t coil_address = starting_address;
	for(uint8_t i = 0; i < byte_count; ++i) {
		for(uint8_t bit = 0; bit < 8; ++bit) {
			if(m_c_read_buffer[MODBUS_WRITE_BYTE_COUNT_INDEX + 1 + i] & (1 << bit))
				m_c_coils[coil_address >> 3] |= (1 << (coil_address & 0x07));
			else
				m_c_coils[coil_address >> 3] &= ~(1 << (coil_address & 0x07));

			++coil_address;

			if((coil_address - starting_address) >= quantity_of_coils)
				break;
		}
	}

	m_c_write_buffer[MODBUS_STARTING_ADDRESS_INDEX] 		= m_c_read_buffer[MODBUS_STARTING_ADDRESS_INDEX];
	m_c_write_buffer[MODBUS_STARTING_ADDRESS_INDEX + 1] 	= m_c_read_buffer[MODBUS_STARTING_ADDRESS_INDEX + 1];
	m_c_write_buffer[MODBUS_QUANTITY_OF_COILS_INDEX] 		= m_c_read_buffer[MODBUS_QUANTITY_OF_COILS_INDEX];
	m_c_write_buffer[MODBUS_QUANTITY_OF_COILS_INDEX + 1]	= m_c_read_buffer[MODBUS_QUANTITY_OF_COILS_INDEX + 1];

	m_c_write_buffer_size = MODBUS_QUANTITY_OF_COILS_INDEX + 2;

	modbus_controller_write();
}

// Function 0x10: Write Multiple Registers
void process_write_multiple_registers(void) {
	if((m_c_read_buffer_size - MODBUS_CRC_BYTES) < (MODBUS_WRITE_BYTE_COUNT_INDEX + 1))
		return;

	uint16_t starting_address = (m_c_read_buffer[MODBUS_STARTING_ADDRESS_INDEX] << 8) |
								(m_c_read_buffer[MODBUS_STARTING_ADDRESS_INDEX + 1]);

	if(starting_address >= MODBUS_CONTROLLER_HOLDING_REGISTERS_SIZE) {
		modbus_controller_exception(MODBUS_ILLEGAL_DATA_ADDRESS);
		modbus_controller_write();
		return;
	}

	uint16_t quantity_of_registers = (m_c_read_buffer[MODBUS_QUANTITY_OF_REGISTERS_INDEX] << 8) |
									 (m_c_read_buffer[MODBUS_QUANTITY_OF_REGISTERS_INDEX + 1]);

	if(
		(((uint32_t)starting_address) + quantity_of_registers) > MODBUS_CONTROLLER_HOLDING_REGISTERS_SIZE ||
		(quantity_of_registers == 0)
	) {
		modbus_controller_exception(MODBUS_ILLEGAL_DATA_VALUE);
		modbus_controller_write();
		return;
	}

	uint8_t byte_count = m_c_read_buffer[MODBUS_WRITE_BYTE_COUNT_INDEX];

	if(
		(byte_count >> 1) != quantity_of_registers ||
		(m_c_read_buffer_size - MODBUS_CRC_BYTES - (MODBUS_WRITE_BYTE_COUNT_INDEX + 1)) < byte_count
	) {
		modbus_controller_exception(MODBUS_ILLEGAL_DATA_VALUE);
		modbus_controller_write();
		return;
	}

	uint32_t data_index = MODBUS_WRITE_BYTE_COUNT_INDEX + 1;
	for(uint32_t i = starting_address; (i - starting_address) < quantity_of_registers; ++i) {
		m_c_holding_registers[i] = m_c_read_buffer[data_index++] << 8;
		m_c_holding_registers[i] |= m_c_read_buffer[data_index++];
	}

	m_c_write_buffer[MODBUS_STARTING_ADDRESS_INDEX] 			= m_c_read_buffer[MODBUS_STARTING_ADDRESS_INDEX];
	m_c_write_buffer[MODBUS_STARTING_ADDRESS_INDEX + 1] 		= m_c_read_buffer[MODBUS_STARTING_ADDRESS_INDEX + 1];
	m_c_write_buffer[MODBUS_QUANTITY_OF_REGISTERS_INDEX] 		= m_c_read_buffer[MODBUS_QUANTITY_OF_REGISTERS_INDEX];
	m_c_write_buffer[MODBUS_QUANTITY_OF_REGISTERS_INDEX + 1]	= m_c_read_buffer[MODBUS_QUANTITY_OF_REGISTERS_INDEX + 1];

	m_c_write_buffer_size = MODBUS_QUANTITY_OF_REGISTERS_INDEX + 2;

	modbus_controller_write();
}
