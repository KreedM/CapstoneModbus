#include "modbus_controller.h"
#include "modbus_io.h"

static uint8_t modbus_controller_address;

static uint32_t modbus_controller_buffer_size;
static uint8_t modbus_controller_buffer[MODBUS_IO_BUFFER_SIZE];

static uint8_t coils[256] = {0};
static uint8_t discrete_inputs[256] = {0};
static uint16_t holding_registers[128] = {0};
static uint16_t input_registers[128] = {0};

void modbus_controller_init(void) {
	modbus_controller_address = 0xFF;
	holding_registers[0] = 0x1234;
	holding_registers[1] = 0x5678;
	input_registers[0] = 0xABCD;
	input_registers[1] = 0xEF01;
	coils[0] = 1;
	coils[10] = 1;
	discrete_inputs[5] = 1;
}

void process_modbus_message(void);

uint16_t CRC_calc(uint32_t length);

void modbus_controller_tick(void) {
	modbus_controller_buffer_size = modbus_io_read(modbus_controller_buffer);

	if(modbus_controller_buffer_size < MODBUS_MIN_MESSAGE_BYTES)
		return;

	if(modbus_controller_buffer[0] != modbus_controller_address)
		return;

	uint16_t crc = (modbus_controller_buffer[modbus_controller_buffer_size - 2] << 8) |
			   	   (modbus_controller_buffer[modbus_controller_buffer_size - 1]);

	if(crc != CRC_calc(modbus_controller_buffer_size - 1))
		return;

	process_modbus_message();
}

uint16_t CRC_calc(uint32_t length) {
    uint8_t temp;
    uint16_t crc = 0xFFFF;  // Initialize CRC register

    for (uint32_t i = 0; i < length; ++i)
	{
    	temp = modbus_controller_buffer[i] ^ crc;  	// XOR byte with low byte of CRC
		crc >>= 8;                                  // Shift CRC right 8 bits
		crc ^= crc_table[temp];                     // XOR with table lookup
	}

    return crc;
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


