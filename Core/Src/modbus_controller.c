#include "modbus_controller.h"
#include "modbus_io.h"

void modbus_controller_init(void) {
	modbus_controller_address = 0xFF;
}

void process_modbus_message(void);

uint16_t CRC_calc(uint8_t *data);

void modbus_controller_tick(void) {
	modbus_controller_buffer_size = modbus_io_read(modbus_controller_buffer);

	if(modbus_controller_buffer_size < MODBUS_MIN_MESSAGE_BYTES)
		return;

	if(modbus_controller_buffer[0] != modbus_controller_address)
		return;

	uint16_t CRC = (modbus_controller_buffer[modbus_controller_buffer_size - 2] << 8) |
			   	   (modbus_controller_buffer[modbus_controller_buffer_size - 1]);

	if(CRC != CRC_calc(modbus_controller_buffer_size - 1))
		return;

	process_modbus_message();
}
