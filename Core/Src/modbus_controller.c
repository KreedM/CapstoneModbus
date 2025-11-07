#include "modbus_controller.h"
#include "modbus_io.h"

void modbus_controller_init(void) {
	modbus_controller_address = 0xFF;
}

void modbus_controller_tick(void) {
	modbus_controller_buffer_size = modbus_io_read(modbus_controller_buffer);

	if(modbus_controller_buffer_size < MODBUS_MIN_MESSAGE_BYTES)
		return;

	if(modbus_controller_buffer[0] != modbus_controller_address)
		return;

	uint16_t CRC = modbus_controller_buffer[modbus_controller_buffer_size - 2] << 8 |
			   modbus_controller_buffer[modbus_controller_buffer_size - 1] ;

	if(CRC != CRC_calc(modbus_controller_buffer_size))
		return;

	process_modbus_message();
}

uint16_t CRC_calc(uint32_t length)
{
    uint8_t temp;
    uint16_t crc = 0xFFFF;  // Initialize CRC register

    for (uint32_t i = 0; i < length; i++)
        {
            temp = modbus_controller_buffer[i] ^ crc;  // XOR byte with low byte of CRC
            crc >>= 8;                                  // Shift CRC right 8 bits
            crc ^= crc_table[temp];                     // XOR with table lookup
       }

    return crc;
}





