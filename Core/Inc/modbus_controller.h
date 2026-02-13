#ifndef MODBUS_CONTROLLER_H
#define MODBUS_CONTROLLER_H

#include <stdbool.h>

#include "modbus_io.h"
#include "modbus_constants.h"

// Max 2^13 - 1
#define MODBUS_CONTROLLER_COILS_BYTE_SIZE 			128
#define MODBUS_CONTROLLER_DISCRETE_INPUTS_BYTE_SIZE 128

// Max 2^16 - 1
#define MODBUS_CONTROLLER_HOLDING_REGISTERS_SIZE 	128
#define MODBUS_CONTROLLER_INPUT_REGISTERS_SIZE 		128

void modbus_controller_init(uint8_t address); // Sets address

void modbus_controller_tick(void); // Call every tick, checks if Modbus message is available and processes it

bool modbus_controller_read_coils(uint8_t *dst, uint16_t starting_address, uint16_t quantity_of_coils);
bool modbus_controller_read_discrete_inputs(uint8_t *dst, uint16_t starting_address, uint16_t quantity_of_coils);

bool modbus_controller_read_holding_registers(uint16_t *dst, uint16_t starting_address, uint16_t quantity_of_registers);
bool modbus_controller_read_input_registers(uint16_t *dst, uint16_t starting_address, uint16_t quantity_of_registers);

bool modbus_controller_write_single_coil(uint16_t coil_address, uint16_t coil_value);										// Pass in MODBUS_COIL_ON or MODBUS_COIL_OFF
bool modbus_controller_write_single_discrete_input(uint16_t coil_address, uint16_t coil_value);								// Pass in MODBUS_COIL_ON or MODBUS_COIL_OFF

bool modbus_controller_write_single_holding_register(uint16_t register_address, uint16_t register_value);
bool modbus_controller_write_single_input_register(uint16_t register_address, uint16_t register_value);

bool modbus_controller_write_multiple_coils(uint8_t *src, uint16_t starting_address, uint16_t quantity_of_coils);			// Lowest address is src LSB, but subsequent bytes are all higher
bool modbus_controller_write_multiple_discrete_inputs(uint8_t *src, uint16_t starting_address, uint16_t quantity_of_coils);	// Lowest address is src LSB, but subsequent bytes are all higher

bool modbus_controller_write_multiple_holding_registers(uint16_t *src, uint16_t register_address, uint16_t register_value);
bool modbus_controller_write_multiple_input_registers(uint16_t *src, uint16_t register_address, uint16_t register_value);

#endif
