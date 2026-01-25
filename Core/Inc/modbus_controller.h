#ifndef MODBUS_CONTROLLER_H
#define MODBUS_CONTROLLER_H

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

#endif
