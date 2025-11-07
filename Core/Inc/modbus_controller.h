#ifndef MODBUS_CONTROLLER_H
#define MODBUS_CONTROLLER_H

#include "modbus_io.h"
#include "modbus_constants.h"

static volatile uint8_t modbus_controller_address;

static volatile uint8_t modbus_controller_buffer_size;
static volatile uint8_t modbus_controller_buffer[MODBUS_IO_BUFFER_SIZE];

void modbus_controller_init(void);

void modbus_controller_tick(void); // Call every tick, checks if Modbus message is available and processes it

#endif
