#ifndef MODBUS_CONTROLLER_H
#define MODBUS_CONTROLLER_H

#include "modbus_io.h"
#include "modbus_constants.h"

void modbus_controller_init(void);

void modbus_controller_tick(void); // Call every tick, checks if Modbus message is available and processes it

#endif
