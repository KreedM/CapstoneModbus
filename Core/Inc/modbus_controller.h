#ifndef MODBUS_CONTROLLER_H
#define MODBUS_CONTROLLER_H

#include "modbus_io.h"

static volatile uint8_t modbus_controller_buffer[MODBUS_IO_BUFFER_SIZE];

void process_modbus_message(void);
void modbus_controller_tick(void);
uint16_t CRC_calc(uint8_t *data);

#endif
