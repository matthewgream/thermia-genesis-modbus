#ifndef THERMIA_MODBUS_H
#define THERMIA_MODBUS_H

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------

#include <stdbool.h>

typedef enum { MODEL_MEGA = 0x01, MODEL_INVERTER = 0x2 } model_t;

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------

bool thermia_modbus_open(const char *address, int port, model_t model);
void thermia_modbus_close(void);

bool thermia_modbus_read_register_bit(const char *name, bool *value);
/*
 * Note: Values are returned raw. Apply scaling if needed:
 *   - Most temperatures: divide by 10
 *   - Currents: divide by 100
 */
bool thermia_modbus_read_register_int(const char *name, int *value);
bool thermia_modbus_write_register_bit(const char *name, bool value);
/*
 * Write an integer register (holding register only)
 *
 * Note: Values should be pre-scaled:
 *   - For 22.0°C, write 220 (scale factor 10)
 *   - For 15.50A, write 1550 (scale factor 100)
 */
bool thermia_modbus_write_register_int(const char *name, int value);

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------

#endif /* THERMIA_MODBUS_H */
