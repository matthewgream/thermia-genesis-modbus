
// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------

#include <errno.h>
#include <modbus.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common_thermia.h"

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------

typedef enum { REG_COIL_STATUS = 0x01, REG_INPUT_STATUS = 0x02, REG_INPUT = 0x04, REG_HOLDING = 0x08 } reg_type_t;

typedef struct {
    const char *name;
    reg_type_t type;
    int address;
    int defacto;
    int scale;
    model_t model;
    const char *system;
    const char *subsystem;
    const char *description;
} register_def_t;

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------

static modbus_t *g_ctx = NULL;
static model_t g_model = MODEL_MEGA;

static const register_def_t g_registers[] = {
#include "common_thermia_registers.h"
};
static const int g_num_registers = sizeof(g_registers) / sizeof(register_def_t);

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------

static const register_def_t *find_register(const char *name, reg_type_t type) {
    for (int i = 0; i < g_num_registers; i++)
        if (strcmp(g_registers[i].name, name) == 0 && g_registers[i].type & type)
            return &g_registers[i];
    return NULL;
}
static bool is_register_supported(const register_def_t *reg) { return reg->model & g_model; }

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------

bool thermia_modbus_open(const char *address, int port, model_t model) {
    if (g_ctx != NULL) {
        fprintf(stderr, "modbus: initialised already\n");
        return false;
    }
    g_model = model;
    g_ctx = modbus_new_tcp(address, port);
    if (g_ctx == NULL) {
        fprintf(stderr, "modbus: initialisation failed: %s\n", modbus_strerror(errno));
        return false;
    }
    modbus_set_slave(g_ctx, 1);
    if (modbus_connect(g_ctx) == -1) {
        fprintf(stderr, "modbus: connection failed: %s\n", modbus_strerror(errno));
        modbus_free(g_ctx);
        g_ctx = NULL;
        return false;
    }
    printf("modbus: connected to %s:502 (model: %s)\n", address, model == MODEL_MEGA ? "MEGA" : "INVERTER");
    return true;
}

void thermia_modbus_close(void) {
    if (g_ctx != NULL) {
        modbus_close(g_ctx);
        modbus_free(g_ctx);
        g_ctx = NULL;
    }
}

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------

#define MODBUS_CHECK(ctx)                                                                                                                                      \
    if (ctx == NULL) {                                                                                                                                         \
        fprintf(stderr, "modbus: not initialised/connected\n");                                                                                                \
        return false;                                                                                                                                          \
    }
#define REGISTER_CHECK(reg)                                                                                                                                    \
    if (reg == NULL) {                                                                                                                                         \
        fprintf(stderr, "register: '%s' not found\n", name);                                                                                                   \
        return false;                                                                                                                                          \
    }                                                                                                                                                          \
    if (!is_register_supported(reg)) {                                                                                                                         \
        fprintf(stderr, "register: '%s' not supported by model\n", name);                                                                                      \
        return false;                                                                                                                                          \
    }

bool thermia_modbus_read_register_bit(const char *name, bool *value) {
    MODBUS_CHECK(g_ctx);

    const register_def_t *reg = find_register(name, REG_COIL_STATUS | REG_INPUT_STATUS);
    REGISTER_CHECK(reg);

    uint8_t reg_value;
    int rc;
    if (reg->type == REG_COIL_STATUS)
        rc = modbus_read_bits(g_ctx, reg->address, 1, &reg_value);
    else
        rc = modbus_read_input_bits(g_ctx, reg->address, 1, &reg_value);
    if (rc == -1) {
        fprintf(stderr, "register: failed to read bit: %s\n", modbus_strerror(errno));
        return false;
    }

    *value = reg_value != 0;
    return true;
}

bool thermia_modbus_read_register_int(const char *name, int *value) {
    MODBUS_CHECK(g_ctx);

    const register_def_t *reg = find_register(name, REG_INPUT | REG_HOLDING);
    REGISTER_CHECK(reg);

    uint16_t reg_value;
    int rc;
    if (reg->type == REG_INPUT)
        rc = modbus_read_input_registers(g_ctx, reg->address, 1, &reg_value);
    else
        rc = modbus_read_registers(g_ctx, reg->address, 1, &reg_value);
    if (rc == -1) {
        fprintf(stderr, "register: failed to read int: %s\n", modbus_strerror(errno));
        return false;
    }

    *value = (int16_t)reg_value;
    return true;
}

bool thermia_modbus_write_register_bit(const char *name, bool value) {
    MODBUS_CHECK(g_ctx);

    const register_def_t *reg = find_register(name, REG_COIL_STATUS);
    REGISTER_CHECK(reg);

    if (modbus_write_bit(g_ctx, reg->address, value ? 1 : 0) == -1) {
        fprintf(stderr, "register: failed to write bit: %s\n", modbus_strerror(errno));
        return false;
    }

    return true;
}

bool thermia_modbus_write_register_int(const char *name, int value) {
    MODBUS_CHECK(g_ctx);

    const register_def_t *reg = find_register(name, REG_HOLDING);
    REGISTER_CHECK(reg);

    if (modbus_write_register(g_ctx, reg->address, (uint16_t)value) == -1) {
        fprintf(stderr, "register: failed to write int: %s\n", modbus_strerror(errno));
        return false;
    }

    return true;
}

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------

#ifdef TEST

void print_usage(const char *prog) {
    printf("Usage:\n");
    printf("  %s <address> <model> read <register_name>\n", prog);
    printf("  %s <address> <model> write <register_name> <value>\n", prog);
    printf("\n");
    printf("Models: mega, inverter\n");
    printf("\n");
    printf("Examples:\n");
    printf("  %s 192.168.0.106 mega read valueHeatpumpBrineInTemperature\n", prog);
    printf("  %s 192.168.0.106 mega read alarmHeatpumpBrineInSensor\n", prog);
    printf("  %s 192.168.0.106 mega write enableHeatpumpResetAllAlarms 1\n", prog);
}

int main(int argc, char *argv[]) {
    if (argc < 5) {
        print_usage(argv[0]);
        return 1;
    }

    const char *address = argv[1], *model_str = argv[2], *operation = argv[3];

    model_t model;
    if (strcmp(model_str, "mega") == 0)
        model = MODEL_MEGA;
    else if (strcmp(model_str, "inverter") == 0)
        model = MODEL_INVERTER;
    else {
        fprintf(stderr, "model unknown, must be 'mega' or 'inverter': %s\n", model_str);
        return EXIT_FAILURE;
    }

    if (!thermia_modbus_open(address, 502, model))
        return EXIT_FAILURE;

    if (strcmp(operation, "read") == 0) {
        for (int argi = 4; argi < argc; argi++) {
            const char *reg_name = argv[argi];
            const register_def_t *reg = find_register(reg_name, REG_COIL_STATUS | REG_INPUT_STATUS | REG_INPUT | REG_HOLDING);
            if (reg == NULL) {
                fprintf(stderr, "register: not found '%s'\n", reg_name);
                goto failure;
            }
            if (reg->type == REG_COIL_STATUS || reg->type == REG_INPUT_STATUS) {
                bool value;
                if (thermia_modbus_read_register_bit(reg_name, &value))
                    printf("%s = %d (read)\n", reg_name, value ? 1 : 0);
            } else {
                int value;
                if (thermia_modbus_read_register_int(reg_name, &value)) {
                    if (reg->scale > 1)
                        printf("%s = %.2f (read) (raw = %d)\n", reg_name, (double)value / reg->scale, value);
                    else
                        printf("%s = %d (read)\n", reg_name, value);
                }
            }
        }
    } else if (strcmp(operation, "write") == 0) {
        if (argc < 6) {
            fprintf(stderr, "register: missing value for write operation\n");
            goto failure;
        }
        const char *reg_name = argv[4];
        const register_def_t *reg = find_register(reg_name, REG_COIL_STATUS | REG_HOLDING);
        if (reg == NULL) {
            fprintf(stderr, "register: not found '%s'\n", reg_name);
            goto failure;
        }
        const int value = atoi(argv[5]);
        if (reg->type == REG_COIL_STATUS) {
            if (thermia_modbus_write_register_bit(reg_name, value != 0))
                printf("%s = %d (write)\n", reg_name, value);
        } else {
            if (thermia_modbus_write_register_int(reg_name, value))
                printf("%s = %d (write)\n", reg_name, value);
        }
    } else {
        fprintf(stderr, "operation unknown: %s\n", operation);
        goto failure;
    }

    thermia_modbus_close();
    return EXIT_SUCCESS;

failure:
    thermia_modbus_close();
    return EXIT_FAILURE;
}

#endif

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------
