#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Reference extension for the Hiwonder ros_robot_controller STM32 firmware.
 *
 * The PC-side packet remains:
 *   AA 55 function=0x05 data_len data... crc8
 *
 * This extension adds bus-servo subcommands inside data[0]:
 *   0xF0 RAW_WRITE      data = [F0, raw_len, raw_bytes...]
 *   0xF1 RAW_TRANSACTION data = [F1, raw_len, raw_bytes...]
 *   0xF2 UNLOAD_MANY    data = [F2, id_count, ids...]
 *   0xF3 LOAD_MANY      data = [F3, id_count, ids...]
 *
 * Integrate bus_servo_handle_custom_command() into the existing bus-servo
 * command dispatcher before the default stock command handling.
 */

#ifdef __cplusplus
extern "C" {
#endif

enum {
    SO101_BUS_EXT_RAW_WRITE = 0xF0,
    SO101_BUS_EXT_RAW_TRANSACTION = 0xF1,
    SO101_BUS_EXT_UNLOAD_MANY = 0xF2,
    SO101_BUS_EXT_LOAD_MANY = 0xF3,
};

enum {
    SO101_HIWONDER_POS_READ = 28,
    SO101_HIWONDER_LOAD_OR_UNLOAD_WRITE = 31,
    SO101_HIWONDER_LOAD_OR_UNLOAD_READ = 32,
};

bool so101_bus_servo_handle_custom_command(const uint8_t *data, uint16_t len);
uint8_t so101_hiwonder_make_packet(
    uint8_t servo_id,
    uint8_t cmd,
    const uint8_t *params,
    uint8_t param_len,
    uint8_t *out,
    uint8_t out_cap);

#ifdef __cplusplus
}
#endif
