#include "ros_robot_controller_bus_servo_raw_extension.h"

#include <string.h>

/*
 * Hardware hooks to bind in the real STM32 firmware project.
 *
 * so101_bus_uart_write:
 *   Write bytes to the half-duplex bus-servo UART and return after TX complete.
 *
 * so101_bus_uart_transaction:
 *   TX raw bytes, switch to RX, collect one servo response until timeout.
 *   Return number of bytes copied into rx_buf.
 *
 * so101_pc_report_bus_servo:
 *   Send a PACKET_FUNC_BUS_SERVO report back to the PC.
 *   The existing firmware already has an equivalent report function for read
 *   responses; wire this hook to that function.
 */
extern void so101_bus_uart_write(const uint8_t *data, uint16_t len);
extern uint16_t so101_bus_uart_transaction(
    const uint8_t *tx,
    uint16_t tx_len,
    uint8_t *rx_buf,
    uint16_t rx_cap,
    uint32_t timeout_ms);
extern void so101_pc_report_bus_servo(const uint8_t *data, uint16_t len);

enum {
    SO101_RAW_MAX_LEN = 128,
    SO101_TRANSACTION_TIMEOUT_MS = 20,
};

static uint8_t hiwonder_checksum(const uint8_t *body, uint8_t body_len)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < body_len; ++i) {
        sum += body[i];
    }
    return (uint8_t)(~sum);
}

uint8_t so101_hiwonder_make_packet(
    uint8_t servo_id,
    uint8_t cmd,
    const uint8_t *params,
    uint8_t param_len,
    uint8_t *out,
    uint8_t out_cap)
{
    const uint8_t total_len = (uint8_t)(param_len + 6U);
    const uint8_t body_len = (uint8_t)(param_len + 3U);

    if (out == 0 || out_cap < total_len) {
        return 0;
    }

    out[0] = 0x55;
    out[1] = 0x55;
    out[2] = servo_id;
    out[3] = (uint8_t)(param_len + 3U);
    out[4] = cmd;

    if (param_len > 0U && params != 0) {
        memcpy(&out[5], params, param_len);
    }

    out[5U + param_len] = hiwonder_checksum(&out[2], body_len);
    return total_len;
}

static void write_load_or_unload(uint8_t servo_id, bool load)
{
    uint8_t packet[8];
    uint8_t param = load ? 1U : 0U;
    uint8_t len = so101_hiwonder_make_packet(
        servo_id,
        SO101_HIWONDER_LOAD_OR_UNLOAD_WRITE,
        &param,
        1,
        packet,
        sizeof(packet));

    if (len > 0U) {
        so101_bus_uart_write(packet, len);
    }
}

static void handle_raw_write(const uint8_t *data, uint16_t len)
{
    if (len < 2U) {
        return;
    }

    const uint8_t raw_len = data[1];
    if (raw_len == 0U || raw_len > SO101_RAW_MAX_LEN || (uint16_t)(raw_len + 2U) > len) {
        return;
    }

    so101_bus_uart_write(&data[2], raw_len);
}

static void handle_raw_transaction(const uint8_t *data, uint16_t len)
{
    uint8_t rx[SO101_RAW_MAX_LEN];
    uint8_t report[SO101_RAW_MAX_LEN + 3U];
    uint16_t rx_len = 0;

    if (len < 2U) {
        return;
    }

    const uint8_t raw_len = data[1];
    if (raw_len == 0U || raw_len > SO101_RAW_MAX_LEN || (uint16_t)(raw_len + 2U) > len) {
        return;
    }

    rx_len = so101_bus_uart_transaction(
        &data[2],
        raw_len,
        rx,
        sizeof(rx),
        SO101_TRANSACTION_TIMEOUT_MS);

    report[0] = SO101_BUS_EXT_RAW_TRANSACTION;
    report[1] = rx_len > 0U ? 0U : 1U;
    report[2] = (uint8_t)rx_len;
    if (rx_len > 0U) {
        memcpy(&report[3], rx, rx_len);
    }
    so101_pc_report_bus_servo(report, (uint16_t)(rx_len + 3U));
}

static void handle_load_or_unload_many(const uint8_t *data, uint16_t len, bool load)
{
    if (len < 2U) {
        return;
    }

    const uint8_t count = data[1];
    if (count == 0U || (uint16_t)(count + 2U) > len) {
        return;
    }

    for (uint8_t i = 0; i < count; ++i) {
        write_load_or_unload(data[2U + i], load);
    }
}

bool so101_bus_servo_handle_custom_command(const uint8_t *data, uint16_t len)
{
    if (data == 0 || len == 0U) {
        return false;
    }

    switch (data[0]) {
    case SO101_BUS_EXT_RAW_WRITE:
        handle_raw_write(data, len);
        return true;

    case SO101_BUS_EXT_RAW_TRANSACTION:
        handle_raw_transaction(data, len);
        return true;

    case SO101_BUS_EXT_UNLOAD_MANY:
        handle_load_or_unload_many(data, len, false);
        return true;

    case SO101_BUS_EXT_LOAD_MANY:
        handle_load_or_unload_many(data, len, true);
        return true;

    default:
        return false;
    }
}
