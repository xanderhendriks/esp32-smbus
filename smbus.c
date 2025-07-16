/* MIT License
 *
 * Copyright (c) 2017 David Antliff
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file smbus.c
 * @brief SMBus helper layer implemented on top of the ESP-IDF v5.x *new* I²C master driver.
 *
 * The implementation keeps the original high-level API intact while replacing all
 * low-level legacy calls with the following new primitives declared in
 * `driver/i2c_master.h`:
 *   • `i2c_master_transmit()` – write transaction
 *   • `i2c_master_receive()`  – pure read transaction
 *   • `i2c_master_transmit_receive()` – combined write/read with repeated-START
 *
 * The driver uses an opaque `i2c_master_dev_handle_t` instead of raw port numbers
 * and 7-bit addresses.  A handle is obtained once via `i2c_master_bus_add_device()`
 * during initialisation in `main.c` and passed to this module via
 * `smbus_init_device_handle()`.
 */

#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

#include "smbus.h"

#define MAX_BLOCK_LEN         255      /**< SMBus 3.0 maximum block length. */
#define SMBUS_DEFAULT_TIMEOUT 1000     /**< Default timeout in milliseconds. */

static const char *TAG = "smbus";

/**
 * @brief Internal SMBus context.
 */
struct smbus_info_t
{
    i2c_master_dev_handle_t dev;  /**< Device handle obtained from new I²C driver. */
    uint32_t                timeout_ms; /**< Per-transaction timeout. */
    bool                    init;       /**< Indicates successful initialisation. */
};

/* ------------------------------------------------------------------------- */
/*                      ───  PRIVATE HELPERS  ───                            */
/* ------------------------------------------------------------------------- */

/**
 * @brief Test whether the context pointer is valid and initialised.
 */
static bool _is_init(const smbus_info_t *info)
{
    if (info && info->init) {
        return true;
    }
    ESP_LOGE(TAG, "smbus_info not initialised");
    return false;
}

/**
 * @brief Translate ESP-IDF error codes to readable log messages.
 */
static esp_err_t _check_err(esp_err_t err)
{
    if (err == ESP_OK) {
        return ESP_OK;
    }
    ESP_LOGE(TAG, "I²C error: %d", err);
    return err;
}

/**
 * @brief Write arbitrary bytes to a device register.
 *
 * @param[in] info    SMBus context.
 * @param[in] command Register/command code.
 * @param[in] data    Data buffer to transmit (may be NULL when len==0).
 * @param[in] len     Number of data bytes.
 *
 * @return ESP_OK on success.
 */
static esp_err_t _write_bytes(const smbus_info_t *info, uint8_t command,
                              const uint8_t *data, size_t len)
{
    if (!_is_init(info)) {
        return ESP_FAIL;
    }

    uint8_t txbuf[MAX_BLOCK_LEN + 1];
    if (len > MAX_BLOCK_LEN) {
        ESP_LOGE(TAG, "write length %d > %d", (int)len, MAX_BLOCK_LEN);
        return ESP_ERR_INVALID_ARG;
    }

    txbuf[0] = command;
    if (data && len) {
        memcpy(&txbuf[1], data, len);
    }

    return _check_err(i2c_master_transmit(info->dev, txbuf, len + 1, info->timeout_ms));
}

/**
 * @brief Read arbitrary bytes from a device register (write-then-read with repeated-START).
 *
 * @param[in]  info    SMBus context.
 * @param[in]  command Register/command code.
 * @param[out] data    Destination buffer.
 * @param[in]  len     Number of bytes to read.
 */
static esp_err_t _read_bytes(const smbus_info_t *info, uint8_t command,
                             uint8_t *data, size_t len)
{
    if (!_is_init(info)) {
        return ESP_FAIL;
    }
    if (!data || !len) {
        return ESP_ERR_INVALID_ARG;
    }

    return _check_err(i2c_master_transmit_receive(info->dev, &command, 1,
                                                  data, len, info->timeout_ms));
}

/* ------------------------------------------------------------------------- */
/*                      ───  PUBLIC API  ───                                 */
/* ------------------------------------------------------------------------- */

/**
 * @brief Allocate an SMBus context.
 */
smbus_info_t *smbus_malloc(void)
{
    smbus_info_t *info = calloc(1, sizeof(*info));
    if (!info) {
        ESP_LOGE(TAG, "malloc failed");
    }
    return info;
}

/**
 * @brief Free an SMBus context.
 */
void smbus_free(smbus_info_t **info)
{
    if (info && *info) {
        free(*info);
        *info = NULL;
    }
}

/**
 * @brief Initialise the context with an I²C device handle.
 */
esp_err_t smbus_init_device_handle(smbus_info_t *info, i2c_master_dev_handle_t dev)
{
    if (!info) {
        return ESP_ERR_INVALID_ARG;
    }
    info->dev        = dev;
    info->timeout_ms = SMBUS_DEFAULT_TIMEOUT;
    info->init       = true;
    return ESP_OK;
}

/**
 * @brief Set per-transaction timeout.
 */
esp_err_t smbus_set_timeout(smbus_info_t *info, uint32_t timeout_ms)
{
    if (!_is_init(info)) {
        return ESP_FAIL;
    }
    info->timeout_ms = timeout_ms;
    return ESP_OK;
}

/* --------  Simple SMBus transactions  ----------------------------------- */

esp_err_t smbus_send_byte(const smbus_info_t *info, uint8_t data)
{
    /* SMBus definition: command byte == data for Send-Byte. */
    return _write_bytes(info, data, NULL, 0);
}

esp_err_t smbus_receive_byte(const smbus_info_t *info, uint8_t *data)
{
    /* SMBus Receive-Byte: no command phase, only address + read. */
    if (!_is_init(info) || !data) {
        return ESP_FAIL;
    }
    return _check_err(i2c_master_receive(info->dev, data, 1, info->timeout_ms));
}

esp_err_t smbus_write_byte(const smbus_info_t *info, uint8_t command, uint8_t data)
{
    return _write_bytes(info, command, &data, 1);
}

esp_err_t smbus_write_word(const smbus_info_t *info, uint8_t command, uint16_t data)
{
    uint8_t buf[2] = { (uint8_t)(data & 0xFF), (uint8_t)(data >> 8) };
    return _write_bytes(info, command, buf, 2);
}

esp_err_t smbus_read_byte(const smbus_info_t *info, uint8_t command, uint8_t *data)
{
    return _read_bytes(info, command, data, 1);
}

esp_err_t smbus_read_word(const smbus_info_t *info, uint8_t command, uint16_t *data)
{
    uint8_t buf[2];
    esp_err_t err = _read_bytes(info, command, buf, 2);
    if (err == ESP_OK && data) {
        *data = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    }
    return err;
}

esp_err_t smbus_write_block(const smbus_info_t *info, uint8_t command,
                            uint8_t *data, uint8_t len)
{
    if (len > MAX_BLOCK_LEN) {
        return ESP_ERR_INVALID_ARG;
    }
    /* SMBus block write prefixes length. */
    uint8_t buf[MAX_BLOCK_LEN + 2];
    buf[0] = command;
    buf[1] = len;
    memcpy(&buf[2], data, len);
    return _check_err(i2c_master_transmit(info->dev, buf, len + 2, info->timeout_ms));
}

esp_err_t smbus_read_block(const smbus_info_t *info, uint8_t command,
                           uint8_t *data, uint8_t *len)
{
    if (!data || !len) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Read first byte to learn length */
    uint8_t length = 0;
    esp_err_t err = i2c_master_transmit_receive(info->dev, &command, 1,
                                                &length, 1, info->timeout_ms);
    if (err != ESP_OK) {
        *len = 0;
        return _check_err(err);
    }

    uint8_t to_read = (length > *len) ? *len : length;
    err = i2c_master_receive(info->dev, data, to_read, info->timeout_ms);
    *len = (err == ESP_OK) ? to_read : 0;
    return _check_err(err);
}

/* Convenience wrappers that keep the old function names ------------------ */

esp_err_t smbus_i2c_write_block(const smbus_info_t *info, uint8_t command,
                                uint8_t *data, size_t len)
{
    return _write_bytes(info, command, data, len);
}

esp_err_t smbus_i2c_read_block(const smbus_info_t *info, uint8_t command,
                               uint8_t *data, size_t len)
{
    return _read_bytes(info, command, data, len);
}
