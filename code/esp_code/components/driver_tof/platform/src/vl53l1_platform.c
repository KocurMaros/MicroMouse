#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "soc/gpio_struct.h"

#include "vl53l1_platform.h"
#include "vl53l1_platform_log.h"
#include "vl53l1_api.h"

uint8_t _I2CBuffer[256];

#define I2C_MASTER_NUM                  I2C_NUM_0
#define WRITE_BIT                       I2C_MASTER_WRITE
#define READ_BIT                        I2C_MASTER_READ
#define ACK_CHECK_EN                    0x1
#define ACK_CHECK_DIS                   0x0
#define ACK_VAL                         0x0
#define NACK_VAL                        0x1

uint8_t _I2CBuffer[256];


static esp_err_t i2c_write(uint8_t dev_addr, uint8_t *buf, uint32_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write(cmd, buf, len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    // printf("i2c_write: %d\n", ret);
    if (ret != ESP_OK) {
        return ret;
    }
    return ESP_OK;
}

static esp_err_t i2c_read(uint8_t dev_addr, uint8_t *buf, uint32_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    // printf("i2c_read: %d\n", ret);
    if (ret != ESP_OK) {
        return ret;
    }
    return ESP_OK;
}

static esp_err_t i2c_master_write_bytes(uint8_t dev_addr, uint16_t reg_addr, const uint8_t *data, size_t len) {
    size_t buf_len = len + 2; // Add 2 bytes for the register address
    uint8_t *buf = (uint8_t *)malloc(buf_len);
    if (buf == NULL) {
        return ESP_ERR_NO_MEM;
    }
    buf[0] = reg_addr >> 8; // Register high byte
    buf[1] = reg_addr & 0xFF; // Register low byte
    memcpy(buf + 2, data, len); // Copy data after register address
    esp_err_t ret = i2c_write(dev_addr, buf, buf_len);
    // printf("i2c_master_write_bytes: %d\n", ret);
    free(buf);
    return ret;
}

static esp_err_t i2c_master_read_bytes(uint8_t dev_addr, uint16_t reg_addr, uint8_t *data, size_t len) {
    uint8_t reg_address[2];
    reg_address[0] = reg_addr >> 8; // Register high byte
    reg_address[1] = reg_addr & 0xFF; // Register low byte
    esp_err_t ret = i2c_write(dev_addr, reg_address, 2);
    // printf("i2c_master_read_bytes: %d\n", ret);

    if (ret != ESP_OK) {
        return ret;
    }
    return i2c_read(dev_addr, data, len);
}

static void VL53L1_GetI2cBus(void)
{
    // No action needed
}

static void VL53L1_PutI2cBus(void)
{
    // No action needed
}

VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    if (count > sizeof(_I2CBuffer) - 2) { // Subtract 2 for index bytes
        return VL53L1_ERROR_INVALID_PARAMS;
    }

    _I2CBuffer[0] = index >> 8;
    _I2CBuffer[1] = index & 0xFF;
    memcpy(&_I2CBuffer[2], pdata, count);

    status_int = i2c_master_write_bytes(Dev->I2cDevAddr, index, _I2CBuffer, count + 2);

    if (status_int != ESP_OK) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }

    // printf("VL53L1_WriteMulti: %d\n", status_int);

    return Status;
}

VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    if (count > sizeof(_I2CBuffer)) {
        return VL53L1_ERROR_INVALID_PARAMS;
    }

    _I2CBuffer[0] = index >> 8;
    _I2CBuffer[1] = index & 0xFF;

    status_int = i2c_master_read_bytes(Dev->I2cDevAddr, index, pdata, count);

    if (status_int != ESP_OK) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }

    // printf("VL53L1_ReadMulti: %d\n", status_int);

    return Status;
}


// VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count)
// {
//     int status_int;
//     VL53L1_Error Status = VL53L1_ERROR_NONE;
//     if (count > sizeof(_I2CBuffer) - 1) {
//         return VL53L1_ERROR_INVALID_PARAMS;
//     }
//     _I2CBuffer[0] = index >> 8;
//     _I2CBuffer[1] = index & 0xFF;
//     memcpy(&_I2CBuffer[2], pdata, count);
//     VL53L1_GetI2cBus();
//     status_int = i2c_master_write_bytes(Dev->I2cDevAddr, index, _I2CBuffer + 2, count);
//     if (status_int != ESP_OK) {
//         Status = VL53L1_ERROR_CONTROL_INTERFACE;
//     }
//     printf("VL53L1_WriteMulti: %d\n", status_int);
//     VL53L1_PutI2cBus();
//     return Status;
// }

// VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count)
// {
//     VL53L1_Error Status = VL53L1_ERROR_NONE;
//     int32_t status_int;

//     _I2CBuffer[0] = index >> 8;
//     _I2CBuffer[1] = index & 0xFF;
//     VL53L1_GetI2cBus();
//     status_int = i2c_master_write_bytes(Dev->I2cDevAddr, index, _I2CBuffer, 2);
//     if (status_int != ESP_OK) {
//         Status = VL53L1_ERROR_CONTROL_INTERFACE;
//         goto done;
//     }
//     status_int = i2c_master_read_bytes(Dev->I2cDevAddr, index, pdata, count);
//     if (status_int != ESP_OK) {
//         Status = VL53L1_ERROR_CONTROL_INTERFACE;
//     }
// done:
//     VL53L1_PutI2cBus();
//     printf("VL53L1_ReadMulti: %d\n", status_int);
//     return Status;
// }

VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data)
{
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index >> 8;
    _I2CBuffer[1] = index & 0xFF;
    _I2CBuffer[2] = data;

    VL53L1_GetI2cBus();
    status_int = i2c_master_write_bytes(Dev->I2cDevAddr, index, _I2CBuffer + 2, 1);
    if (status_int != ESP_OK) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    // printf("VL53L1_WrByte: %d\n", status_int);
    VL53L1_PutI2cBus();
    return Status;
}

VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data)
{
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index >> 8;
    _I2CBuffer[1] = index & 0xFF;
    _I2CBuffer[2] = data >> 8;
    _I2CBuffer[3] = data & 0xFF;

    VL53L1_GetI2cBus();
    status_int = i2c_master_write_bytes(Dev->I2cDevAddr, index, _I2CBuffer + 2, 2);
    if (status_int != ESP_OK) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    // printf("VL53L1_WrWord: %d\n", status_int);
    VL53L1_PutI2cBus();
    return Status;
}

VL53L1_Error VL53L1_WrDWord(VL53L1_DEV Dev, uint16_t index, uint32_t data)
{
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;
    _I2CBuffer[0] = index >> 8;
    _I2CBuffer[1] = index & 0xFF;
    _I2CBuffer[2] = (data >> 24) & 0xFF;
    _I2CBuffer[3] = (data >> 16) & 0xFF;
    _I2CBuffer[4] = (data >> 8) & 0xFF;
    _I2CBuffer[5] = (data >> 0) & 0xFF;
    VL53L1_GetI2cBus();
    status_int = i2c_master_write_bytes(Dev->I2cDevAddr, index, _I2CBuffer + 2, 4);
    if (status_int != ESP_OK) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    // printf("VL53L1_WrDWord: %d\n", status_int);
    VL53L1_PutI2cBus();
    return Status;
}

VL53L1_Error VL53L1_UpdateByte(VL53L1_DEV Dev, uint16_t index, uint8_t AndData, uint8_t OrData)
{
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    uint8_t data;

    Status = VL53L1_RdByte(Dev, index, &data);
    if (Status) {
        goto done;
    }
    data = (data & AndData) | OrData;
    Status = VL53L1_WrByte(Dev, index, data);
done:
    return Status;
}

VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *data)
{
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index >> 8;
    _I2CBuffer[1] = index & 0xFF;
    VL53L1_GetI2cBus();
    status_int = i2c_master_read_bytes(Dev->I2cDevAddr, index, data, 1);
    if (status_int != ESP_OK) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    VL53L1_PutI2cBus();
    // printf("VL53L1_RdByte: %d\n", status_int);
    return Status;
}

VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *data)
{
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index >> 8;
    _I2CBuffer[1] = index & 0xFF;
    VL53L1_GetI2cBus();
    status_int = i2c_master_read_bytes(Dev->I2cDevAddr, index, _I2CBuffer, 2);
    if (status_int != ESP_OK) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }

    *data = ((uint16_t)_I2CBuffer[0] << 8) + (uint16_t)_I2CBuffer[1];
done:
    VL53L1_PutI2cBus();
    // printf("VL53L1_RdWord: %d\n", status_int);
    return Status;
}

VL53L1_Error VL53L1_RdDWord(VL53L1_DEV Dev, uint16_t index, uint32_t *data)
{
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index >> 8;
    _I2CBuffer[1] = index & 0xFF;
    VL53L1_GetI2cBus();
    status_int = i2c_master_read_bytes(Dev->I2cDevAddr, index, _I2CBuffer, 4);
    if (status_int != ESP_OK) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }

    *data = ((uint32_t)_I2CBuffer[0] << 24) + ((uint32_t)_I2CBuffer[1] << 16) + ((uint32_t)_I2CBuffer[2] << 8) +
            (uint32_t)_I2CBuffer[3];

done:
    // printf("VL53L1_RdDWord: %d\n", status_int);
    VL53L1_PutI2cBus();
    return Status;
}

VL53L1_Error VL53L1_GetTickCount(uint32_t *ptick_count_ms)
{
    // Returns current tick count in [ms]
    VL53L1_Error status = VL53L1_ERROR_NONE;

    //*ptick_count_ms = timeGetTime();
    // *ptick_count_ms = 0;
    *ptick_count_ms = xTaskGetTickCount();

    return status;
}

VL53L1_Error VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
    *ptimer_freq_hz = 0;

    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitMs(VL53L1_Dev_t *pdev, int32_t wait_ms)
{
    (void)pdev;
    vTaskDelay(wait_ms / portTICK_PERIOD_MS);
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t wait_us)
{
    (void)pdev;
    vTaskDelay((wait_us / 1000) / portTICK_PERIOD_MS);
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitValueMaskEx(VL53L1_Dev_t *pdev,
                                     uint32_t timeout_ms,
                                     uint16_t index,
                                     uint8_t value,
                                     uint8_t mask,
                                     uint32_t poll_delay_ms)
{
    VL53L1_Error status = VL53L1_ERROR_NONE;
    uint32_t start_time_ms = 0;
    uint32_t current_time_ms = 0;
    uint32_t polling_time_ms = 0;
    uint8_t byte_value = 0;
    uint8_t found = 0;
    char register_name[VL53L1_MAX_STRING_LENGTH];

    /* look up register name */
#ifdef PAL_EXTENDED
    VL53L1_get_register_name(index, register_name);
#else
    VL53L1_COPYSTRING(register_name, "");
#endif

    /* calculate time limit in absolute time */

    VL53L1_GetTickCount(&start_time_ms);
    /* wait until value is found, timeout reached on error occurred */

    while ((status == VL53L1_ERROR_NONE) && (polling_time_ms < timeout_ms) && (found == 0)) {

        if (status == VL53L1_ERROR_NONE)
            status = VL53L1_RdByte(pdev, index, &byte_value);
        if ((byte_value & mask) == value)
            found = 1;

        if (status == VL53L1_ERROR_NONE && found == 0 && poll_delay_ms > 0)
            status = VL53L1_WaitMs(pdev, poll_delay_ms);
        /* Update polling time (Compare difference rather than absolute to
        negate 32bit wrap around issue) */
        VL53L1_GetTickCount(&current_time_ms);
        polling_time_ms = current_time_ms - start_time_ms;
    }

    if (found == 0 && status == VL53L1_ERROR_NONE)
        status = VL53L1_ERROR_TIME_OUT;

    printf("wait value mask status: %d\n", status);
    return status;
}
