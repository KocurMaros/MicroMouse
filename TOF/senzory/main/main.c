#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>
#include "vl53l0x.h"


#define I2C_MASTER_SCL_IO 22         // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO 21         // GPIO number for I2C master data
#define I2C_MASTER_NUM I2C_NUM_0     // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ 400000    // Updated I2C master clock frequency to a valid value

void i2c_master_init()
{
    i2c_config_t conf;
    conf.clk_flags = 0;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}



void app_main(void)
{
    i2c_master_init();
    
    vl53l0x_t *vl53l0x = vl53l0x_config(1, 22, 21, 23, 0x29, 0);
    if (vl53l0x == NULL)
    {
        printf("Could not configure VL53L0X\n");
        return;
    }
    const char *err = vl53l0x_init(vl53l0x);
    if (err)
    {
        printf("Could not initialise VL53L0X: %s\n", err);
        return;
    }
    printf("VL53L0X initialised\n");
    vl53l0x_setTimeout(vl53l0x, 500);
    vl53l0x_startContinuous(vl53l0x, 100);
    while (1)
    {
        printf("Range: %dmm\n", vl53l0x_readRangeContinuousMillimeters(vl53l0x));
        vTaskDelay(100/portTICK_PERIOD_MS);
    }

    vl53l0x_stopContinuous(vl53l0x);
    vl53l0x_end(vl53l0x);
}



// #define I2C_MASTER_SCL_IO    22   /*!< Default I2C master clock GPIO pin */
// #define I2C_MASTER_SDA_IO    21   /*!< Default I2C master data GPIO pin */
// #define I2C_MASTER_NUM       I2C_NUM_0  /*!< I2C port number for master dev */
// #define I2C_MASTER_FREQ_HZ   100000    /*!< I2C master clock frequency */

// void i2c_master_init() {
//     i2c_config_t conf = {
//         .mode = I2C_MODE_MASTER,
//         .sda_io_num = I2C_MASTER_SDA_IO,
//         .sda_pullup_en = GPIO_PULLUP_ENABLE,
//         .scl_io_num = I2C_MASTER_SCL_IO,
//         .scl_pullup_en = GPIO_PULLUP_ENABLE,
//         .master.clk_speed = I2C_MASTER_FREQ_HZ,
//     };
//     i2c_param_config(I2C_MASTER_NUM, &conf);
//     i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
// }

// void i2c_scan_task(void *param) {
//     i2c_master_init();

//     printf("Scanning I2C bus...\n");

//     for (uint8_t address = 1; address <= 127; ++address) {
//         i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//         i2c_master_start(cmd);
//         i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
//         i2c_master_stop(cmd);

//         esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
//         i2c_cmd_link_delete(cmd);

//         if (ret == ESP_OK) {
//             printf("Found device at address 0x%02X\n", address);
//         }
//     }

//     printf("Scan complete.\n");

//     vTaskDelete(NULL);
// }

// void app_main() {
//     xTaskCreate(i2c_scan_task, "i2c_scan_task", 2048, NULL, 10, NULL);
// }
