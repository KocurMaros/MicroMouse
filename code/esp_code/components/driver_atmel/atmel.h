/**
 * @file MAX11254.h
 * @author Kocur Maros (kocur.maros@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-01-12
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __ATMEL_H__
#define __ATMEL_H__
#include "freertos/FreeRTOS.h"
#include "driver/spi_master.h"
#include "../../main/main.h"
#include "driver/gpio.h"

#include "atmel_reg.h"
#include "VALUES.h"

#define bitRead(value,bit) (((value) >> (bit)) & 0x01)
#define bitClear(value,bit) ((value) &= ~(1UL << (bit)))
#define bitSet(value,bit) ((value) |= (1UL << (bit)))

/** SPI Defines*/
#define SCK     18
#define MOSI    23 
#define MISO    19 
#define CLOCK_SPEED 8000000
#define CS      21

#define ATMEL_RESET 5555 

/**
 * @brief   Init SPI device as master (MOSI - 19, MISO - 23, 8MHz, no pull)
 * @note    SPI ports are defined
 * @return  esp_err_t 
 */
esp_err_t spi_master_init(void);
esp_err_t atmel_init();
void atmel_reset();
void atmel_write(uint8_t reg, uint8_t reg_val_HSB, uint8_t reg_val_MSB, uint8_t reg_val_LSB, int length);
uint32_t atmel_read(uint8_t reg, int length);
void atmel_command(uint8_t cmd);

#endif
