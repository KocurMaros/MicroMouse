#include "atmel.h"
#include "../../build/config/sdkconfig.h"
#include "esp_log.h"
#include "nvs.h"

/* For memset */
#include <string.h>
#include <inttypes.h>


spi_device_handle_t spi3;
bool spi_init = false;

esp_err_t spi_master_init(void){
    spi_bus_config_t buscfg={
    .miso_io_num = MISO,
    .mosi_io_num = MOSI,
    .sclk_io_num = SCK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    };
    ESP_LOGI("SPI","... Init bus");
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, 1));
    ESP_LOGI("SPI","... Init handle");
    
    
    spi_device_interface_config_t devcfg={
        .address_bits = 0,
        .command_bits = 8,
        .mode = 0,                      //SPI mode 0
        .duty_cycle_pos = 0,
        .cs_ena_posttrans = 0,
        .cs_ena_pretrans = 0,
        .clock_speed_hz = CLOCK_SPEED,
        .spics_io_num = CS,     
        .flags = 0,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL,
    };
    ESP_LOGI("SPI", "... Adding device bus.");
    return spi_bus_add_device(SPI3_HOST, &devcfg, &spi3);
}

void atmel_reset(){
    gpio_set_level(ATMEL_RESET, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(ATMEL_RESET, 1);
}

esp_err_t atmel_init(){
    
    esp_err_t ret;
    gpio_set_direction(ATMEL_RESET, GPIO_MODE_OUTPUT);
    
    if(!spi_init){
        ret = spi_master_init();
        ESP_LOGI("SPI", "... Bus Initilized.");
        spi_init = true;
    }else{
        ret = ESP_OK;
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return ret;
}

uint32_t atmel_read(uint8_t reg, int length){
   
    uint8_t read_command = 0xC1;	
	uint32_t result = 0;		
	

	read_command |= (reg << 1);
    
	spi_transaction_t reg_read;	 
	
	memset(&reg_read, 0, sizeof(reg_read));  //sets trans_desc1 to zero.  Errors will occur if this is not done.
    
    reg_read.addr = 0;
	reg_read.cmd  = read_command;
	reg_read.flags = SPI_TRANS_USE_RXDATA; 

    if(length == 8){	// read 1 byte
		reg_read.length = length;    
		reg_read.user =(void*)1;
		ESP_ERROR_CHECK(spi_device_transmit(spi3, &reg_read));
		result = *(uint32_t *)reg_read.rx_data; 
	}else if(length == 16){
		reg_read.length = length;    
		reg_read.user =(void*)1;
		ESP_ERROR_CHECK(spi_device_transmit(spi3, &reg_read));
		//result = __bswap_32(*(uint32_t *)reg_read.rx_data);
		result = reg_read.rx_data[0];
	 	result = (result << 8);
	 	result = (result | reg_read.rx_data[1]);
	}
	else{		
		reg_read.length = length;    
		reg_read.user =(void*)1;
        ESP_ERROR_CHECK(spi_device_transmit(spi3, &reg_read));
		//result = __bswap_32(*(uint32_t *)reg_read.rx_data);
		result = reg_read.rx_data[0];
	 	result = (result << 16);
	 	result = (result | (reg_read.rx_data[1] << 8));
	 	result = (result | reg_read.rx_data[2]);
	}		

    return result;

}

