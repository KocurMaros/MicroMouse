#include "driver/i2c.h"
#include "vl53l1_api.h"

#define I2C_MASTER_SCL_IO 22 //make menuconfig 
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_FREQ_HZ 400000

void i2c_init(void);
void vl53l1_init(VL53L1_Dev_t *dev, int address);
void setupManualCalibration(VL53L1_Dev_t *dev);
void readResults(VL53L1_Dev_t *dev);
uint16_t vl53l1_read(VL53L1_Dev_t *dev);