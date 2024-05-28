#include "tof.h"

void i2c_init(void)
{
	int i2c_master_port = I2C_MASTER_NUM;
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = I2C_MASTER_SDA_IO,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_io_num = I2C_MASTER_SCL_IO,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_MASTER_FREQ_HZ,
	};
	i2c_param_config(i2c_master_port, &conf);
	i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void vl53l1_init(VL53L1_Dev_t *dev, int address)
{
	// int status = VL53L1_WaitDeviceBooted(dev);
	int status;
	VL53L1_WrByte(dev, VL53L1_SOFT_RESET, 0x00);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	VL53L1_WrByte(dev, VL53L1_SOFT_RESET, 0x01);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	VL53L1_range_data_t cal_data;
	uint16_t osc_calibrate_val;
	VL53L1_RdWord(dev, VL53L1_OSC_MEASURED__FAST_OSC__FREQUENCY, &cal_data.fast_osc_frequency);
	VL53L1_RdWord(dev, VL53L1_RESULT__OSC_CALIBRATE_VAL, &osc_calibrate_val);
	VL53L1_WrWord(dev, VL53L1_DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, 0x0A00);
	VL53L1_WrByte(dev, VL53L1_GPIO__TIO_HV_STATUS, 0x02);
	VL53L1_WrByte(dev, VL53L1_SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8);
	VL53L1_WrByte(dev, VL53L1_SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16);
	VL53L1_WrByte(dev, VL53L1_ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01);
	VL53L1_WrByte(dev, VL53L1_ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF);
	VL53L1_WrByte(dev, VL53L1_ALGO__RANGE_MIN_CLIP, 0);
	VL53L1_WrByte(dev, VL53L1_ALGO__CONSISTENCY_CHECK__TOLERANCE, 2);
	VL53L1_WrWord(dev, VL53L1_SYSTEM__THRESH_RATE_HIGH, 0x0000);
	VL53L1_WrWord(dev, VL53L1_SYSTEM__THRESH_RATE_LOW, 0x0000);
	VL53L1_WrByte(dev, VL53L1_DSS_CONFIG__APERTURE_ATTENUATION, 0x38);
	VL53L1_WrWord(dev, VL53L1_RANGE_CONFIG__SIGMA_THRESH, 360);
	VL53L1_WrWord(dev, VL53L1_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192);
	VL53L1_WrByte(dev, VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01);
	VL53L1_WrByte(dev, VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01);
	VL53L1_WrByte(dev, VL53L1_SD_CONFIG__QUANTIFIER, 2);
	VL53L1_WrByte(dev, VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD, 0x00);
	VL53L1_WrByte(dev, VL53L1_SYSTEM__SEED_CONFIG, 1);
	VL53L1_WrByte(dev, VL53L1_SYSTEM__SEQUENCE_CONFIG, 0x8B);
	VL53L1_WrWord(dev, VL53L1_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8);
	VL53L1_WrByte(dev, VL53L1_DSS_CONFIG__ROI_MODE_CONTROL, 2);

	//set distance
	//set measurement timing budget     MAYBE PROBLEM ?

	uint16_t outer_offset_mm;

	VL53L1_SetDistanceMode(dev, VL53L1_DISTANCEMODE_LONG); //Max distance in dark:Short:136cm Medium:290cm long:360cm
	VL53L1_SetMeasurementTimingBudgetMicroSeconds(dev, 160000);
	VL53L1_RdWord(dev, VL53L1_MM_CONFIG__OUTER_OFFSET_MM, &outer_offset_mm);
	VL53L1_WrWord(dev, VL53L1_ALGO__PART_TO_PART_RANGE_OFFSET_MM, outer_offset_mm * 4);

	//start measurement

	//set the address here!
	uint8_t byteData;
	status = VL53L1_SetDeviceAddress(dev, address * 2);
	dev->I2cDevAddr = address;
	status = VL53L1_RdByte(dev, 0x0001, &byteData);

	printf("status = %d\n", status);
	printf("deviceAddress = %d\n", byteData);

	VL53L1_WrDWord(dev, VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, 50 * osc_calibrate_val);
	VL53L1_WrByte(dev, VL53L1_SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range
	VL53L1_WrByte(dev, VL53L1_SYSTEM__MODE_START, 0x40);	  // mode_range__timed
}

void setupManualCalibration(VL53L1_Dev_t *dev)
{
	uint8_t saved_vhv_init;
	uint8_t saved_vhv_timeout;
	VL53L1_RdByte(dev, VL53L1_VHV_CONFIG__INIT, &saved_vhv_init);
	VL53L1_RdByte(dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, &saved_vhv_timeout);
	VL53L1_WrByte(dev, VL53L1_VHV_CONFIG__INIT, saved_vhv_init & 0x7F);
	VL53L1_WrByte(dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, (saved_vhv_timeout & 0x03) + (3 << 2));
	VL53L1_WrByte(dev, VL53L1_PHASECAL_CONFIG__OVERRIDE, 0x01);
	uint8_t phasecal_result;
	VL53L1_RdByte(dev, VL53L1_PHASECAL_RESULT__VCSEL_START, &phasecal_result);
	VL53L1_WrByte(dev, VL53L1_CAL_CONFIG__VCSEL_START, phasecal_result);
}

uint16_t readResults(VL53L1_Dev_t *dev)
{
	uint8_t results[17];


	VL53L1_ReadMulti(dev, VL53L1_RESULT__RANGE_STATUS, results, 17);

	uint8_t rangeStatus = results[0];
	uint8_t streamCount = results[2];
	uint16_t dssActualEffectiveSpadsSD0 = (results[3] << 8) + results[4];
	uint16_t ambientCountRateMCPS = (results[7] << 8) + results[8];
	uint16_t finalCrossTalkCorrectedMm = (results[13] << 8) + results[14];
	uint16_t peakSignalCountRateCrosstalk = (results[15] << 8) + results[16];

	if (!dev->calibrated) {
		setupManualCalibration(dev);
		dev->calibrated = 1;
	}

	uint16_t spadCount = dssActualEffectiveSpadsSD0;
	if (spadCount != 0) {
		// "Calc total rate per spad"

		uint32_t totalRatePerSpad = (uint32_t)peakSignalCountRateCrosstalk + ambientCountRateMCPS;

		// "clip to 16 bits"
		if (totalRatePerSpad > 0xFFFF) {
			totalRatePerSpad = 0xFFFF;
		}

		// "shift up to take advantage of 32 bits"
		totalRatePerSpad <<= 16;

		totalRatePerSpad /= spadCount;

		if (totalRatePerSpad != 0) {
			// "get the target rate and shift up by 16"
			uint32_t requiredSpads = ((uint32_t)0x0A00 << 16) / totalRatePerSpad;

			// "clip to 16 bit"
			if (requiredSpads > 0xFFFF) {
				requiredSpads = 0xFFFF;
			}

			// "override DSS config"
			VL53L1_WrWord(dev, VL53L1_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, requiredSpads);
			// DSS_CONFIG__ROI_MODE_CONTROL should already be set to REQUESTED_EFFFECTIVE_SPADS
		}
	}
	uint16_t range_mm = ((uint32_t)finalCrossTalkCorrectedMm * 2011 + 0x0400) / 0x0800;
	return range_mm;
	//todo: status ?
}

uint16_t vl53l1_read(VL53L1_Dev_t *dev)
{
	//TODO: make calibrated be sensor specific
	uint16_t timeout_start = xTaskGetTickCount();
	uint16_t io_timeout = 500;
	uint8_t byteData;
	VL53L1_RdByte(dev, VL53L1_GPIO__TIO_HV_STATUS, &byteData);

	while (!(byteData & 0x01) == 0) {
		if ((io_timeout > 0) && ((uint16_t)(xTaskGetTickCount() - timeout_start) > io_timeout)) {
			return -1;
		}
		VL53L1_RdByte(dev, VL53L1_GPIO__TIO_HV_STATUS, &byteData);
	}

	return readResults(dev);
}
