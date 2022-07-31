
#include "bmx160.h"
#include "bmx160_reg.h"

static float accelRange =   BMX160_ACCEL_RAW;
static float gyroRange  =   BMX160_GYRO_SENSITIVITY_1000DPS*DEG_TO_RAD;

int begin_bmx160(bmx160_spi * bmxIMU_spi_p)
{
    uint8_t chipID = 0;

    readReg_bmx160(bmxIMU_spi_p, BMX160_STARTUP_WRITE_REG, NULL, 1);
    //writeReg_bmx160(bmxIMU_spi_p,BMX160_NV_CONF_REG,0b10000000);
    getChipID_bmx160(bmxIMU_spi_p, &chipID);
#if USE_BMI160
    if(chipID != BMI160_CHIP_ID)
    {
        return BMX160_WRONG_CHIP_ID;
    }
#elif USE_BMX160
    if(chipID != BMX160_CHIP_ID)
    {
        return BMX160_WRONG_CHIP_ID;
    }
#else
    #error Please Select only the sed CHIP in 'bmx160.h'
#endif

    /* Set accel to normal mode */
    setSensorMode_bmx160(bmxIMU_spi_p, bmx160_accel_normal_mode, BMX160_ACCEL_DELAY_MS);    
    /* Set gyro to normal mode */
    setSensorMode_bmx160(bmxIMU_spi_p, bmx160_gyro_normal_mode, BMX160_GRYO_DELAY_MS);  
#if (USE_BMX160 && !USE_BMI160)
    // /* Set mag to normal mode */
    setSensorMode_bmx160(bmxIMU_spi_p, bmx160_magn_lowpower_mode, BMX160_MAGN_DELAY_MS);
#elif (USE_BMX160 && USE_BMI160)
    #error Please Select only the sed CHIP in 'bmx160.h'
#endif

    //set accel and gyro data rate, bwp and range
    //Accel at 1600Hz, 3dB Cutoff frequency 80 Hz - Normal filter mode
    setAccel_ODR_BWP_bmx160(bmxIMU_spi_p, bmx160_accel_odr_1600, bmx160_accel_bwp_osr_1);
    //Gyro at 1600Hz, 3dB Cutoff frequceny 74.6 Hz - Normal filter mode
    setGyro_ODR_BWP_bmx160(bmxIMU_spi_p, bmx160_gyr_odr_1600, bmx160_gyr_bwp_osr_1);

    setAccelRange_bmx160(bmxIMU_spi_p, bmx160_accelRange_4g);
    setGyroRange_bmx160(bmxIMU_spi_p, bmx160_gyroRange_1000dps);

    //getData_bmx160(bmxIMU_spi_p, NULL);
    return BMX160_OK;
}

int getChipID_bmx160(bmx160_spi * bmxIMU_spi_p, uint8_t * chipID)
{
    int result;
    result = readReg_bmx160(bmxIMU_spi_p, BMX160_CHIP_ID_REG, chipID, 1);
    return result;
}

int getStatus_bmx160(bmx160_spi * bmxIMU_spi_p, bmx160_status_flags flagType, bool * flagSet)
{
    int result;
    uint8_t regRetVal;
    result = readReg_bmx160(bmxIMU_spi_p, BMX160_STATUS_REG, &regRetVal, 1);
    switch (flagType)
    {
    case bmx160_drdy_acc:
        if(((regRetVal & bmx160_drdy_acc) >> 7) == 1) *flagSet = true;
        break;
    case bmx160_drdy_gyr:
        if(((regRetVal & bmx160_drdy_gyr) >> 6) == 1) *flagSet = true;
        break;
    case bmx160_nvm_rdy:
        if(((regRetVal & bmx160_nvm_rdy) >> 4) == 1) *flagSet = true;
        break;
    case bmx160_foc:
        if(((regRetVal & bmx160_foc) >> 3) == 1) *flagSet = true;
        break;
    default:
        flagSet = false;
        break;
    }
    return result;
}

int getError_bmx160(bmx160_spi * bmxIMU_spi_p, bmx160_error_flags errType, bool * flagSet)
{
    int result;
    uint8_t regRetVal;
    result = readReg_bmx160(bmxIMU_spi_p, BMX160_ERR_REG, &regRetVal, 1);
    switch (errType)
    {
    case bmx160_drop_cmd_err:
        if(((regRetVal & 0b01000000) >> 6) == 1) *flagSet = true;
        break;
    case bmx160_fatal_err:
        if(((regRetVal & 0b00000001)) == 1) *flagSet = true;
        break;
    case bmx160_error_1:
        if(((regRetVal & 0b00000010) >> 1) == 1) *flagSet = true;
        break;
    case bmx160_error_2:
        if(((regRetVal & 0b00000100) >> 2) == 1) *flagSet = true;
        break;
    case bmx160_error_LPMintPFD:
        if(((regRetVal & 0b00000110) >> 1) == 0b00000011) *flagSet = true;
        break;
    case bmx160_ODRsNOmatch:
        if(((regRetVal & 0b00001100) >> 2) == 0b00000011) *flagSet = true;
        break;
    case bmx160_PDFinLPM:
        if(((regRetVal & 0b00001110) >> 1) == 0b00000111) *flagSet = true;
        break;

    default:
        flagSet = false;
        break;
    }
    return result;
}

void setAccel_ODR_BWP_bmx160(bmx160_spi * bmxIMU_spi_p, bmx160_accel_odr passVal_odr, bmx160_accel_osr passVal_osr)
{
    uint8_t sentVal = 0b0;
    sentVal = (uint8_t)(passVal_odr | passVal_osr);
    writeReg_bmx160(bmxIMU_spi_p, BMX160_ACC_CONF_REG, sentVal);
}

void setGyro_ODR_BWP_bmx160(bmx160_spi * bmxIMU_spi_p, bmx160_gyr_odr passVal_odr, bmx160_gyr_osr passVal_osr)
{
    uint8_t sentVal = 0b0;
    sentVal = (uint8_t)(passVal_odr | passVal_osr);
    writeReg_bmx160(bmxIMU_spi_p, BMX160_GYR_CONF_REG, sentVal);
}

void setAccelRange_bmx160(bmx160_spi * bmxIMU_spi_p, bmx160_accel_range passVal)
{
    uint8_t sentVal = 0b0;
    sentVal = (uint8_t)passVal;
    writeReg_bmx160(bmxIMU_spi_p, BMX160_ACC_RANGE_REG, sentVal);
    switch (passVal){
        case bmx160_accelRange_2g:
            accelRange = BMX160_ACCEL_MG_LSB_2G * G_CONST;
            break;
        case bmx160_accelRange_4g:
            accelRange = BMX160_ACCEL_MG_LSB_4G * G_CONST;
            break;
        case bmx160_accelRange_8g:
            accelRange = BMX160_ACCEL_MG_LSB_8G * G_CONST;
            break;
        case bmx160_accelRange_16g:
            accelRange = BMX160_ACCEL_MG_LSB_16G * G_CONST;
            break;
        default:
            accelRange = BMX160_ACCEL_MG_LSB_2G * G_CONST;
            break;
    }
}

void setGyroRange_bmx160(bmx160_spi * bmxIMU_spi_p, bmx160_gyro_range passVal)
{
    uint8_t sentVal = 0b0;
    sentVal = (uint8_t)passVal;
    writeReg_bmx160(bmxIMU_spi_p, BMX160_GYR_RANGE_REG, sentVal);
    switch (passVal){
        case bmx160_gyroRange_125dps:
            gyroRange = BMX160_GYRO_SENSITIVITY_125DPS;
            break;
        case bmx160_gyroRange_250dps:
            gyroRange = BMX160_GYRO_SENSITIVITY_250DPS;
            break;
        case bmx160_gyroRange_500dps:
            gyroRange = BMX160_GYRO_SENSITIVITY_500DPS;
            break;
        case bmx160_gyroRange_1000dps:
            gyroRange = BMX160_GYRO_SENSITIVITY_1000DPS;
            break;
        case bmx160_gyroRange_2000dps:
            gyroRange = BMX160_GYRO_SENSITIVITY_2000DPS;
            break;
        default:
            gyroRange = BMX160_GYRO_SENSITIVITY_250DPS;
            break;
    }
}

void softReset_bmx160(bmx160_spi * bmxIMU_spi_p)
{
    uint8_t data = (uint8_t)bmx160_soft_reset;
    writeReg_bmx160(bmxIMU_spi_p, BMX160_CMD_REG, data);
    k_sleep(K_MSEC(BMX160_SOFT_RESET_DELAY_MS));
}

void setSensorMode_bmx160(bmx160_spi * bmxIMU_spi_p, bmx160_commands_send passVal, uint8_t sleepTime)
{
    uint8_t data = (uint8_t)passVal;
    writeReg_bmx160(bmxIMU_spi_p, BMX160_CMD_REG, data);
    k_sleep(K_MSEC(sleepTime));
}

uint8_t do_foc_bmx160(bmx160_spi * bmxIMU_spi_p, int16_t *retOffsetVal, bool saveToNVM)
{
    //TODO
    return 0;
}

/*
float getAccX(bmx160_spi * bmxIMU_spi_p);
{
    uint8_t data[2] = {0};
    int16_t x=0,y=0,z=0;
    result = readReg(bmxIMU_spi_p, BMX160_MAG_X_LSB_REG, data, 20);
    x = (int16_t) (((uint16_t)data[1] << 8) | data[0]);


}*/
uint8_t getData_bmx160(bmx160_spi * bmxIMU_spi_p, bmx160_sensor_data * sen_data)
{
    int result;
    int16_t x=0,y=0,z=0; 
    if (sen_data == NULL)
    {
        return BMX160_E_NULL_PTR;
    }
    
#if USE_BMX160
    uint8_t data[20] = {0};
    result = readReg_bmx160(bmxIMU_spi_p, BMX160_MAG_X_LSB_REG, data, 20);
    
    x = (int16_t) (((uint16_t)data[9] << 8) | data[8]);
    y = (int16_t) (((uint16_t)data[11] << 8) | data[10]);
    z = (int16_t) (((uint16_t)data[13] << 8) | data[12]);
    sen_data->gx = (float)(x) * gyroRange;
    sen_data->gy = (float)(y) * gyroRange;
    sen_data->gz = (float)(z) * gyroRange;

    x = (int16_t) (((uint16_t)data[15] << 8) | data[14]);
    y = (int16_t) (((uint16_t)data[17] << 8) | data[16]);
    z = (int16_t) (((uint16_t)data[19] << 8) | data[18]);
    sen_data->ax = (float)(x) * accelRange;
    sen_data->ay = (float)(y) * accelRange;
    sen_data->az = (float)(z) * accelRange;
#elif USE_BMI160
    uint8_t data[12] = {0};
    result = readReg_bmx160(bmxIMU_spi_p, BMX160_GYR_X_LSB_REG, data, 12);
    
    x = (int16_t) (((uint16_t)data[1] << 8) | data[0]);
    y = (int16_t) (((uint16_t)data[3] << 8) | data[2]);
    z = (int16_t) (((uint16_t)data[5] << 8) | data[4]);
    sen_data->gx = (float)(x) * gyroRange;
    sen_data->gy = (float)(y) * gyroRange;
    sen_data->gz = (float)(z) * gyroRange;

    x = (int16_t) (((uint16_t)data[7] << 8) | data[6]);
    y = (int16_t) (((uint16_t)data[9] << 8) | data[8]);
    z = (int16_t) (((uint16_t)data[11] << 8) | data[10]);
    sen_data->ax = (float)(x) * accelRange;
    sen_data->ay = (float)(y) * accelRange;
    sen_data->az = (float)(z) * accelRange;
#else
    #error Please Select only the sed CHIP in 'bmx160.h'
#endif

    return result;
}

int writeReg_bmx160(bmx160_spi * bmxIMU_spi_p, uint8_t reg, uint8_t data)
{
    int result;
	const struct spi_buf buf[2] = {
		{
			.buf = &reg,
			.len = 1,
		},
		{
			.buf = &data,
			.len = 1,
		}
	};
	const struct spi_buf_set tx = {
		.buffers = buf,
		.count = 2,
	};
	gpio_pin_set(bmxIMU_spi_p->gpio0_dev, bmxIMU_spi_p->spi_cs_pin, 1);

	//result = spi_write_dt(bus, &tx);
	result = spi_write(bmxIMU_spi_p->spi_dev, &bmxIMU_spi_p->spi_cfg, &tx);
	if (result) 
    {
        return result;
		//printk("SPI error: %d\n", result);
	}
    gpio_pin_set(bmxIMU_spi_p->gpio0_dev, bmxIMU_spi_p->spi_cs_pin, 0);
    return 0;
}

int readReg_bmx160(bmx160_spi * bmxIMU_spi_p, uint8_t reg, uint8_t *data, size_t len)
{
    int result;
	unsigned char tx_buffer[2] = { 0, 0};

	tx_buffer[0] = 0x80 | reg;

	const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = 1,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	struct spi_buf rx_buf[2] = {
		{
			.buf = tx_buffer,
			.len = 1,
		},
		{
			.buf = data,
			.len = len,
		}
	};

	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 2,
	};
	gpio_pin_set(bmxIMU_spi_p->gpio0_dev, bmxIMU_spi_p->spi_cs_pin, 1);
	result = spi_transceive(bmxIMU_spi_p->spi_dev, &bmxIMU_spi_p->spi_cfg, &tx, &rx);
    gpio_pin_set(bmxIMU_spi_p->gpio0_dev, bmxIMU_spi_p->spi_cs_pin, 0);

	if (result) {
		return result;
	}

	return 0;
}

/*

void setLowPower(bmx160_spi * bmxIMU_spi_p)
{
    softReset(bmxIMU_spi_p);
    k_sleep(K_MSEC(100));
    setMagnConf(bmxIMU_spi_p);
    k_sleep(K_MSEC(100));
    Set accel to low power mode 
    writeReg(bmxIMU_spi_p, BMX160_COMMAND_REG_ADDR, BMX160_ACCEL_LOWPOWER_MODE);
    k_sleep(K_MSEC(100));
    Set gyro to low power mode 
    writeReg(bmxIMU_spi_p, BMX160_COMMAND_REG_ADDR, BMX160_GYRO_FASTSTARTUP_MODE);
    k_sleep(K_MSEC(100));
    Set mag to low power mode 
    writeReg(bmxIMU_spi_p, BMX160_COMMAND_REG_ADDR, BMX160_MAGN_LOWPOWER_MODE);
    k_sleep(K_MSEC(100));
}

void wakeUp(bmx160_spi * bmxIMU_spi_p)
{
    softReset(bmxIMU_spi_p);
    k_sleep(K_MSEC(100));
    setMagnConf(bmxIMU_spi_p);
    k_sleep(K_MSEC(100));
    Set accel to normal power mode 
    writeReg(bmxIMU_spi_p, BMX160_COMMAND_REG_ADDR, BMX160_ACCEL_NORMAL_MODE);
    k_sleep(K_MSEC(100));
    Set gyro to normal power mode 
    writeReg(bmxIMU_spi_p, BMX160_COMMAND_REG_ADDR, BMX160_GYRO_NORMAL_MODE);
    k_sleep(K_MSEC(100));
    Set mag to normal power mode 
    writeReg(bmxIMU_spi_p, BMX160_COMMAND_REG_ADDR, BMX160_MAGN_NORMAL_MODE);
    k_sleep(K_MSEC(100));
}
*/