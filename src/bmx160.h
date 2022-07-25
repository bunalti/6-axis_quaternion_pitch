#ifndef BMX_160_H_
#define BMX_160_H_

#include <device.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <sys/util.h>
#include <zephyr/types.h>

#include "bmx160_reg.h"

/**
 * @brief
 * BMI160 6-axis and BMX160 9-axis sensors
 * are register compatible. Adding BMI160
 * support to enable lower power applications.
 **/
 

#define USE_BMX160                      0
#define USE_BMI160                      1



/**
 * @brief TODO:
 * -make FOC function procedure (+ saving factors to NVM procedure)
 * -axis orientation setup
 * -error handler at lower layer for every function call (inside driver file)
 * -error handler at upper layer for every function call (inside main.c file)
 * -reset mode 
 * -low power modes
 */

/**
 * @struct bmx160_spi
 * @brief  holds all spi relevant data
 */
typedef struct{
	const struct spi_config spi_cfg;
	const struct device * spi_dev;
	const int spi_cs_pin;
	const struct device * gpio0_dev;
}bmx160_spi;

/**
 * @struct bmx160_sensor_data
 * @brief holds x/y/z accel and gyro sensor data
 */
typedef struct {
  float ax;           /**< X-axis sensor data */
  float ay;           /**< Y-axis sensor data */
  float az;           /**< Z-axis sensor data */

  float gx;           /**< X-axis sensor data */
  float gy;           /**< Y-axis sensor data */
  float gz;           /**< Z-axis sensor data */
}bmx160_sensor_data;

/**
 * @fn begin_bmx160
 * @brief Startup initalization process for the BMX160 IMU.
 * @return Returns true/false if reading succeeded
 */
int begin_bmx160(bmx160_spi * bmxIMU_spi_p); 

/**
 * @fn getChipID_bmx160
 * @brief Read Chip ID and pass it by reference
 * @return Returns true/false if reading succeeded
 */
int getChipID_bmx160(bmx160_spi * bmxIMU_spi_p, uint8_t *chipID); 

/**
 * @fn getStatus_bmx160
 * @brief Read BMX160 status flag by type and return by reference if it is set or not
 * @return Returns true/false if reading succeeded
 */
int getStatus_bmx160(bmx160_spi * bmxIMU_spi_p, bmx160_status_flags flagType, bool * flagSet); 

/**
 * @fn getError_bmx160
 * @brief Read BMX160 error flag by type and return by reference if it is set or not
 * @return Returns true/false if reading succeeded
 */
int getError_bmx160(bmx160_spi * bmxIMU_spi_p, bmx160_error_flags errType, bool * flagSet);

/**
 * @fn setAccelDataRate_bmx160
 * @brief sets BMX160 accelerometer odr and bwp
 */
void setAccel_ODR_BWP_bmx160(bmx160_spi * bmxIMU_spi_p, bmx160_accel_odr passVal_odr, bmx160_accel_osr passVal_osr);

/**
 * @fn setGyro_ODR_BWP_bmx160
 * @brief sets BMX160 gyroscope odr and bwp
 */
void setGyro_ODR_BWP_bmx160(bmx160_spi * bmxIMU_spi_p, bmx160_gyr_odr passVal_odr, bmx160_gyr_osr passVal_osr);

/**
 * @fn setAccelRange_bmx160
 * @brief sets BMX160 accelerometer g range
 */
void setAccelRange_bmx160(bmx160_spi * bmxIMU_spi_p, bmx160_accel_range passVal);

/**
 * @fn setGyroRange_bmx160
 * @brief sets BMX160 gyroscope dps range
 */
void setGyroRange_bmx160(bmx160_spi * bmxIMU_spi_p, bmx160_gyro_range passVal);

/**
 * @fn softReset_bmx160bmx160_spi
 * @brief resets BMX160 imu
 */
void softReset_bmx160(bmx160_spi * bmxIMU_spi_p);

/**
 * @fn setSensorMode_bmx160
 * @brief sets BMX160 mode for one of the sensors to suspend/normal/faststartup/lowpower mode
 */
void setSensorMode_bmx160(bmx160_spi * bmxIMU_spi_p, bmx160_commands_send passVal, uint8_t sleepTime);

/**
 * @fn do_foc_bmx160
 * @brief Fast offset configuration with options to save to Non volatile memory and to return offset values
 * @return returns 0 if successful
 * @retval 0 if successful
 * @retval error if rading failed
 */
uint8_t do_foc_bmx160(bmx160_spi * bmxIMU_spi_p, int16_t *retOffsetVal, bool saveToNVM);

/**
 * @fn getAllData_bmx160
 * @brief get the gyro and accel data 
 * @return returns 0 if  successful
 * @retval 0 if successful
 * @retval error if rading failed
 */
uint8_t getData_bmx160(bmx160_spi * bmxIMU_spi_p, bmx160_sensor_data * sen_data);

/**
 * @fn readReg
 * @brief get the sensor IIC data
 * @param reg register
 * @param pBuf write the store and buffer of the data
 * @param len data length to be readed
 */
int readReg_bmx160(bmx160_spi * bmxIMU_spi_p, uint8_t reg, uint8_t *data, size_t len);

/**
 * @fn writeReg
 * @brief write the sensor IIC data
 * @param reg register
 * @param pBuf write the store and buffer of the data
 * @param len data length to be written 
 * @return return the actually written length
 * @fn writeBmxReg
 * @brief Write data to the BMX register
 * @param reg register
 * @param value  Data written to the BMX register
 * @return return the actually written length
 */
int writeReg_bmx160(bmx160_spi * bmxIMU_spi_p, uint8_t reg, uint8_t data);

#endif /* BMX_160_H */