#ifndef BMX_160_REG_H_
#define BMX_160_REG_H_


#define NUM_OF_DATA_READ                20
#define BMX160_SOFT_RESET_DELAY_MS      50
#define BMX160_ACCEL_DELAY_MS           4
#define BMX160_GRYO_DELAY_MS            80
#define BMX160_MAGN_DELAY_MS            1

#define G_CONST                         9.81f
#define DEG_TO_RAD                      0.0174533f

/** Error code definitions */
#define BMX160_OK                                0
#define BMX160_E_NULL_PTR                        -1
#define BMX160_E_COM_FAIL                        -2
#define BMX160_E_DEV_NOT_FOUND                   -3
#define BMX160_E_OUT_OF_RANGE                    -4
#define BMX160_E_INVALID_INPUT                   -5
#define BMX160_E_ACCEL_ODR_BW_INVALID            -6
#define BMX160_E_GYRO_ODR_BW_INVALID             -7
#define BMX160_E_LWP_PRE_FLTR_INT_INVALID        -8
#define BMX160_E_LWP_PRE_FLTR_INVALID            -9
#define BMX160_E_MAGN_NOT_FOUND                  -10
#define BMX160_FOC_FAILURE                       -11
#define BMX160_ERR_CHOOSE                        -12
#define BMX160_WRONG_CHIP_ID                     -99

#define BMX160_CHIP_ID                  0xD8
#define BMI160_CHIP_ID                  0xD1

#define BMX160_NVM_PROG_EN              0x02
#define BMX160_GYRO_OFFSET_ENABLE       0x80
#define BMX160_ACCEL_OFFSET_ENABLE      0x40

#define BMX160_ACCEL_RAW            1.000000000F
#define BMX160_ACCEL_MG_LSB_2G      0.000061035F   ///< Macro for mg per LSB at +/- 2g sensitivity (1 LSB = 0.000061035mg) */
#define BMX160_ACCEL_MG_LSB_4G      0.000122070F   ///< Macro for mg per LSB at +/- 4g sensitivity (1 LSB = 0.000122070mg) */
#define BMX160_ACCEL_MG_LSB_8G      0.000244141F   ///< Macro for mg per LSB at +/- 8g sensitivity (1 LSB = 0.000244141mg) */
#define BMX160_ACCEL_MG_LSB_16G     0.000488281F   ///< Macro for mg per LSB at +/- 16g sensitivity (1 LSB = 0.000488281mg) */



#define BMX160_GYRO_RAW                     1.0000000F ///< Use raw gyroscope data */      
#define BMX160_GYRO_SENSITIVITY_125DPS      0.0038110F ///< Gyroscope sensitivity at 125dps */
#define BMX160_GYRO_SENSITIVITY_250DPS      0.0076220F ///< Gyroscope sensitivity at 250dps */
#define BMX160_GYRO_SENSITIVITY_500DPS      0.0152439F ///< Gyroscope sensitivity at 500dps */
#define BMX160_GYRO_SENSITIVITY_1000DPS     0.0304878F ///< Gyroscope sensitivity at 1000dps */
#define BMX160_GYRO_SENSITIVITY_2000DPS     0.0609756F ///< Gyroscope sensitivity at 2000dps */

//General registers:
#define BMX160_STARTUP_WRITE_REG    0x7F
#define BMX160_CHIP_ID_REG          0x00
#define BMX160_ERR_REG              0x02
#define BMX160_STATUS_REG           0x1B
#define BMX160_ACC_CONF_REG         0x40
#define BMX160_ACC_RANGE_REG        0x41
#define BMX160_GYR_CONF_REG         0x42
#define BMX160_GYR_RANGE_REG        0x43
#define BMX160_FOC_CONF_REG         0x69
#define BMX160_CONF_REG             0x6A    //Enable NVM programming
#define BMX160_NV_CONF_REG          0x70
#define BMX160_CMD_REG              0x7E

//Data registers:
#define BMX160_MAG_X_LSB_REG        0x04
#define BMX160_MAG_X_MSB_REG        0x05
#define BMX160_MAG_Y_LSB_REG        0x06
#define BMX160_MAG_Y_MSB_REG        0x07
#define BMX160_MAG_Z_LSB_REG        0x08
#define BMX160_MAG_Z_MSB_REG        0x09
#define BMX160_RHALL_LSB_REG        0x0A
#define BMX160_RHALL_MSB_REG        0x0B
#define BMX160_GYR_X_LSB_REG        0x0C
#define BMX160_GYR_X_MSB_REG        0x0D
#define BMX160_GYR_Y_LSB_REG        0x0E
#define BMX160_GYR_Y_MSB_REG        0x0F
#define BMX160_GYR_Z_LSB_REG        0x10
#define BMX160_GYR_Z_MSB_REG        0x11
#define BMX160_ACC_X_LSB_REG        0x12
#define BMX160_ACC_X_MSB_REG        0x13
#define BMX160_ACC_Y_LSB_REG        0x14
#define BMX160_ACC_Y_MSB_REG        0x15
#define BMX160_ACC_Z_LSB_REG        0x16
#define BMX160_ACC_Z_MSB_REG        0x17

//Offset registers:
#define BMX160_OFFSET_ACC_X           0x71
#define BMX160_OFFSET_ACC_Y           0x72
#define BMX160_OFFSET_ACC_Z           0x73
#define BMX160_OFFSET_GYR_X           0x74
#define BMX160_OFFSET_GYR_Y           0x75
#define BMX160_OFFSET_GYR_Z           0x76

//Must mask bwp with odr before being written to BMX160_ACC_CONF_REG
/**
 * @enum bmx160_accel_osr
 */
typedef enum{
    bmx160_accel_bwp_osr_1      = (0x20),
    bmx160_accel_bwp_osr_2      = (0x10),
    bmx160_accel_bwp_osr_4      = (0x00)
} bmx160_accel_osr;

/**
 * @enum bmx160_accel_odr
 */
typedef enum{
    bmx160_accel_odr_12_5       = (0x05),
    bmx160_accel_odr_25         = (0x06),
    bmx160_accel_odr_50         = (0x07),
    bmx160_accel_odr_100        = (0x08),
    bmx160_accel_odr_200        = (0x09),
    bmx160_accel_odr_400        = (0x0A),
    bmx160_accel_odr_800        = (0x0B),
    bmx160_accel_odr_1600       = (0x0C)
} bmx160_accel_odr;

//Must mask bwp with odr before being written to BMX160_GYR_CONF_REG
/**
 * @enum bmx160_gyr_osr
 */
typedef enum{
    bmx160_gyr_bwp_osr_1    = (0x20),
    bmx160_gyr_bwp_osr_2    = (0x10),
    bmx160_gyr_bwp_osr_4    = (0x00)
} bmx160_gyr_osr;
/**
 * @enum bmx160_gyr_odr
 */
typedef enum{
    bmx160_gyr_odr_25       = (0x06),
    bmx160_gyr_odr_50       = (0x07),
    bmx160_gyr_odr_100      = (0x08),
    bmx160_gyr_odr_200      = (0x09),
    bmx160_gyr_odr_400      = (0x0A),
    bmx160_gyr_odr_800      = (0x0B),
    bmx160_gyr_odr_1600     = (0x0C),
    bmx160_gyr_odr_3200     = (0x0D)
} bmx160_gyr_odr;

/**
 * @enum bmx160_accel_range
 */
typedef enum{
    bmx160_accelRange_2g        = (0x03),
    bmx160_accelRange_4g        = (0x05),
    bmx160_accelRange_8g        = (0x08),
	bmx160_accelRange_16g       = (0x0C)
} bmx160_accel_range;

/**
 * @enum bmx160_gyro_range
 */
typedef enum{
    bmx160_gyroRange_2000dps        = (0x00),
    bmx160_gyroRange_1000dps        = (0x01),
    bmx160_gyroRange_500dps         = (0x02),
	bmx160_gyroRange_250dps         = (0x03),
	bmx160_gyroRange_125dps         = (0x04)
} bmx160_gyro_range;

typedef enum{
    bmx160_accel_normal_mode            = (0x11),
    bmx160_accel_lowpower_mode          = (0x12),
    bmx160_accel_suspend_mode           = (0x10),

    bmx160_gyro_suspend_mode           = (0x14),
    bmx160_gyro_normal_mode            = (0x15),
    bmx160_gyro_faststartup_mode       = (0x17),

    bmx160_magn_suspend_mode            = (0x18),
    bmx160_magn_normal_mode             = (0x19),
    bmx160_magn_lowpower_mode           = (0x1A),

    bmx160_start_foc                    = (0x03),
    bmx160_prog_nvm                     = (0xA0),
    bmx160_soft_reset                   = (0xB6)
}bmx160_commands_send;

//Mask values as peer data sheet
typedef enum{
    bmx160_foc_gyr_en                   = (0x40),

    bmx160_foc_acc_x_disabled           = (0x00),
    bmx160_foc_acc_x_p1g                = (0x10),
    bmx160_foc_acc_x_n1g                = (0x20),
    bmx160_foc_acc_x_0g                 = (0x30),

    bmx160_foc_acc_y_disabled           = (0x00),
    bmx160_foc_acc_y_p1g                = (0x04),
    bmx160_foc_acc_y_n1g                = (0x08),
    bmx160_foc_acc_y_0g                 = (0x0C),

    bmx160_foc_acc_z_disabled           = (0x00),
    bmx160_foc_acc_z_p1g                = (0x01),
    bmx160_foc_acc_z_n1g                = (0x02),
    bmx160_foc_acc_z_0g                 = (0x03),
}bmx160_foc_conf;

//Mask values before reading flags
typedef enum{
    bmx160_drdy_acc         = (0x80),       // Accelerometer data ready
    bmx160_drdy_gyr         = (0x40),       // Gyroscope data ready
    bmx160_nvm_rdy          = (0x20),       // NVM controller status ("0" NVM write in progress, "1" NVM ready to accept new write reg)
    bmx160_foc              = (0x08),       // FOC completed
}bmx160_status_flags;


//Mask values before reading flags
typedef enum{
    bmx160_drop_cmd_err         = (0x40),       // Dropped command to CMD(0x7E)
    bmx160_error_1              = (0x02),       // Error 1
    bmx160_error_2              = (0x04),       // Error 2
    bmx160_error_LPMintPFD      = (0x06),       // Low power mode and int uses pre-filtered data
    bmx160_ODRsNOmatch          = (0x0C),       // ODRs of enabled sensor in header-less mode do not match
    bmx160_PDFinLPM             = (0x0E),       // Prefiltered data are used in low power mode
    bmx160_fatal_err            = (0x01)        // Chip not operable
}bmx160_error_flags;


#endif /* BMX_160_REG_H_ */