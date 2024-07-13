/*! ----------------------------------------------------------------------------
 * @file    imusensor_10dof_interface.h
 * @brief   Interface file for 10 DOF Sensor
 *
 * @attention
 *
 * Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#ifndef __IMUSENSOR_10DOF_INTERFACE_H__
#define __IMUSENSOR_10DOF_INTERFACE_H__

#include "port_platform.h"
#include "nrf_drv_twi.h"
#include "lsm6dsl.h"
#include "lis2mdl.h"
#include "lps22hb.h"
#include "lps22hb_reg.h"

/* LSM6DSl WHO_AM_I Responses */
#define WHO_AM_I_ACC        0x6A
#define WHO_AM_I_MAG   	    0x40
#define WHO_AM_I_BARO	    0xB1

#define IMU_NO_DATA      0xDEAD

/// gyro_scale defines the possible full-scale ranges of the gyroscope:
typedef enum {
    G_SCALE_250DPS = 0x0,      // 00: +/- 250 degrees per second
    G_SCALE_500DPS = 0x1,      // 01: +/- 500 dps
    G_SCALE_1000DPS = 0x2,     // 10: +/- 1000 dps
    G_SCALE_2000DPS= 0x3,      // 11: +/- 2000 dps
    G_SCALE_MASK   = 0x3
} gyro_scale_e;

/// gyro_oder defines all possible output data rates of the gyroscope:
typedef enum {                  
    G_POWER_DOWN,   // Power-down mode (0x0)
    G_ODR_12_5,       // 12.5 Hz (0x1)
    G_ODR_26,         // 26 Hz (0x2)
    G_ODR_52,         // 52 Hz (0x3)
    G_ODR_104,        // 104 Hz (0x4)
    G_ODR_208,        // 208 Hz (0x5)
    G_ODR_416,        // 416 Hz (0x6)
    G_ODR_833,        // 833 Hz (0x7)
    G_ODR_1660,       // 1660 Hz (0x8)
    G_ODR_3330,       // 3330 Hz (0x9)
    G_ODR_6660,       // 6660 Hz (0xA)
} gyro_odr_e;

// accel_scale defines all possible FSR's of the accelerometer:
typedef enum {
    A_SCALE_2G  = 0x00, // 00: +/- 2g
    A_SCALE_16G = 0x01, // 01: +/- 16g
    A_SCALE_4G  = 0x02, // 10: +/- 4g
    A_SCALE_8G  = 0x03, // 11: +/- 8g
    A_SCALE_MASK= 0x03
} accel_scale_e;

/// accel_oder defines all possible output data rates of the accelerometer:
typedef enum {
    A_POWER_DOWN,   // Power-down mode (0x0)
    A_ODR_12_5,       // 12.5 Hz (0x1)
    A_ODR_26,         // 26 Hz (0x2)
    A_ODR_52,         // 52 Hz (0x3)
    A_ODR_104,        // 104 Hz (0x4)
    A_ODR_208,        // 208 Hz (0x5)
    A_ODR_416,        // 416 Hz (0x6)
    A_ODR_833,        // 833 Hz (0x7)
    A_ODR_1660,       // 1660 Hz (0x8)
    A_ODR_3330,       // 3330 Hz (0x9)
    A_ODR_6660,       // 6660 Hz (0xA)
} accel_odr_e;

// specifies the driver working mode
typedef enum {
    LSM_GYRO    = 0x01,
    LSM_ACCEL   = 0x02,
    LIS_MAG     = 0x04,
    LPS_BARO    = 0x08
}imu_func_e;

// driver structure, returned to the application
// on calling of imusensor_10dof_lsm6dsl_driver_open(imu_func_e);
// imusensor_10dof_lsm6dsl_driver_open() is also calling an init "constructor", which read the status
//
typedef struct {

        const void *self;
        imu_func_e  enable;
        uint16_t    lsm6dsl_devID;
        uint16_t    lis2mdl_devID;
        uint16_t    lps22hb_devID;

        uint16_t (* lsm6dsl_init)(gyro_scale_e, accel_scale_e, gyro_odr_e, accel_odr_e);
        void     (* lsm6dsl_deinit)(void);

        uint16_t (* lis2mdl_init)();
        void     (* lis2mdl_deinit)(void);

        uint16_t (* lps22hb_init)();
        void     (* lps22hb_deinit)(void);

        void (* readGyro)(void);
        void (* readAccel)(void);
        void (* readMag)(void);
        void (* readBaro)(void);

        void (* setGyroScale)(gyro_scale_e);
        void (* setAccelScale)(accel_scale_e);

        void (* setGyroODR)(gyro_odr_e);
        void (* setAccelODR)(accel_odr_e);

        int (* accelReady)(void);
        int (* gyroReady)(void);
        int (* magReady)(void);
        int (* baroReady)(void);

        struct {
            void (*write)(uint8_t addr, uint8_t *buf, uint16_t len);
            void (*read) (uint8_t addr, uint8_t *buf, uint16_t len);
        } imusensor_10dof_lsm6dsl;

        struct {
            void (*write)(uint8_t addr, uint8_t buf);
            void (*read) (uint8_t addr, uint8_t *buf, uint8_t len);
        } imusensor_10dof_lis2mdl;

        struct {
            void (*write)(uint8_t addr, uint8_t buf);
            void (*read) (uint8_t addr, uint8_t *buf);
        } imusensor_10dof_lps22hb;

        // We'll store the gyro, accel readings in following
        // variables. Each sensor gets three variables -- one for each
        // axis.
        // Call readGyro(), readAccel(), readMag(), readBaro(),before using variables.
        //
        int16_t gx_raw, gy_raw, gz_raw; // x, y, and z axis readings of the gyroscope
        int16_t ax_raw, ay_raw, az_raw; // x, y, and z axis readings of the accelerometer
        int16_t mx_raw, my_raw, mz_raw; // x, y, and z axis readings of the magnetometer
        uint32_t b_pres_raw;            // Pressure readings of the Barometer
        int32_t b_temp_raw;             // Temperature readings of the Barometer

        // floating-point values of scaled data in real-world units
        float gx, gy, gz; // in "dps"
        float ax, ay, az; // in "m/s^2"
        float mx, my, mz; // in "uT"
        float b_alt;      // in "mm"
        
}imusensor_10dof_drv_t;

/* exported driver instance */
extern imusensor_10dof_drv_t *drv;


/* exported function prototypes */
void                imusensor_10dof_driver_close(void);
imusensor_10dof_drv_t *imusensor_10dof_driver_open(int imu_functions_enable);

/* function prototypes */

/* @fn imusensor_10dof_lsm6dsl_init()
 * @brief
 * Initialize the gyro, accelerometer
 *  This will set up the scale and output rate of each sensor. It'll also
 *  "turn on" every sensor and every axis of every sensor, if its configured in the
 *  "enable" variable of the driver.
 *
 *  If sensor is not enabled, software will turn it off.
 *
 *  Input:
 *   - gScl = The scale of the gyroscope. This should be a gyro_scale value. 
 *   - aScl = The scale of the accelerometer. Should be a accel_scale value.
 *   - gODR = Output data rate of the gyroscope. gyro_odr value. 
 *   - aODR = Output data rate of the accelerometer. accel_odr value.
 *  Output: The function will return an unsigned 16-bit value. The most-sig
 *       bytes of the output are the WHO_AM_I reading of the accel. 
 *  All parameters have a defaulted value, so you can call just "begin()".
 *  Default values are FSR's of: +/- 2000DPS, 16Gs; ODRs of 6660 Hz for   
 *  accelerometer and gyroscope
 *  Use the return value of this function to verify communication.
 */
uint16_t imusensor_10dof_lsm6dsl_init(gyro_scale_e, accel_scale_e, gyro_odr_e, accel_odr_e);

/* @fn imusensor_10dof_lsm6dsl_deinit()
 * @brief
 *      instruct accel, gyro to go to power-down mode
 */
void imusensor_10dof_lsm6dsl_deinit();

/* @fn imusensor_10dof_lis2mdl_init()
 * @brief
 * Initialize the Magnetometer
 *  This will set up the output rate of sensor and Interrupts. 
 *  Output: The function will return an unsigned 16-bit value. The most-sig
 *       bytes of the output are the WHO_AM_I reading of the Magnetometer. 
 *  Use the return value of this function to verify communication.
 */
uint16_t imusensor_10dof_lis2mdl_init();

/* @fn imusensor_10dof_lis2mdl_deinit()
 * @brief
 *      instruct Magnetometer to go to power-down mode
 */
void imusensor_10dof_lis2mdl_deinit();

/* @fn imusensor_10dof_lps22hb_init()
 * @brief
 * Initialize the Barometer
 *  Output: The function will return an unsigned 16-bit value. The most-sig
 *       bytes of the output are the WHO_AM_I reading of the Barometer. 
 *  Use the return value of this function to verify communication.
 */
uint16_t imusensor_10dof_lps22hb_init();

/* @fn imusensor_10dof_lps22hb_deinit()
 * @brief
 *      instruct Barometer to go to power-down mode
 */
void imusensor_10dof_lps22hb_deinit();

/* @fn  readGyro()
 * @brief
 * Read the gyroscope output.
 * This function will read all six gyroscope output registers.
 * The readings are stored in the gx_raw, gy_raw, and gz_raw variables.
 */
void readGyro();

/* @fn readAccel()
 * @brief Read the accelerometer output registers.
 *
 * This function will read all six accelerometer output registers.
 * The readings are stored in the ax_raw, ay_raw, and az_raw variables.
 */
void readAccel();

/* @fn readMag()
 * @brief Read the magnetometer output registers.
 *
 * This function will read all six magnetometer output registers.
 * The readings are stored in the mx_raw, my_raw, and mz_raw variables.
 */
void readMag();

/* @fn readBaro()
 * @brief Read the Barometer output registers.
 *
 * This function will read pressure data
 * The readings are stored in the b_pres_raw, b_temp_raw variable.
 */
void readBaro();

/* @fn setGyroScale()
 * @brief Sets the full-scale range of the gyroscope.
 *  This function can be called to set the scale of the gyroscope to
 *  245, 500, or 2000 degrees per second.
 * @param
 *   gScl = The desired gyroscope scale. Must be one of three possible
 *       values from the gyro_scale_e.
 */
void setGyroScale(gyro_scale_e);

/* @fn setAccelScale()
 * @brief Set the full-scale range of the accelerometer.
 *  This function can be called to set the scale of the accelerometer to
 *  2, 4, 8, or 16 g's.
 * @param
 *   aScl = The desired accelerometer scale. Must be one of five possible
 *       values from the accel_scale_e.
 */
void setAccelScale(accel_scale_e);

/* @fn setGyroODR()
 * @brief   Set the output data rate  of the gyroscope
 * @param
 *   gRate = The desired output rate of the gyro.
 *       Must be a value from the gyro_odr_e.
 */
void setGyroODR(gyro_odr_e);

/* @fn setAccelODR()
 * @brief   Set the output data rate of the accelerometer
 * @param
 *   aRate = The desired output rate of the accel.
 *       Must be a value from the accel_odr_e.
 */
void setAccelODR(accel_odr_e);

/* @fn accelReady()
 * @brief Accelerometer data ready indicator.
 * @output 0 when accelerometer data are not ready yet
 */
int accelReady();

/* @fn gyroReady()
 * @brief Gyro data ready indicator.
 * @output 0 when gyro data are not ready yet
 */
int gyroReady();

/* @fn magReady()
 * @brief Magnetometer data ready indicator.
 * @output 0 when Magnetometer data are not ready yet
 */
int magReady();

/* @fn baroReady()
 * @brief Barometer data ready indicator.
 * @output 0 when Barometer data are not ready yet
 */
int baroReady();

#endif // __IMUSENSOR_10DOF_INTERFACE_H__
