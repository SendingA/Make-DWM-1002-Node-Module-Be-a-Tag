/*! ----------------------------------------------------------------------------
 * @file    imusensor_10dof_interface.c
 * @brief   Defines Interface functions of Accel, Gyro, Magnetometer and Barometer sensors 
 *
 *  usage:
 *  1) drv = imusensor_10dof_driver_open - this will return pointer to driver instance struct, if success. See imusensor_10dof_drv_t.
 *  2) drv->lsm6dsl_devID should be (WHO_AM_I_ACC)
 *  3) drv->lis2mdl_devID should be (WHO_AM_I_MAG)
 *  4) drv->lps22hb_devID should be (WHO_AM_I_BARO)
 *  5) access to accelerometer driver functions:
 *    if(drv->accelReady()) {
 *     drv->readAccel();
 *    }
 *  results are in drv->ax,ay,az variables, see imusensor_10dof_drv_t
 *  6) access to gyroscope driver functions:
 *    if(drv->gyroReady()) {
 *     drv->readGyro();
 *    }
 *  results are in drv->gx,gy,gz variables, see imusensor_10dof_drv_t
 *  7) access to magnetometer driver functions:
 *    if(drv->magReady()) {
 *     drv->readMag();
 *    }
 *  results are in drv->mx,my,mz variables, see imusensor_10dof_drv_t
 *  8) access to barometer driver functions:
 *    if(drv->baroReady()) {
 *     drv->baroAccel();
 *    }
 *  results are in drv->aalt variables, see imusensor_10dof_drv_t
 *
 * @attention
 *
 * Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

#include "imusensor_10dof_interface.h"
#include "dw_pdoa_tag_common.h"
#include <math.h>

imusensor_10dof_drv_t * drv = NULL;

extern float lsm6dsl_accel_bias_val[3];
extern float lsm6dsl_gyro_bias_val[3];
extern float lis2mdl_offset_val[3];
extern float lis2mdl_scale_val[3];
extern uint32_t lps22hb_baro_bias_val;

#define ACCEL_NORMALIZED_RESOLUTION (1000)

//------------------------------------------------------------------------------
// Local variables

/* We have 10DOF IMU sensor on the board */
imusensor_10dof_drv_t imusensor_10dof_drv =
{
        .self           = &imusensor_10dof_drv,
        .enable         = (LSM_ACCEL | LSM_GYRO | LIS_MAG | LPS_BARO),

        .lsm6dsl_init   = &imusensor_10dof_lsm6dsl_init,
        .lsm6dsl_deinit = &imusensor_10dof_lsm6dsl_deinit,

        .lis2mdl_init   = &imusensor_10dof_lis2mdl_init,
        .lis2mdl_deinit = &imusensor_10dof_lis2mdl_deinit,

        .lps22hb_init   = &imusensor_10dof_lps22hb_init,
        .lps22hb_deinit = &imusensor_10dof_lps22hb_deinit,

        .readGyro       = &readGyro,
        .readAccel      = &readAccel,
        .readMag        = &readMag,
        .readBaro       = &readBaro,

        .setGyroScale   = &setGyroScale,
        .setAccelScale  = &setAccelScale,

        .setGyroODR     = &setGyroODR,
        .setAccelODR    = &setAccelODR,

        .accelReady     = &accelReady,
        .gyroReady      = &gyroReady,
        .magReady       = &magReady,
        .baroReady      = &baroReady,

        .imusensor_10dof_lsm6dsl.read    = &sensor_10dof_lsm6dsl_read,
        .imusensor_10dof_lsm6dsl.write   = &sensor_10dof_lsm6dsl_write,  

        .imusensor_10dof_lis2mdl.read    = &sensor_10dof_lis2mdl_read,
        .imusensor_10dof_lis2mdl.write   = &sensor_10dof_lis2mdl_write,  

        .imusensor_10dof_lps22hb.read    = &sensor_10dof_lps22hb_read,
        .imusensor_10dof_lps22hb.write   = &sensor_10dof_lps22hb_write,  

        .gx_raw = IMU_NO_DATA, .gy_raw = IMU_NO_DATA, .gz_raw = IMU_NO_DATA,
        .ax_raw = IMU_NO_DATA, .ay_raw = IMU_NO_DATA, .az_raw = IMU_NO_DATA,
        .mx_raw = IMU_NO_DATA, .my_raw = IMU_NO_DATA, .mz_raw = IMU_NO_DATA,
        .b_pres_raw = IMU_NO_DATA, .b_temp_raw = IMU_NO_DATA,
        
        .gx = IMU_NO_DATA, .gy = IMU_NO_DATA, .gz = IMU_NO_DATA,
        .ax = IMU_NO_DATA, .ay = IMU_NO_DATA, .az = IMU_NO_DATA,
        .mx = IMU_NO_DATA, .my = IMU_NO_DATA, .mz = IMU_NO_DATA,
        .b_alt = IMU_NO_DATA,
};


/*  gRes, aRes store the current resolution for each sensor.
 *  Units of these values would be DPS (or g's or Gs's) per ADC tick.
 *  This value is calculated as (scaleX) / (2^15). 
 */
static float gRes, aRes;

//------------------------------------------------------------------------------
// Interface: "Constructor" and "destructor" for driver

/* @fn  imusensor_10dof_driver_open()
 * @brief  Shall be called on beginning
 *         Opens driver and put all sensors to power down mode
 *  returns the pointer to driver instance imusensor_10dof_drv_t
 */
imusensor_10dof_drv_t *imusensor_10dof_driver_open(int imu_functions_enable)
{
    drv = &imusensor_10dof_drv;
    drv->enable = imu_functions_enable;
    drv->lsm6dsl_devID = drv->lsm6dsl_init(G_SCALE_2000DPS, A_SCALE_16G, G_ODR_6660, A_ODR_6660);

    if(drv->lsm6dsl_devID != WHO_AM_I_ACC)
    {
        error_handler(0, _ERR_IMU_INIT);
        drv = NULL;
    }

    return drv;
};


/* @fn  imusensor_10dof_driver_close()
 * @brief   close driver and put all sensors to power down mode
 */
void imusensor_10dof_driver_close(void)
{
    if(drv)
    {
        drv->lsm6dsl_deinit();
        drv->lis2mdl_deinit();
        drv->lps22hb_deinit();
    }
}

//------------------------------------------------------------------------------
// Internal driver functions

/* @fn  initAccel()
 * @brief   Sets up the accelerometer to begin reading.
 *  This function steps through all three accelerometer control registers.
 */
static void initAccel(accel_scale_e tmp)
{
    uint8_t temp;

    temp = 0x04;
    drv->imusensor_10dof_lsm6dsl.write(LSM6DSM_CTRL3_C, &temp, 1);
    drv->imusensor_10dof_lsm6dsl.read(LSM6DSM_CTRL3_C, &temp, 1);

    // enable block update (bit 6 = 1), auto-increment registers (bit 2 = 1)
    temp = temp | 0x40 | 0x04;
    drv->imusensor_10dof_lsm6dsl.write(LSM6DSM_CTRL3_C, &temp, 1);

    // by default, interrupts active HIGH, push pull, little endian data 
    // (can be changed by writing to bits 5, 4, and 1, resp to above register)
    drv->imusensor_10dof_lsm6dsl.read(LSM6DSM_CTRL3_C, &temp, 1);

    // enable accel LP2 (bit 7 = 1), set LP2 tp ODR/9 (bit 6 = 1), enable input_composite (bit 3) for low noise    
    temp = 0x80 | 0x40 | 0x08;
    drv->imusensor_10dof_lsm6dsl.write(LSM6DSM_CTRL8_XL, &temp, 1);

    // interrupt handling
    temp = 0x80;
    drv->imusensor_10dof_lsm6dsl.write(LSM6DSM_DRDY_PULSE_CFG, &temp, 1);  // latch interrupt until data read

    temp = 0x3;
    drv->imusensor_10dof_lsm6dsl.write(LSM6DSM_INT1_CTRL, &temp, 1);  // enable accel/gyro interrupts on INT1
}

/* @fn  initMag()
 * @brief   Sets up the Magnetometer to begin reading.
 *  This function Intialize the Magnetometer
 */
static void initMag()
{
    uint8_t temp;

    // enable temperature compensation (bit 7 == 1), continuous mode (bits 0:1 == 00)
    temp = 0x80 | MODR<<2;
    drv->imusensor_10dof_lis2mdl.write(LIS2MDL_CFG_REG_A, temp);

    // enable low pass filter (bit 0 == 1), set to ODR/4
    temp = 0x01;
    drv->imusensor_10dof_lis2mdl.write(LIS2MDL_CFG_REG_B, temp);

    // enable data ready on interrupt pin (bit 0 == 1), enable block data read (bit 4 == 1)
    temp = 0x01 | 0x10;
    drv->imusensor_10dof_lis2mdl.write(LIS2MDL_CFG_REG_C, temp);
}

/* @fn  initBaro()
 * @brief   Sets up the Barometer to begin reading.
 *  This function Intialize the Barometer
 */
static void initBaro()
{
    drv->imusensor_10dof_lps22hb.write(LPS22HB_CTRL_REG2, LPS22HB_REG2_RESET); 
    drv->imusensor_10dof_lps22hb.write(LPS22HB_CTRL_REG3, LPS22HB_REG3_DRDY);

    //nrf_gpio_cfg_input(LPS22HB_BR_CS_Pin, GPIO_PIN_CNF_PULL_Pullup);  // TBD

}
/* @fn deInitGyro()
 * @brief   Sets gyro to power down.
 */
static void deInitGyro()
{
    uint8_t temp = 0x0;

    drv->imusensor_10dof_lsm6dsl.read(LSM6DSM_CTRL7_G, &temp, 1);
    temp |= 0x80;
    drv->imusensor_10dof_lsm6dsl.write(LSM6DSM_CTRL7_G, &temp, 1); //Gyro power down
}


/* @fn deInitAccel()
 * @brief   Sets accelerometer to power down.
 */
static void deInitAccel()
{
    uint8_t temp = 0x0;

    drv->imusensor_10dof_lsm6dsl.read(LSM6DSM_CTRL6_C, &temp, 1);
    temp |= 0x10;
    drv->imusensor_10dof_lsm6dsl.write(LSM6DSM_CTRL6_C, &temp, 1); //Accel power down

}

/* @fn deInitMag()
 * @brief   Sets Magnetometer to power down.
 */
static void deInitMag()
{
    uint8_t temp = 0x0;

    drv->imusensor_10dof_lis2mdl.read(LIS2MDL_CFG_REG_A, &temp, 1);
    temp |= 0x10;
    drv->imusensor_10dof_lis2mdl.write(LIS2MDL_CFG_REG_A, temp); //Mag power down
}

/* @fn deInitBaro()
 * @brief   Sets Barometer to power down.
 */
static void deInitBaro()
{
    uint8_t temp = 0x1;

    drv->imusensor_10dof_lps22hb.write(LPS22HB_RES_CONF, temp);  // Barometer Power down Mode
}


/* @fn  calcgRes()
 * @brief    Calculate the resolution of the gyroscope.
 *  This function will set the value of the mRes variable according to scale value.
 */
static void calcgRes(gyro_scale_e tmp)
{
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), 2000 DPS (11).
    switch (tmp)
    {
        case G_SCALE_250DPS:
            gRes = 250.0 / 32768.0;
            break;
        case G_SCALE_500DPS:
            gRes = 500.0 / 32768.0;
            break;
        case G_SCALE_1000DPS:
            gRes = 1000.0 / 32768.0;
            break;
        case G_SCALE_2000DPS:
            gRes = 2000.0 / 32768.0;
            break;
    }
}

/* @fn  calcaRes()
 * @brief    Calculate the resolution of the accelerometer.
 *  This function will set the value of the mRes variable according to scale value.
 */
static void calcaRes(accel_scale_e tmp)
{
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 g (000), 4g (001), 6g (010) 8g (011), 16g (100).
    switch (tmp)
    {
        case A_SCALE_2G:
            aRes = 2.0 / 32768.0;
            break;
        case A_SCALE_4G:
            aRes = 4.0 / 32768.0;
            break;
        case A_SCALE_8G:
            aRes = 8.0 / 32768.0;
            break;
        case A_SCALE_16G:
            aRes = 16.0 / 32768.0;
            break;
    }
}

//-----------------------------------------------------------------------------


//------------------------------------------------------------------------------
// Exported driver functions

uint16_t imusensor_10dof_lsm6dsl_init(  gyro_scale_e    gScl,
                                        accel_scale_e   aScl,
                                        gyro_odr_e      gODR,
                                        accel_odr_e     aODR)
{
    uint16_t ret;
    uint8_t value;

    calcaRes(aScl); // Calculate g / ADC tick, stored in aRes variable
    
    // To verify communication, we need to read from the WHO_AM_I registers of each device.
    drv->imusensor_10dof_lsm6dsl.read(LSM6DSM_WHO_AM_I, &value, 1);
    ret = value;            // Read the accel
    
    if(drv->enable & LSM_GYRO)
    {
        // Gyro
       //initGyro(gScl);     // init the gyro. Setting up interrupts, etc.
        setGyroODR(gODR);   // Set the gyro output data rate 
        setGyroScale(gScl); // Set the gyro range
    }
    else
    {
        deInitGyro();
    }

    if(drv->enable & LSM_ACCEL)
    {
        // Accelerometer
        initAccel(aScl);    // init all axes of the accel. Set up interrupts, etc.
        setAccelODR(aODR);  // Set the accel data rate.
        setAccelScale(aScl);// Set the accel range.
    }
    else
    {
        deInitAccel();
    }

    return (ret);          //should be (WHO_AM_I_ACC)
}

void imusensor_10dof_lsm6dsl_deinit()
{
    if(drv)
    {
        deInitGyro();
        deInitAccel();
    }
}

uint16_t imusensor_10dof_lis2mdl_init()
{
    uint16_t ret;
    uint8_t value;

    // To verify communication, we need to read from the WHO_AM_I registers of each device.
    drv->imusensor_10dof_lis2mdl.read(LIS2MDL_WHO_AM_I, &value, 1);
    ret = value;            // Read the Magnetometer 

    if(drv->enable & LIS_MAG)
    {
        // Magnetometer
        initMag();    // initialize Magnetometer Sensor. Set up interrupts, etc.
    }
    else
    {
        deInitMag();
    }

    return (ret);          //should be (WHO_AM_I_MAG)
}

void imusensor_10dof_lis2mdl_deinit()
{
    if(drv)
    {
        deInitMag();
    }
}


uint16_t imusensor_10dof_lps22hb_init()
{
    uint16_t ret;
    uint8_t value;

    // To verify communication, we need to read from the WHO_AM_I registers of each device.
    drv->imusensor_10dof_lps22hb.read(LPS22HB_WHO_AM_I, &value);
    ret = value;            // Read the Barometer

    if(drv->enable & LPS_BARO)
    {
        // Barometer
        initBaro();    // initialize Barometer Sensor 
    }
    else
    {
        deInitBaro();
    }

    return (ret);          //should be (WHO_AM_I_BARO)
}

void imusensor_10dof_lps22hb_deinit()
{
    if(drv)
    {
        deInitBaro();
    }
}

void readGyro()
{
    if(drv->enable & LSM_GYRO)
    {
        int16_t rawdata[7];
        drv->imusensor_10dof_lsm6dsl.read(LSM6DSM_OUT_TEMP_L, (uint8_t *)rawdata, 14);

        drv->gx_raw = rawdata[1];
        drv->gy_raw = rawdata[2];
        drv->gz_raw = rawdata[3];

        drv->gx_raw -= lsm6dsl_gyro_bias_val[0];
        drv->gy_raw -= lsm6dsl_gyro_bias_val[1];
        drv->gz_raw -= lsm6dsl_gyro_bias_val[2];

        drv->gx = drv->gx_raw * gRes;
        drv->gy = drv->gy_raw * gRes;
        drv->gz = drv->gz_raw * gRes;
    }
    else
    {
        drv->gx_raw = drv->gy_raw = drv->gz_raw = IMU_NO_DATA;
        drv->gx = drv->gy = drv->gz = IMU_NO_DATA;
    }
}

void readAccel()
{
    if(drv->enable & LSM_ACCEL)
    {
        int16_t rawdata[7];
        drv->imusensor_10dof_lsm6dsl.read(LSM6DSM_OUT_TEMP_L, (uint8_t *)rawdata, 14);

        drv->ax_raw = rawdata[4];
        drv->ay_raw = rawdata[5];
        drv->az_raw = rawdata[6];

        drv->ax_raw -= lsm6dsl_accel_bias_val[0];
        drv->ay_raw -= lsm6dsl_accel_bias_val[1];
        drv->az_raw -= lsm6dsl_accel_bias_val[2];

        // convert to m/s^2
        drv->ax = drv->ax_raw * ((aRes) * 9.81);
        drv->ay = drv->ay_raw * ((aRes) * 9.81);
        drv->az = drv->az_raw * ((aRes) * 9.81);
    }
    else
    {
        drv->ax_raw = drv->ay_raw = drv->az_raw = IMU_NO_DATA;
        drv->ax = drv->ay = drv->az = IMU_NO_DATA;
    }
}

void readMag()
{
    if(drv->enable & LIS_MAG)
    {
        uint8_t rawdata[6];       // x/y/z mag register data stored here
        drv->imusensor_10dof_lis2mdl.read((0x80 | LIS2MDL_OUTX_L_REG), &rawdata[0], 8);     // Read the 6 raw data registers into data array

        drv->mx_raw = ((int16_t)rawdata[1] << 8) | rawdata[0] ;		 // Turn the MSB and LSB into a signed 16-bit value
        drv->my_raw = ((int16_t)rawdata[3] << 8) | rawdata[2] ;	
        drv->mz_raw = ((int16_t)rawdata[5] << 8) | rawdata[4] ; 

        drv->mx_raw -= lis2mdl_offset_val[0];
        drv->my_raw -= lis2mdl_offset_val[1];
        drv->mz_raw -= lis2mdl_offset_val[2];

        drv->mx_raw *= lis2mdl_scale_val[0];
        drv->my_raw *= lis2mdl_scale_val[1];
        drv->mz_raw *= lis2mdl_scale_val[2];

        drv->mx = drv->mx_raw;
        drv->my = drv->my_raw;
        drv->mz = drv->mz_raw;
    }
    else
    {
        drv->mx_raw = drv->my_raw = drv->mz_raw = IMU_NO_DATA;
        drv->mx = drv->my = drv->mz = IMU_NO_DATA;
    }
}

void readBaro()
{
    if(drv->enable & LPS_BARO)
    {
        uint8_t rawdataH, rawdataL, rawdataXL;       // Barometer register data stored here
        const float alt_f = 0.1;
        static float altitude = 0.0f;

        drv->imusensor_10dof_lps22hb.read(LPS22HB_TEMP_OUT_H, &rawdataH);     
        drv->imusensor_10dof_lps22hb.read(LPS22HB_TEMP_OUT_L, &rawdataL);     

        drv->b_temp_raw = ((int32_t)rawdataH << 8) | (int32_t)rawdataL;

        drv->imusensor_10dof_lps22hb.read(LPS22HB_PRESS_OUT_L, &rawdataL);     
        drv->imusensor_10dof_lps22hb.read(LPS22HB_PRESS_OUT_H, &rawdataH);     
        drv->imusensor_10dof_lps22hb.read(LPS22HB_PRESS_OUT_XL, &rawdataXL);     

        drv->b_pres_raw = (((uint32_t)rawdataH << 16) | ((uint32_t)rawdataL << 8) |
                           ((uint32_t)rawdataXL)) * 
                          100 / 4096;

        drv->b_pres_raw = drv->b_pres_raw - lps22hb_baro_bias_val;

        float alt = 44330.8-4946.54*powf(drv->b_pres_raw,0.190263);

        altitude = (altitude == 0.0f) ? alt : altitude*(1-alt_f) + alt_f*alt;
    
        drv->b_alt = (((int)altitude) * 1000)  +  (int)(1000*fabs(altitude - (int)altitude));
    }
    else
    {
        drv->b_temp_raw = drv->b_pres_raw = IMU_NO_DATA;
        drv->b_alt = IMU_NO_DATA;
    }

    drv->imusensor_10dof_lps22hb.write(LPS22HB_CTRL_REG2, LPS22HB_REG2_ONESHOT);     
}
int gyroReady()
{
    int ret=0;

    if(drv->enable & LSM_GYRO)
    {
        uint8_t temp;

        drv->imusensor_10dof_lsm6dsl.read(LSM6DSM_STATUS_REG, &temp, 1);
        ret = (temp & 0x02)>>1;
    }
    return ret;
}

int accelReady()
{
    int ret=0;

    if(drv->enable & LSM_ACCEL)
    {
        uint8_t temp;

        drv->imusensor_10dof_lsm6dsl.read(LSM6DSM_STATUS_REG, &temp, 1);
        ret = temp & 0x01;
    }
    
    return ret;
}

int magReady()
{
    int ret=0;

    if(drv->enable & LIS_MAG)
    {
        uint8_t temp;

        drv->imusensor_10dof_lis2mdl.read(LIS2MDL_STATUS_REG, &temp, 1);
        ret = (temp & 0x08) >> 3;
    }
    
    return ret;
}

int baroReady()
{
    int ret=0;

    if(drv->enable & LPS_BARO)
    {
        uint8_t temp;

        drv->imusensor_10dof_lps22hb.read(LPS22HB_STATUS, &temp);
        ret = (temp & 0x01);
    }
    
    return ret;
}

void setGyroScale(gyro_scale_e gScl)
{
    uint8_t temp;

    drv->imusensor_10dof_lsm6dsl.read(LSM6DSM_CTRL2_G, &temp, 1);
    temp = temp | (gScl << 2);
    drv->imusensor_10dof_lsm6dsl.write(LSM6DSM_CTRL2_G, &temp, 1);        // Set the gyro scale

    calcgRes(gScl); // calculate a new gRes
}

void setAccelScale(accel_scale_e aScl)
{
    uint8_t temp;

    drv->imusensor_10dof_lsm6dsl.read(LSM6DSM_CTRL1_XL, &temp, 1);
    temp = temp | (aScl << 2);
    drv->imusensor_10dof_lsm6dsl.write(LSM6DSM_CTRL1_XL, &temp, 1);       // Set the Accel scale
    
    calcaRes(aScl); // calculate a new aRes
}

void setGyroODR(gyro_odr_e gRate)
{
    uint8_t temp;

    drv->imusensor_10dof_lsm6dsl.read(LSM6DSM_CTRL2_G, &temp, 1);
    temp = temp | (gRate << 4);
    drv->imusensor_10dof_lsm6dsl.write(LSM6DSM_CTRL2_G, &temp, 1);
}

void setAccelODR(accel_odr_e aRate)
{
    uint8_t temp;

    drv->imusensor_10dof_lsm6dsl.read(LSM6DSM_CTRL1_XL, &temp, 1);
    temp = temp | (aRate << 4);
    drv->imusensor_10dof_lsm6dsl.write(LSM6DSM_CTRL1_XL, &temp, 1);
}
