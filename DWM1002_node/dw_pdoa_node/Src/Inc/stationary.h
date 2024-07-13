/*! ----------------------------------------------------------------------------
 * @file    stationary.h
 *
 * @brief   header for stationary function
 *
 *          This function uses the reading of acceleration to determine if the device is stationary or moving, by monitoring the acceleration
 *          values evolution.
 *
 * @attention
 *
 * Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#ifndef __STATIONARY_H_
#define __STATIONARY_H_ 1

#include <stdint.h>
#include <stdlib.h>
#include "nrf_drv_twi.h"
#include "lsm6dsl.h"
#include "imusensor_10dof_interface.h"

typedef struct {
    int16_t  acc_x;
    int16_t  acc_y;
    int16_t  acc_z;
}stationary_imuData_t;


/*! ----------------------------------------------------------------------------
 * @fn simple_stationary()
 *
 * @brief
 *      This function uses the acceleration values on all axes to determine
 *      if the device is moving or stationary.
 *
 * @param
 * *drv - pointer to the driver instance
 *          driver shall provide these functions:
 *          drv->accelReady() - IMU has accelerometer data ready 
 *          drv->readAccel()  - IMU read accelerometer data
 *          and these output values
 *          drv-> ax ay az - accelerometer - normalized to values of X,Y,Z axis
 
 * delay_ms - the delay, at which this called from the upper application, ms
 *
 * output parameters none
 *
 * returns 1 if the device is stationary, 0 if the device is moving.
 */
int simple_stationary(imusensor_10dof_drv_t *drv, uint16_t delay_ms);


#endif /* __STATIONARY_H_ */
