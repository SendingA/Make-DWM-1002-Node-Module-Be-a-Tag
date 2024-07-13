/*! ----------------------------------------------------------------------------
 * @file
 * @brief   stationary function
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

#include "nrf_drv_spi.h"
#include "stationary.h"
#include <task_imu.h>
#include "dw_pdoa_tag_common.h"

#include "usb_uart_tx.h"


/* Accelerometer floating point "1g" will be normalized to the number below */
#define ACCEL_NORMALIZED_RESOLUTION (1000)

int simple_stationary(imusensor_10dof_drv_t *drv, uint16_t delay_ms)
{
    static int      stationary = 0;
    static uint16_t consequence_moving_detection = 0;
    static uint16_t consequence_stationary_detection = 0;
    static int16_t prev_acc_x = 0, prev_acc_y = 0, prev_acc_z = 0;
    int16_t ax, ay, az, result;
    stationary_imuData_t imuData;

    if (drv)
    {
        /* Stationary state detection. Proceed if new accelerometer data is available. */
      if (drv->accelReady())
      {

        drv->readAccel();

        imuData.acc_x = (int16_t)(ACCEL_NORMALIZED_RESOLUTION* drv->ax);
        imuData.acc_y = (int16_t)(ACCEL_NORMALIZED_RESOLUTION* drv->ay);
        imuData.acc_z = (int16_t)(ACCEL_NORMALIZED_RESOLUTION* drv->az);

        /* Send normalized to values to the callback*/
        stationary_imu_data_cb(&imuData);

        /* Check if any axis of acceleration has changed "significantly" */
        if ((abs(imuData.acc_x - prev_acc_x) + (abs(imuData.acc_y - prev_acc_y) + (abs(imuData.acc_z - prev_acc_z)))) > app.pConfig->s.acc_threshold )  
        {
            if (stationary)
            {
                /* Check if acceleration values have been evolving for long enough to consider state is moving */
                consequence_moving_detection++;
                if (consequence_moving_detection > (app.pConfig->s.acc_moving_ms/delay_ms))
                {
                    /* Acceleration values have changed significantly for several readings now, we are moving. */
                    stationary = 0;
                    consequence_stationary_detection = 0;
                }
            }
            else
            {
                /* We are moving: reduce the "stationary" counter, if stationary state was detected during movement */
                consequence_stationary_detection = (consequence_stationary_detection > 1) ? (consequence_stationary_detection-2) : (0);
            }
        }
        else
        {
            if (stationary)
            {
                /* We are stationary: reduce the "moving" counter, if movement state was detected during stationary */
                consequence_moving_detection = (consequence_moving_detection > 1) ? (consequence_moving_detection-2) : (0);
            }
            else
            {
                /* Check if acceleration values have been steady for long enough to consider state is not moving */
                consequence_stationary_detection++;
                if (consequence_stationary_detection > (app.pConfig->s.acc_stationary_ms/delay_ms))
                {
                    /* Acceleration values have been steady for several readings now, we are stationary. */
                    stationary = 1;
                    consequence_moving_detection = 0;
                }
            }
        }

        /* Update previous values. */
        prev_acc_x = imuData.acc_x;
        prev_acc_y = imuData.acc_y;
        prev_acc_z = imuData.acc_z;

      }
    }

    return stationary;
}

