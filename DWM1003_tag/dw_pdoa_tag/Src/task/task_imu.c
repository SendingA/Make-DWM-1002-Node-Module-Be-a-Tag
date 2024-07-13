/*
 * @file      task_imu.c
 * @brief
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#include <task_imu.h>
#include "cmd_fn.h"
#include <usb_uart_tx.h>

#define IMU_READ_DELAY_MS    (100)

imusensor_10dof_drv_t *p_drv;
float lsm6dsl_accel_bias_val[3];
float lsm6dsl_gyro_bias_val[3];
float lis2mdl_offset_val[3];
float lis2mdl_scale_val[3];
uint32_t lps22hb_baro_bias_val;

extern nrf_drv_twi_t _twi;
extern void sensor_lsm6dsl_set_bias_cfg();

extern const nrf_drv_spi_t imu_spi_inst;
//extern void spi2_three_wire_read(int en);

void stationary_imu_data_cb(stationary_imuData_t *imuData)
{
    static bool prev = false;
    twr_info_t  *pTwrInfo;
    pTwrInfo = getTwrInfoPtr();

    taskENTER_CRITICAL();

    pTwrInfo->acc.acc_x[0] = imuData->acc_x    &0xff;
    pTwrInfo->acc.acc_x[1] = (imuData->acc_x>>8) &0xff;
    pTwrInfo->acc.acc_y[0] = imuData->acc_y    &0xff;
    pTwrInfo->acc.acc_y[1] = (imuData->acc_y>>8) &0xff;
    pTwrInfo->acc.acc_z[0] = imuData->acc_z    &0xff;
    pTwrInfo->acc.acc_z[1] = (imuData->acc_z>>8) &0xff;


    taskEXIT_CRITICAL();

    if (!prev || (prev && !pTwrInfo->stationary_imu))
    {
        stationary_res_t    p;
        p.addr = AR2U16(pTwrInfo->env.tagAddr);
        p.flag = pTwrInfo->stationary_imu;

        p.acc_x = imuData->acc_x;
        p.acc_y = imuData->acc_y;
        p.acc_z = imuData->acc_z;


        //send_to_pc_stationary(&p);            // TBD
    }

    prev = pTwrInfo->stationary_imu;
}



/* @brief sets the stationary indicator flag in shared parameters
 *           (we can use other methods to inform other threads that
 *            IMU is stationary)
 * */
static bool run_imu(void)
{
    bool ret = FALSE;

    if (simple_stationary(p_drv, IMU_READ_DELAY_MS))
    {
        ret = TRUE;
#ifndef PDOA_V2_BOARD
        nrf_gpio_pin_write(LED_BLUE, 1);
#endif
    }
    else
    {
#ifndef PDOA_V2_BOARD
        nrf_gpio_pin_write(LED_BLUE, 0);
#endif
    }

    return (ret);
}


/*
 * */
static bool start_imu(void)
{
    bool    ret = TRUE, imu_spi_enable;
    static bool imusensor_calib_flag = 0;
    char *str = CMD_MALLOC(MAX_STR_SIZE);
    struct lsm6dsl_cfg lsm_cfg = LSM6DSL_CFG_DEFAULTS;

#if defined(PDOA_V2_BOARD) && defined(IMU_SPI_ENABLE)
    imu_spi_enable = 1;
#else
    imu_spi_enable = 0;
#endif

#ifndef PDOA_V2_BOARD
    //power ON IMU
    nrf_gpio_pin_write(LED_GREEN, 1);
#endif

    if(imu_spi_enable)
    {
      lsm_cfg.spi = &imu_spi_inst;
      lsm_cfg.spi_cs_pin = LSM6DSL_CS_PIN;
    }
    else
    {
      lsm_cfg.spi = 0;
      lsm_cfg.twi = &_twi;
      lsm_cfg.twi_addr = LSM6DSL_I2C_ADDR;
    }

    lsm6dsl_cfg_init(&lsm_cfg);


    //init IMU I2C and read IMU ID : this may fail sometimes
    p_drv = imusensor_10dof_driver_open(LSM_ACCEL);

    if(!p_drv)
    {
      error_handler(0, _ERR_IMU_INIT);
      ret = FALSE;
    }

    if ((imusensor_calib_flag == 0) && (ret == TRUE))
    {

      lsm6dsl_offsetBias(lsm6dsl_gyro_bias_val, lsm6dsl_accel_bias_val);

      imusensor_calib_flag = 1;
    } 

    return (ret);
}

/*
 * */
static void stop_imu(void)
{
    //deinit IMU I2C
    imusensor_10dof_driver_close();
    p_drv = NULL;

#ifndef PDOA_V2_BOARD
    //power OFF IMU
    nrf_gpio_pin_write(LED_GREEN, 0);
#endif
}


//-----------------------------------------------------------------------------

/* Bckgrnd Imu Service Task */
void ImuTask(void const * argument)
{
    bool        imu_enabled;
    twr_info_t  *pTwrInfo;

    do{
        osDelay(1000);
    }while(!(pTwrInfo = getTwrInfoPtr()));    //wait for initialization of pTwrInfo

    stop_imu();

    imu_enabled = FALSE;
    pTwrInfo->stationary_imu = FALSE;

    while(1)
    {
        osMutexRelease(app.imuTask.MutexId);

        osDelay(IMU_READ_DELAY_MS / portTICK_PERIOD_MS);
	
        osMutexWait(app.imuTask.MutexId, 0);

        if(pTwrInfo->env.imuOn)
        {
            if(imu_enabled)
            {
                pTwrInfo->stationary_imu = run_imu();
            }
            else
            {
                imu_enabled = start_imu();
            }
        }
        else
        {
            if(imu_enabled)
            {
                stop_imu();
                imu_enabled = FALSE;
                pTwrInfo->stationary_imu = FALSE;
            }
        }

        osThreadYield();
    }
}
