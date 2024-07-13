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
#include "dw_pdoa_node_common.h"

#define IMU_READ_DELAY_MS       (500)

imusensor_10dof_drv_t *p_drv;
float lsm6dsl_accel_bias_val[3];
float lsm6dsl_gyro_bias_val[3];
float lis2mdl_offset_val[3];
float lis2mdl_scale_val[3];
uint32_t lps22hb_baro_bias_val;

extern nrf_drv_twi_t _twi;
extern void sensor_lsm6dsl_set_bias_cfg();

extern const nrf_drv_spi_t imu_spi_inst;
extern void spi2_three_wire_read(int en);
extern void imusensor_twi_init(void);
extern void imusensor_spi_init(void);

static bool stationary = false;

//-----------------------------------------------------------------------------
// extern functions to report output data
extern void send_to_pc_stationary(stationary_res_t *p);


//-----------------------------------------------------------------------------
// Implementation

/*
 * @brief callback for normalized values from accelerometer sensor
 * Reports to the PC values on moving every time it is called.
 * If stationary was detected, sends last stationary data only once.
 *
 * */
void stationary_imu_data_cb(stationary_imuData_t *imuData)
{
    static bool prev = false;
    twr_info_t    *pTwrInfo;
    pTwrInfo  = getTwrInfoPtr();

    if (!prev || (prev && !stationary))
    {
        stationary_res_t    p;
        p.addr = pTwrInfo->eui16;
        p.flag = stationary; //bit 0 is stationary

        p.acc_x = imuData->acc_x;
        p.acc_y = imuData->acc_y;
        p.acc_z = imuData->acc_z;

        send_to_pc_stationary(&p);
    }
    prev = stationary;
}

/* @brief sets the stationary indicator flag
 * */
static bool run_imu(void)
{
    bool ret = FALSE;

    if (simple_stationary(p_drv, IMU_READ_DELAY_MS))
    {
        ret = TRUE;
        nrf_gpio_pin_write(LED_STATIONARY, 1);
    }
    else
    {
        nrf_gpio_pin_write(LED_STATIONARY, 0);
    }

    return (ret);
}


/* @brief
 * Power On IMU and initialize it
 * */
static bool start_imu(void)
{
    bool    ret = TRUE, imu_spi_enable;
    static bool imusensor_calib_flag = 0;
    char *str = CMD_MALLOC(MAX_STR_SIZE);
    struct lsm6dsl_cfg lsm_cfg = LSM6DSL_CFG_DEFAULTS;

//power ON IMU : IMU sensor is always powered on the Node
//init IMU SPI and read IMU ID

#if defined(PDOA_V2_BOARD) && defined(IMU_SPI_ENABLE)
    imu_spi_enable = 1;
#else
    imu_spi_enable = 0;
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
      sprintf(str, "\n\n\r IMU Calibration Started");
      port_tx_msg((uint8_t*)str, strlen(str));
      osDelay(100);

      lsm6dsl_offsetBias(lsm6dsl_gyro_bias_val, lsm6dsl_accel_bias_val);

      sprintf(str, "\n\n\r IMU Calibration Completed");
      port_tx_msg((uint8_t*)str, strlen(str));
      osDelay(100);

      imusensor_calib_flag = 1;
    }

    return (ret);
}


/* @brief
 * deInit IMU and power it off
 * */
void stop_imu(void)
{
    //deinit IMU SPI
    imusensor_10dof_driver_close();
    p_drv = NULL;
    stationary = false;
    //power OFF IMU : IMU sensor is always powered on the Node
}


//-----------------------------------------------------------------------------

/* @brief IMU Service Task
 *
 * */
void ImuTask(void const * argument)
{
    bool    imu_enabled;

#if defined(PDOA_V2_BOARD) && defined(IMU_SPI_ENABLE)
      imusensor_spi_init();
#else
      imusensor_twi_init();
#endif

    imu_enabled = FALSE;
    stationary = FALSE;

    osMutexDef(imuMutex);
    app.imuTask.MutexId = osMutexCreate(osMutex(imuMutex));

    while(1)
    {
        osMutexRelease(app.imuTask.MutexId);

        osDelay(IMU_READ_DELAY_MS / portTICK_PERIOD_MS);

        osMutexWait(app.imuTask.MutexId, osWaitForever);

        if(imu_enabled)
        {
            stationary = run_imu();
        }
        else
        {
            imu_enabled = start_imu();
        }

        osThreadYield();
    }
}
