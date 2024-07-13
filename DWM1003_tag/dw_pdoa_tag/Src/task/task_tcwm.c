/*
 * @file      task_tcwm.c
 *
 * @brief     TCWM implementation
 *
 * @author    Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#include "port_platform.h"
#include "dw_pdoa_tag_common.h"

#include <usb_uart_rx.h>
#include <task_tcwm.h>
#include <error.h>
#include <tcwm.h>

//-----------------------------------------------------------------------------

/*
 * @fn    tcwm_terminate_tasks
 * @brief Kill all tasks and timers related to tcwm if any
 *
 */
void tcwm_terminate_tasks(void)
{
    if(app.tcwmTask.Handle)
    {
        osMutexWait(app.tcwmTask.MutexId, osWaitForever);

        if(osThreadTerminate(app.tcwmTask.Handle) == osOK)
        {
            tcwm_process_terminate();

            osMutexDelete(app.tcwmTask.MutexId);
            app.tcwmTask.Handle = NULL;
        }
        else
        {
            error_handler(1, _ERR_Cannot_Delete_tcwmTask);
        }
    }

}


/*
 * @fn         StartTcwmTask
 * @brief      This starts the tcwm functionality.
 *
 */
void StartTcwmTask(void const *argument)
{
    dw_name_e    chip;

    chip = (*((uint32_t*)argument) == (uint32_t)DW_MASTER)?(DW_MASTER):(DW_SLAVE);

    port_disable_wake_init_dw();

    osMutexDef(tcwmMutex);
    app.tcwmTask.MutexId = osMutexCreate(osMutex(tcwmMutex));

    taskENTER_CRITICAL();

    tcwm_process_init(chip);    /**< the access to dwt_fn() shall be protected */

    taskEXIT_CRITICAL();

    while(1)
    {
        osMutexRelease(app.tcwmTask.MutexId);

       osDelay(1000 / portTICK_PERIOD_MS);
       osMutexWait(app.tcwmTask.MutexId, 0);
       tcwm_process_run();
    }
}
