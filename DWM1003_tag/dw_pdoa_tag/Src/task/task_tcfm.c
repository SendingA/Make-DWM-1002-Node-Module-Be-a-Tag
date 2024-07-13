/*
 * @file      task_tcfm.c
 *
 * @brief     Task for Test Continuous Frame Mode (TCFM)
 *
 * @author    Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#include "dw_pdoa_tag_common.h"
#include "port_platform.h"

#include <usb_uart_rx.h>
#include <task_tcfm.h>
#include <error.h>
#include <tcfm.h>

//-----------------------------------------------------------------------------

/*
 * @fn    tcfm_terminate_tasks
 * @brief Kill all tasks related to TCFM
 *
 */
void tcfm_terminate_tasks(void)
{
    if(app.tcfmTask.Handle)
    {
        osMutexWait(app.tcfmTask.MutexId, osWaitForever);

        if(osThreadTerminate(app.tcfmTask.Handle) == osOK)
        {
            tcfm_process_terminate();

            osMutexDelete(app.tcfmTask.MutexId);
            app.tcfmTask.Handle = NULL;
        }
        else
        {
            error_handler(1, _ERR_Cannot_Delete_tcfmTask);
        }  
        
    }

}


/*
 * @fn         StartTcfmTask
 * @brief      This function starts the TCFM functionality.
 *
 */
void StartTcfmTask(void const *argument)
{
    dw_name_e    chip;

    chip = (*((uint32_t*)argument) == (uint32_t)DW_MASTER)?(DW_MASTER):(DW_SLAVE);

    port_disable_wake_init_dw();

    osMutexDef(tcfmMutex);
    app.tcfmTask.MutexId = osMutexCreate(osMutex(tcfmMutex));

    taskENTER_CRITICAL();

    tcfm_process_init(chip);    /**< the access to dwt_fn() shall be protected */

    taskEXIT_CRITICAL();

    while(1)
    {
        osMutexRelease(app.tcfmTask.MutexId);

	osDelay(1000 / portTICK_PERIOD_MS);
        osMutexWait(app.tcfmTask.MutexId, 0);
        tcfm_process_run();
    }
}
