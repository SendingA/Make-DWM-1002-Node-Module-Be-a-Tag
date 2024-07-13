/*
 * @file     task_usb2spi.c
 * @brief    usb2spi implementation
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#include "port_platform.h"
#include "dw_pdoa_node_common.h"

#include <usb_uart_rx.h>
#include <task_usb2spi.h>
#include <error.h>
#include <usb2spi.h>

//-----------------------------------------------------------------------------

/* @brief
 * Kill all tasks and timers related to usb2spi if any
 *
 * */
void usb2spi_terminate_tasks(void)
{
    if(app.usb2spiTask.Handle)
    {
        osMutexWait(app.usb2spiTask.MutexId, osWaitForever);
        taskENTER_CRITICAL();
		
        if(osThreadTerminate(app.usb2spiTask.Handle) == osOK)
        {
            usb2spi_process_terminate();

            if(app.usb2spiTask.MutexId)
            {
                osMutexDelete(app.usb2spiTask.MutexId);
            }

            app.usb2spiTask.Handle = NULL;
            app.usb2spiTask.MutexId = NULL;
        }
        else
        {
            error_handler(1, _ERR_Cannot_Delete_usb2spiTask);
        }
		
        taskEXIT_CRITICAL();
    }

}


/* @fn         StartUsb2SpiTask
 * @brief     this starts the usb2spi functionality.
 *
 *             Note: Previous tasks which can call shared resources must be killed.
 *            This task needs the RAM size of at least usb2spi_t
 *
 * */
void StartUsb2SpiTask(void const *argument)
{
    error_e        tmp;
    dw_name_e    chip;

    chip = (*((uint32_t*)argument) == (uint32_t)DW_MASTER)?(DW_MASTER):(DW_SLAVE);

    port_disable_wake_init_dw();

    osMutexDef(u2sMutex);
    app.usb2spiTask.MutexId = osMutexCreate(osMutex(u2sMutex));

    tmp = usb2spi_process_init(chip);

    if(tmp != _NO_ERR)
    {
        error_handler(1, tmp);
    }

    while(1)
    {
        osMutexRelease(app.usb2spiTask.MutexId);

        osSignalWait(app.usb2spiTask.Signal , osWaitForever);

        osMutexWait(app.usb2spiTask.MutexId, 0);

        usb2spi_process_run();    //app.local_buff has a Usb2Spi protocol sequence
    }
}

