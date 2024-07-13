/*
 * @file  task_ctrl.c
 * @brief
  *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */
#include <cmd.h>
#include <usb_uart_rx.h>
#include <usb_uart_tx.h>

#include <cmsis_os.h>
#include "dw_pdoa_tag_common.h"

/* @fn      CtrlTask
 * @brief   this is a core Control task (RX).
 *          this task is activated on the startup
 *          there 2 sources of control data: Uart and Usb.
 *
 * */
void CtrlTask(void const * arg)
{
    usb_data_e    res;

    while(1)
    {
        osSignalWait(app.ctrlTask.Signal , osWaitForever);    /* signal from USB/UART that some data has been received */

        taskENTER_CRITICAL();

        /* mutex if usb2spiTask using the app.local_buf*/
        res = usb_uart_rx();    /**< processes usb/uart input :
                                     copy the input to the app.local_buff[ local_buff_length ]
                                     for future processing */
        taskEXIT_CRITICAL();

        if (res == COMMAND_READY)
        {
            int len = MIN((app.local_buff_length-1), (sizeof(app.local_buff)-1));
            app.local_buff[len+1] = 0;
            command_parser((char *)app.local_buff);            //parse and execute the command
        }
        else if (res == DATA_READY)
        {
            if(app.usb2spiTask.Handle)
            {
                osSignalSet(app.usb2spiTask.Handle, app.usb2spiTask.Signal);
            }
        }
    }
}
