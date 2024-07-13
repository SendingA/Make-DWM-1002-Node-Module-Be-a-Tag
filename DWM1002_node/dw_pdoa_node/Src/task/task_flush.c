/*
 * @file  task_flush.c
 * @brief
  *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */
#include "task_flush.h"
#include "usb_uart_tx.h"

#define USB_FLUSH_MS    5

/*
 * @brief this thread is
 *        flushing report buffer on demand or every USB_FLUSH_MS ms
 * */
void FlushTask(void const * argument)
{
    while(1)
    {
        osSignalWait(app.flushTask.Signal, USB_FLUSH_MS / portTICK_PERIOD_MS);
        
        if(app.usbState == USB_CONFIGURED) {
          flush_report_buf();
        }
    }
}

void reset_FlushTask(void)
{
    if(app.flushTask.Handle)
    {
        taskENTER_CRITICAL();
        flush_report_buf();
        reset_report_buf();
        taskEXIT_CRITICAL();
    }
}
