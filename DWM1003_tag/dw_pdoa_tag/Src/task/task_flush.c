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

/* Used to regulate frequency of FlushTask output */
static  uint32_t flush_period = FLUSH_PERIOD_DEFAULT_MS;

/*
 * @brief   This thread is
 *          flushing report buffer on demand or every USB_FLUSH_MS ms
 * */
void FlushTask(void const * argument)
{
    set_FlushPeriod(FLUSH_PERIOD_DEFAULT_MS);
    while(1)
    {
        osSignalWait(app.flushTask.Signal, flush_period);

        flush_report_buf();
        osThreadYield();
    }
}

void reset_FlushTask(void)
{
    if(app.flushTask.Handle)
    {
        taskENTER_CRITICAL();
        reset_report_buf();
        taskEXIT_CRITICAL();
    }
}

void set_FlushPeriod(int ms)
{
    if(app.flushTask.Handle)
    {
        taskENTER_CRITICAL();
        flush_period = ms/portTICK_PERIOD_MS;
        taskEXIT_CRITICAL();
    }
}

