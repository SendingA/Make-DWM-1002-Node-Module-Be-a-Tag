/*! ----------------------------------------------------------------------------
 * @file    main.c
 * @brief   This is the implementation of the PDOA Tag on Nordic nRF52840 on FreeRTOS
 *
 * @author Decawave Software
 *
 * @attention Copyright 2017 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 *
 */
/* Includes ------------------------------------------------------------------*/

#include "port_platform.h"
#include "deca_device_api.h"
#include "deca_regs.h"

#include "dw_pdoa_tag_common.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_drv_wdt.h"

#include <config.h>

#include <tag.h>
#include <task_tag.h>
#include <task_flush.h>
#include <task_tcfm.h>
#include <task_tcwm.h>

#define USB_DRV_UPDATE_MS    200

osThreadId defaultTaskHandle;
app_t app;    /**< All global variables are in the "app" structure */

extern nrf_drv_wdt_channel_id m_channel_id;

void DefaultTask(void const * argument);
void FlushTask(void const * argument);
void CtrlTask(void const * arg);

int USBInitState = 0;

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    int devID = 0, delayCnt = 0, status = 0;
    ret_code_t err_code;
    
    peripherals_init();

    port_init_dw_chips();
    memset(&app,0,sizeof(app));

    load_bssConfig();                 /**< load the RAM Configuration parameters from NVM block */
    app.pConfig = get_pbssConfig();   /**< app.pConfig pointed to the RAM config block */
    app.xStartTaskEvent = xEventGroupCreate(); /**< xStartTaskEvent indicates which tasks to be started */

    nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh

    nrf_delay_ms(1000);
    nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh

    nrf_delay_ms(1000);        /**< small pause to startup */

    set_dw_spi_slow_rate(DW_MASTER);

    /* USER CODE END 2 */

    /* USER CODE BEGIN RTOS_MUTEX */


    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* USER CODE END RTOS_TIMERS */

    /* Create the thread(s) */
    /* definition and creation of defaultTask */
    osThreadDef(defaultTask, DefaultTask, osPriorityLow, 0, 256);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
	
    /* USER CODE BEGIN RTOS_THREADS */
    port_wakeup_dw1000();

    /* Select correct SPI mode (0) */
    reset_DW1000();

    /* USER CODE BEGIN RTOS_THREADS */

    /* FlushTask is always working and flushing the output buffer to uart/usb : 96 + 512bytes of stack */

    osThreadDef(flushTask, FlushTask, osPriorityNormal, 0, 128);
    app.flushTask.Handle = osThreadCreate(osThread(flushTask), NULL);

    /* ctrlTask is always working serving rx from uart/usb it uses 96 + 1024 bytes of stack  */
    //1024B for CTRL task: it needs a lot of memory: it uses malloc(127), sscanf(212bytes)
    osThreadDef(ctrlTask, CtrlTask, osPriorityBelowNormal, 0, 512);
    app.ctrlTask.Handle = osThreadCreate(osThread(ctrlTask), NULL);

   if(!app.flushTask.Handle | !app.ctrlTask.Handle )
    {
        error_handler(1, _Err_Create_Task_Bad);
    } 

    if( !defaultTaskHandle )
    {
        error_handler(1, _Err_Create_Task_Bad);
    }

    /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    }
    /* USER CODE END 3 */

}

void DefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
    const EventBits_t bitsWaitForA = (Ev_Tag_Task | Ev_Tcfm_A_Task | Ev_Tcwm_A_Task);
    const EventBits_t bitsWaitForAny = (bitsWaitForA | Ev_Stop_All);

    EventBits_t    uxBits;
    uint32_t    chip;

  /* Default task is blinking 3 times on power on and then lets other tasks to start */

    nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh

    for(int i=0; i<6; i++)
    {
#ifndef PDOA_V2_BOARD
        nrf_gpio_pin_toggle(LED_BLUE);
#endif
        nrf_gpio_pin_toggle(LED_RED);
        osDelay(250);
        nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh
    }

    app.mode = mIDLE;

    if (app.pConfig->s.autoStartEn == 1)
    {
        xEventGroupSetBits(app.xStartTaskEvent, Ev_Tag_Task);    /**< activate Tag task */
    }

     osDelay(USB_DRV_UPDATE_MS/portTICK_PERIOD_MS);

    /* Infinite loop: this is the helper task, which starts appropriate mode */

    while(1)
    {
        uxBits = xEventGroupWaitBits(app.xStartTaskEvent,
                                     bitsWaitForAny,
                                     pdTRUE, pdFALSE,
                                     USB_DRV_UPDATE_MS/portTICK_PERIOD_MS );

        nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh

        uxBits &= bitsWaitForAny;

        chip = (DW_MASTER);
        /* Event to start/stop task received */
        /* 1. free the resources: kill all user threads and timers */
        if(uxBits)
        {
            /* Turn LEDs off on restart of top-level application */
#ifndef PDOA_V2_BOARD
        nrf_gpio_pin_write(LED_BLUE, 0);
#endif
  	    nrf_gpio_pin_write(LED_RED, 0);

            app.lastErrorCode = _NO_ERR;
            app.DwCanSleep = 0;

            set_FlushPeriod(FLUSH_PERIOD_DEFAULT_MS);

            {
                taskENTER_CRITICAL();
                disable_dw1000_irq();
                port_wakeup_dw1000();
                set_dw_spi_slow_rate(DW_MASTER);
                dwt_forcetrxoff();
                taskEXIT_CRITICAL();
            }

            tag_terminate_tasks();
            nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh
            tcfm_terminate_tasks();

            nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh
            tcwm_terminate_tasks();
        }

        nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh
        osThreadYield();                    //instruct kernel to force switch context that the Idle task can free resources
        osDelay( 100/portTICK_PERIOD_MS );    //Idle task is freeing resources here
        nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh

        taskENTER_CRITICAL();

        /* 2. Start appropriate RTOS task */
        switch (uxBits)
        {
            case Ev_Tag_Task:
                app.mode = mTWR;
                /* execute helper function to setup sub-tasks for Tag process */
                tag_helper(&chip);
                break;

            case Ev_Tcfm_A_Task:
                /* Setup a TCFM task */
                app.mode = mTCFM;

                osThreadDef(tcfmTask, StartTcfmTask, osPriorityNormal, 0, 256);
                app.tcfmTask.Handle = osThreadCreate(osThread(tcfmTask), &chip);
                if(!app.tcfmTask.Handle)
                {
                    error_handler(1, _Err_Create_Task_Bad);
                }
                break;

            case Ev_Tcwm_A_Task:
                /* Setup a TCWM task */
                app.mode = mTCWM;

                osThreadDef(tcwmTask, StartTcwmTask, osPriorityNormal, 0, 256);
                app.tcwmTask.Handle = osThreadCreate(osThread(tcwmTask), &chip);
                if(!app.tcwmTask.Handle)
                {
                    error_handler(1, _Err_Create_Task_Bad);
                }
                break;

            case Ev_Stop_All:
                /* put DW1000 to the Sleep.
                 */ 
                dwt_entersleep();
                app.mode = mIDLE;
                break;


            default:
                nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh
                break;
        }

        taskEXIT_CRITICAL();    //ready to switch to a created task
        osThreadYield();

#ifdef ENABLE_USB_PRINT
        if(USBInitState == 0)
        {
          pp_usb_init();
          app.usbState = USB_CONFIGURED;
          USBInitState = 1;
        }
#endif

    }

  /* USER CODE END 5 */
}

/** @} */


