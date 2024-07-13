/*! ----------------------------------------------------------------------------
 * @file    main.c
 * @brief   This is the implementation of the PDOA Node on Nordic nRF52840 on FreeRTOS
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

#include "dw_pdoa_node_common.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_drv_wdt.h"

#include "nrf_drv_twi.h"

#include <config.h>

#include <node.h>
#include <tag_list.h>
#include <task_node.h>
#include <task_usb2spi.h>
#include <task_tcfm.h>
#include <task_tcwm.h>
#include <task_flush.h>
#include <task_test.h>

#define USB_DRV_UPDATE_MS    1 /*200*/

/* Private variables ---------------------------------------------------------*/

osThreadId defaultTaskHandle;
app_t app;    /**< All global variables are in the "app" structure */
int USBInitState = 0;

extern nrf_drv_wdt_channel_id m_channel_id;
extern int gRangingStart;

void DefaultTask(void const * argument);
void FlushTask(void const * argument);
void CtrlTask(void const * arg);

extern const nrf_drv_spi_t imu_spi_inst;
extern nrf_drv_twi_t _twi;
extern bool imu_spi_init_flag;
extern bool imu_twi_init_flag;

extern void stop_imu(void);

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    int devID = 0, delayCnt = 0, status = 0;
    ret_code_t err_code;

    gRangingStart = 0;

   /* With this change, Reset After Power Cycle is not required */
    nrf_gpio_cfg_input(UART_0_RX_PIN, NRF_GPIO_PIN_PULLUP);

    peripherals_init();

    port_init_dw_chips();

    memset(&app,0,sizeof(app));

    load_bssConfig();                 /**< load the RAM Configuration parameters from NVM block */
    app.pConfig = get_pbssConfig();   /**< app.pConfig pointed to the RAM config block */

    app.xStartTaskEvent = xEventGroupCreate(); /**< xStartTaskEvent indicates which tasks to be started */

#ifdef ENABLE_USB_PRINT
    pp_usb_init();
    app.usbState = USB_CONFIGURED;
#endif

    nrf_delay_ms(1000);        /**< small pause to startup */

    bsp_board_init(BSP_INIT_LEDS);

    reset_DW1000(DW_MASTER);      /**< This will sample DW1000 GPIO5/6 to correct SPI mode (0) */
    reset_DW1000(DW_SLAVE);       /**< This will sample DW1000 GPIO5/6 to correct SPI mode (0) */


    /* USER CODE END 2 */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */

      /* initialize inter-task communication mail queue for Node :
       *
       * The RxTask need to send the rxPckt to the CalcTask.
       *
       * Current code has an implementation where twrInfo is statically defined
       * and rxPcktPool_q is a part of FreeRtos Heap.
       *
       * Note, the debug accumulator & diagnostics readings are a part of
       * mail queue. Every rx_mail_t has a size of ~6kB.
       *
       * */
    osMailQDef(rxPcktPool_q, RX_MAIL_QUEUE_SIZE, rx_mail_t);
    app.rxPcktPool_q_id = osMailCreate(osMailQ(rxPcktPool_q), NULL);

    if(!app.rxPcktPool_q_id)
    {
        error_handler(1, _ERR_Cannot_Alloc_Mail);
    }

    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */
    //Some Comments: The core tasks are always running once the RTOS kernel has been started.
    /* Create the thread(s) */
    /* definition and creation of defaultTask */
    osThreadDef(defaultTask, DefaultTask, osPriorityNormal, 0, 256);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    /* USER CODE BEGIN RTOS_THREADS */

    /* FlushTask is always working and flushing the output buffer to uart/usb */

    osThreadDef(flushTask, FlushTask, osPriorityNormal, 0, 128);
    app.flushTask.Handle = osThreadCreate(osThread(flushTask), NULL);

    /* ctrlTask is always working serving rx from uart/usb */
    //2K for CTRL task: it needs a lot of memory: it uses mallocs(512), sscanf(212bytes)
    osThreadDef(ctrlTask, CtrlTask, osPriorityBelowNormal, 0, 512);
    app.ctrlTask.Handle = osThreadCreate(osThread(ctrlTask), NULL);

    if( !defaultTaskHandle | !app.flushTask.Handle | !app.ctrlTask.Handle )
    {
        error_handler(1, _Err_Create_Task_Bad);
    }

    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_QUEUES */

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

//communication to the user application
void command_stop_received(void)
{
    xEventGroupSetBits(app.xStartTaskEvent, Ev_Stop_All);
}

void DefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
    const EventBits_t bitsWaitForA = (Ev_Node_A_Task | Ev_Tcfm_A_Task | Ev_Tcwm_A_Task | Ev_Usb2spi_A_Task);
    const EventBits_t bitsWaitForB = (Ev_Node_B_Task | Ev_Tcfm_B_Task | Ev_Tcwm_B_Task | Ev_Usb2spi_B_Task);
    const EventBits_t bitsWaitForTag = Ev_Tag_Task;
    const EventBits_t bitsWaitForAny = ( bitsWaitForTag | bitsWaitForA | bitsWaitForB | Ev_Test_Task| Ev_Stop_All);
    //const EventBits_t bitsWaitForAny = ( bitsWaitForA | bitsWaitForB | Ev_Test_Task| Ev_Stop_All);
    EventBits_t    uxBits;
    uint32_t    chip;

    for(int i=0; i<6; i++)
    {
        nrf_gpio_pin_toggle(LED_STATIONARY);
        nrf_gpio_pin_toggle(LED_NODE);
        nrf_gpio_pin_toggle(LED_USB);  
        nrf_gpio_pin_toggle(LED_ERROR);
        nrf_delay_ms(250);
    }

    app.mode = mIDLE;
    // Add the parameter mamually 
    if (app.pConfig->s.autoStartEn == 1)
    {
        xEventGroupSetBits(app.xStartTaskEvent, Ev_Node_A_Task);    /**< activate Node task */
    }
    else if (app.pConfig->s.autoStartEn == 2)
    {
        xEventGroupSetBits(app.xStartTaskEvent, Ev_Node_B_Task);    /**< activate Node task */
    }   
    else if (app.pConfig->s.autoStartEn == 3)
    {
        xEventGroupSetBits(app.xStartTaskEvent, Ev_Tag_Task);       /**< activate Tag task */
    }

    /* Infinite loop: this is the helper task, which starts appropriate mode */
    while(1)
    {
        uxBits = xEventGroupWaitBits(app.xStartTaskEvent,
                                     bitsWaitForAny,
                                     pdTRUE, pdFALSE,
                                     USB_DRV_UPDATE_MS/portTICK_PERIOD_MS );

        nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh

        uxBits &= bitsWaitForAny;

        chip = (uint32_t)(uxBits & bitsWaitForB)?(DW_SLAVE):(DW_MASTER);

        if(uxBits)
        {
            /* Turn LEDs off on restart of top-level application */
            nrf_gpio_pin_write(LED_STATIONARY, 0);
            nrf_gpio_pin_write(LED_NODE, 0);
            nrf_gpio_pin_write(LED_ERROR, 0);

            app.lastErrorCode = _NO_ERR;

            /*   need to switch off DW1000's RX and IRQ before killing tasks */
            if(app.mode != mIDLE)
            {
                taskENTER_CRITICAL();
                disable_dw1000_irq();
                set_dw_spi_slow_rate(DW_SLAVE);
                dwt_forcetrxoff();
                set_dw_spi_slow_rate(DW_MASTER);
                dwt_forcetrxoff();
                taskEXIT_CRITICAL();
            }

            /* Event to start/stop task received */
            /* 1. free the resources: kill all user threads and timers */
            node_terminate_tasks();
            usb2spi_terminate_tasks();
            tcfm_terminate_tasks();
            tcwm_terminate_tasks();
            test_terminate_tasks();
            reset_FlushTask();

            app.lastErrorCode = _NO_ERR;
        }

        nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh

        osThreadYield(); //force switch of context that the Idle task can free resources

        taskENTER_CRITICAL();

        /* 2. Start appropriate RTOS top-level application or run a usb_vbus_driver() if a dummy loop */
        switch (uxBits)
        {
            case Ev_Tag_Task:
                app.mode = mTWR;
                /* execute helper function to setup sub-tasks for Tag process */
                tag_helper(&chip);
                break;  
            case Ev_Node_A_Task:
            case Ev_Node_B_Task:
                app.mode = mTWR;
                node_helper(&chip); /* call Node helper function which will setup sub-tasks for Node process */
                break;

            case Ev_Usb2spi_A_Task:
            case Ev_Usb2spi_B_Task:
                /* Setup a Usb2Spi task : 8K of stack is required to this task */
                app.mode = mUSB2SPI;

                osThreadDef(u2sTask, StartUsb2SpiTask, osPriorityNormal, 0, 256);
                app.usb2spiTask.Handle = osThreadCreate(osThread(u2sTask), &chip);
                if(!app.usb2spiTask.Handle)
                {
                    error_handler(1, _Err_Create_Task_Bad);
                }
                break;

            case Ev_Tcfm_A_Task:
            case Ev_Tcfm_B_Task:
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
            case Ev_Tcwm_B_Task:
                /* Setup a TCWM task */
                app.mode = mTCWM;

                osThreadDef(tcwmTask, StartTcwmTask, osPriorityNormal, 0, 256);
                app.tcwmTask.Handle = osThreadCreate(osThread(tcwmTask), &chip);
                if(!app.tcwmTask.Handle)
                {
                    error_handler(1, _Err_Create_Task_Bad);
                }
                break;

            case Ev_Test_Task:
                /* Setup a Production Test Function task */
                app.mode = mTESTFN;

                osThreadDef(testTask, StartTestTask, osPriorityNormal, 0, 256);
                app.testTask.Handle = osThreadCreate(osThread(testTask), NULL);
                if(!app.testTask.Handle)
                {
                    error_handler(1, _Err_Create_Task_Bad);
                }
                break;

            case Ev_Stop_All:
                app.mode = mIDLE;
				
                if(imu_twi_init_flag || imu_spi_init_flag)
                {
                  stop_imu();
                }
                
                if(imu_spi_init_flag)
                {
                  nrf_drv_spi_uninit(&imu_spi_inst);
                  imu_spi_init_flag = 0;
                }

                if(imu_twi_init_flag)
                {
                  nrf_drv_twi_uninit(&_twi);
                  imu_twi_init_flag = 0;
                }
                
                break;


            default:
                nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh
                break;
        }

        taskEXIT_CRITICAL();    //ready to switch to a created task
        osThreadYield();
    }

  /* USER CODE END 5 */
}

/** @} */


