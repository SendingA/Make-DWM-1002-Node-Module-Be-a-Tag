/*! ----------------------------------------------------------------------------
 * @file    dw_pdoa_tag_common.h
 * @brief   Defines PDOA Tag related Common Macros, structures, function definitions
 *
 * @author  Decawave 
 *
 * @attention
 *
 * Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#ifndef __DW_PDOA_TAG_COMMON__H__
#define __DW_PDOA_TAG_COMMON__H__

#ifdef __cplusplus
 extern "C" {
#endif
#include <nrfx_rtc.h>
#include <nrfx_log.h>

#include "prs/nrfx_prs.h"
#include "nrf_drv_spi.h"

#include "default_config.h"
#include "error.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "event_groups.h"
#include "cmsis_os.h"
#define PDOA_V2_BOARD

#define IMU_SPI_ENABLE

#define DW1000_CLK_Pin     16
#define DW1000_MISO_Pin    18
#define DW1000_MOSI_Pin    20
#define DW1000_CS_A_Pin    17

#define DW1000_IRQ_A_Pin   19
#define DW1000_RST_A_Pin   24

#define DW1000_WUP_A_Pin   17

#define LPS22HB_BR_CS_Pin     11

#define LSM6DSL_I2C_ADDR      0b1101011
#define LIS2MDL_I2C_ADDR      0b0011110

#ifndef PDOA_V2_BOARD
#define LPS22HB_I2C_ADDR      0b1011100
#else
#define LPS22HB_I2C_ADDR      0b1011101
#endif

/* IMU Sensor Pins */
#define LSM6DSL_CS_PIN     (30)   
#define LIS2MDL_CS_PIN     (27)   
#define LPS22HB_CS_PIN     (31)

#define LSM6DSL_LPS22HB_SDO_PIN     (12)

#define LED_RED                  22
#define LED_BLUE                 31
#define LED_GREEN                30
#define LED_RGB_RED_D12          14    // Not working on 1003

#define LED_ERROR               LED_4

#define UART_TX_BUF_SIZE        0x100   /**< Write buffer for UART transmission, shall be 1<<X */
#define UART_RX_BUF_SIZE        0x100   /**< Read buffer for UART reception, shall be 1<<X */
#define USB_RX_BUF_SIZE         0x100   /**< Read buffer for USB reception, shall be 1<<X */
#define COM_RX_BUF_SIZE         USB_RX_BUF_SIZE /**< Communication RX buffer size */

#define NRF52_FLASH_CONFIG_ADDR     0x70000
#define NRF_DRV_USBD_EPSIZE         0x40

/** @brief Macro for forwarding the new implementation. */
#define nrf_drv_rtc_reinit              nrfx_rtc_reinit          /* Added for pdoa_tag application */

#if (COM_RX_BUF_SIZE < 64)
#error "COM_RX_BUF_SIZE should be longer than CDC_DATA_FS_MAX_PACKET_SIZE"
#endif


/* System mode of operation. used to
 *
 * 1. indicate in which mode of operation system is running
 * 2. configure the access rights to command handler in control mode
 * */
typedef enum {
    mANY = 0,    /**< Used only for Commands: indicates the command can be executed in any modes below */
    mIDLE,        /**< IDLE mode */
    mTWR,        /**< TWR (active) mode */
    mUSB2SPI,    /**< USB2SPI mode */
    mTCWM,        /**< Transmit Continuous Wave Mode mode */
    mTCFM        /**< Transmit Continuous Frame Mode mode */
}mode_e;

/* events to start/stop tasks : event group */
enum{
    Ev_Tag_Task         = 0x08,
    Ev_Tcfm_A_Task      = 0x40,
    Ev_Tcwm_A_Task      = 0x100,
    Ev_Usb2spi_A_Task	= 0x400,
    Ev_Stop_All 		= 0x1000
};


 /* Application tasks handles & corresponded signals structure */
 typedef struct
 {
     osThreadId Handle;     /* Task’s handler */
     osMutexId  MutexId;    /* Task’s mutex */
     int32_t    Signal;     /* Task’s signal */
 }task_signal_t;

/* Application's global parameters structure */
typedef struct
{
    param_block_t   *pConfig;       /**< Current configuration */
    mode_e          mode;           /**< Information: handle the current "mode" of operation */
    int             lastErrorCode;  /**< Saves the error code in the error_handler() function */
    int             maxMsgLen;      /**< See the longest string size to optimize the MAX_STR_SIZE */
    volatile int    DwCanSleep;     /**< When 1, app can put DW1000 chip to deep sleep mode */

    /* USB / CTRL */
    enum {
        USB_DISCONNECTED,
        USB_PLUGGED,
        USB_CONNECTED,
        USB_CONFIGURED,
        USB_UNPLUGGED
    }
    usbState;                                        /**< USB connect state */

    struct
    {
    	uint8_t	   tmpRx;
        int16_t    head;
        int16_t    tail;
        uint8_t    buf[UART_RX_BUF_SIZE];
    }uartRx;                                        /**< circular buffer RX from USART */

    struct
    {
        int16_t    head;
        int16_t    tail;
        uint8_t    buf[USB_RX_BUF_SIZE];
    }usbRx;                                         /**< circular buffer RX from USB */

    uint16_t        local_buff_length;              /**< from usb_uart_rx parser to application */
    uint8_t         local_buff[COM_RX_BUF_SIZE];    /**< for RX from USB/USART */

    /* Tasks section */
    EventGroupHandle_t xStartTaskEvent;     /**< This event group will pass activation to tasks to start */
	
    //defaultTask is always running and is not accepting signals

    task_signal_t    ctrlTask;          /* usb/uart RX: Control task */
    task_signal_t    flushTask;         /* usb/uart TX: Flush task */

    /* app task for TAG mode */
    task_signal_t   imuTask;            /* Tag/Node */
    task_signal_t   rxTask;             /* Tag/Node */
    task_signal_t   blinkTask;          /* Tag only */
    task_signal_t   blinkTmr;           /* Tag only: SW timer */
    task_signal_t   pollTask;           /* Tag only */

    /* app tasks for special modes */
    task_signal_t   usb2spiTask;        /* usb2spi mode */
    task_signal_t   tcfmTask;           /* tcfm mode */
    task_signal_t   tcwmTask;           /* tcwm mode */

}__attribute__((packed))
app_t;

extern app_t app;

void error_handler(int block, error_e err);
int8_t usb_data_receive();
nrfx_err_t nrfx_rtc_reinit(nrfx_rtc_t const * const  p_instance,
                           nrfx_rtc_config_t const * p_config,
                           nrfx_rtc_handler_t        handler);

//nrfx_err_t nrfx_spim_init2(nrfx_spim_t const * const  p_instance,
//                          nrfx_spim_config_t const * p_config,
//                          nrfx_spim_evt_handler_t    handler,
//                          void *                     p_context);

//ret_code_t nrf_drv_spi_init2(nrf_drv_spi_t const * const p_instance,
//                             nrf_drv_spi_config_t const * p_config,
//                             nrf_drv_spi_evt_handler_t    handler,
//                             void *                       p_context);


#ifdef __cplusplus
}
#endif

#endif /* __DW_PDOA_TAG_COMMON__H__ */
