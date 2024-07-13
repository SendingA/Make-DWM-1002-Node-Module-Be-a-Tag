/*! ------------------------------------------------------------------------------------------------------------------
 * @file    dw_pdoa_node_common.h
 * @brief   Defines PDoA Node related Common Macros, structures, function definitions
 *
 * @attention
 *
 * Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Deepa Gopinath
 */

#ifndef __DW_PDOA_NODE_COMMON__H__
#define __DW_PDOA_NODE_COMMON__H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <nrfx_rtc.h>
#include <nrfx_log.h>

#include "prs/nrfx_prs.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_spi.h"

#include "default_config.h"
#include "error.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "event_groups.h"
#include "cmsis_os.h"
#include "usb_uart_tx.h"

#define PDOA_V2_BOARD          /* For PDOA_V1_BOARD, Comment the PDOA_V2_BOARD flag */

#ifdef PDOA_V2_BOARD
#define IMU_SPI_ENABLE          // IMU_SPI_ENABLE is supported only for PDOA_V2_BOARD. 
                                // For PDOA_V1_BOARD, Comment the IMU_SPI_ENABLE flag which tests IMU with I2C
#endif    // PDOA_V2_BOARD                             

/* ENABLE_USB_PRINT Macro is uncommented then Segger RTT Print will be enabled*/
#define ENABLE_USB_PRINT

/* DW1000s Pins */
#define DW1000_CLK_Pin        NRF_GPIO_PIN_MAP(0, 16)
#define DW1000_MOSI_Pin       NRF_GPIO_PIN_MAP(0, 20)
#define DW1000_MISO_Pin       NRF_GPIO_PIN_MAP(0, 21)

#define DW1000_IRQ_A_Pin      NRF_GPIO_PIN_MAP(0, 23)
#define DW1000_RST_A_Pin      NRF_GPIO_PIN_MAP(0, 22)
#define DW1000_WUP_A_Pin      NRF_GPIO_PIN_MAP(1,  0)
#define DW1000_CS_A_Pin       NRF_GPIO_PIN_MAP(0, 25) 

#define DW1000_IRQ_B_Pin      NRF_GPIO_PIN_MAP(1, 10)
#define DW1000_RST_B_Pin      NRF_GPIO_PIN_MAP(1,  2) 
#define DW1000_CS_B_Pin       NRF_GPIO_PIN_MAP(1,  1)
#define DW1000_WUP_B_Pin      NRF_GPIO_PIN_MAP(1,  3)

#define DW1000_SYNC_EN_Pin    NRF_GPIO_PIN_MAP(1,13)
#define DW1000_SYNC_CLR_Pin   NRF_GPIO_PIN_MAP(1,14)
#define DW1000_SYNC_Pin       NRF_GPIO_PIN_MAP(1,15)

/* NFC pins */
#define NFC_PIN_1          NRF_GPIO_PIN_MAP(0,9)
#define NFC_PIN_2          NRF_GPIO_PIN_MAP(0,10)

/* IMU Sensor Pins */
#define LSM6DSL_CS_PIN   NRF_GPIO_PIN_MAP(1,  8)   /* P1.08 */
#define LSM6DSR_CS_PIN   NRF_GPIO_PIN_MAP(1,  8)   /* P1.08 */
#define LIS2MDL_CS_PIN   NRF_GPIO_PIN_MAP(1,  9)   /* P1.09 */
#define LPS22HB_CS_PIN   NRF_GPIO_PIN_MAP(0, 11)

#define SPIM2_SCK_PIN   NRF_GPIO_PIN_MAP(0, 28)  // SPI clock GPIO pin number.
#define SPIM2_MOSI_PIN  NRF_GPIO_PIN_MAP(0, 29)  // SPI Master Out Slave In GPIO pin number.

#define LSM6DSL_LPS22HB_SDO_PIN     NRF_GPIO_PIN_MAP(0,  8)

#define LSM6DSL_IRQ2_PIN   NRF_GPIO_PIN_MAP(0, 13)
#define LPS22HB_IRQ_PIN    NRF_GPIO_PIN_MAP(0, 14)
#define LIS2MDL_IRQ_PIN    NRF_GPIO_PIN_MAP(0, 17)
#define LSM6DSL_IRQ1_PIN   NRF_GPIO_PIN_MAP(0, 19)

#define LSM6DSL_I2C_ADDR      0b1101011
#define LIS2MDL_I2C_ADDR      0b0011110

#ifndef PDOA_V2_BOARD
#define LPS22HB_I2C_ADDR      0b1011100
#else
#define LPS22HB_I2C_ADDR      0b1011101 
#endif  // PDOA_V2_BOARD

#define LED_STATIONARY          LED_1
#define LED_NODE                LED_2
#define LED_USB                 LED_3
#define LED_ERROR               LED_4

#define UART_RX_BUF_SIZE        0x100   /**< Read buffer for UART reception, shall be 1<<X */
#define UART_TX_BUF_SIZE        0x100   /**< Write buffer for UART transmission, shall be 1<<X */
#define USB_RX_BUF_SIZE         0x100   /**< Read buffer for USB reception, shall be 1<<X */
#define COM_RX_BUF_SIZE         USB_RX_BUF_SIZE /**< Communication RX buffer size */

#define NRF52_FLASH_CONFIG_ADDR     0x70000

#if (COM_RX_BUF_SIZE < 64)
#error "COM_RX_BUF_SIZE should be longer than CDC_DATA_FS_MAX_PACKET_SIZE"
#endif

typedef struct os_mailQ_cb *osMailQId;
typedef struct os_pool_cb *osPoolId;
typedef QueueHandle_t osMessageQId;          /* Dependnacy for osEvent struct */

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
    mTCFM,        /**< Transmit Continuous Frame Mode mode */
    mTESTFN     /**< Production Test Function mode */
}mode_e;

/* events to start/stop tasks : event group */
enum{
    Ev_Tag_Task         = 0x08,
    Ev_Node_A_Task      = 0x10,
    Ev_Node_B_Task      = 0x20,
    Ev_Tcfm_A_Task      = 0x40,
    Ev_Tcfm_B_Task      = 0x80,
    Ev_Tcwm_A_Task      = 0x100,
    Ev_Tcwm_B_Task      = 0x200,
    Ev_Usb2spi_A_Task	= 0x400,
    Ev_Usb2spi_B_Task   = 0x800,
    Ev_Test_Task        = 0x1000,
    Ev_Stop_All         = 0x2000
};

 /* Application tasks handles & corresponded signals structure */
 typedef struct
 {
     osThreadId Handle;     /* Task�s handler */
     osMutexId  MutexId;    /* Task�s mutex */
     int32_t    Signal;     /* Task�s signal */
 }task_signal_t;

/* Application's global parameters structure */
typedef struct
{
    param_block_t   *pConfig;       /**< Current configuration */
    mode_e          mode;           /**< Information: handle the current "mode" of operation */
    int             lastErrorCode;  /**< Saves the error code in the error_handler() function */
    int             maxMsgLen;      /**< See the longest string size to optimize the MAX_STR_SIZE */

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

    osMailQId         (rxPcktPool_q_id);    /**< Mail queue ID for Twr processes: FreeRTOS does not free the resources */

    //defaultTask is always running and is not accepting signals

    task_signal_t    ctrlTask;          /* usb/uart RX: Control task */
    task_signal_t    flushTask;         /* usb/uart TX: Flush task */

    /* top-level tasks for TWR */
    task_signal_t    rxTask;            /* Tag/Node */
    task_signal_t    calcTask;          /* Node only */
    task_signal_t   blinkTask;          /* Tag only */
    /* top-level tasks for special modes */
    task_signal_t    usb2spiTask;       /* USB2SPI top-level application */
    task_signal_t    tcfmTask;          /* TCFM top-level application */
    task_signal_t    tcwmTask;          /* TCWM top-level application */
    task_signal_t    testTask;          /* Production Test top-level application */

    task_signal_t    imuTask;           /* Tag/Node */


}__attribute__((packed))
app_t;

extern app_t app;

void error_handler(int block, error_e err);
int8_t usb_data_receive();

ret_code_t nrf_drv_spi_reinit(nrf_drv_spi_t const * const p_instance,
                             nrf_drv_spi_config_t const * p_config,
                             nrf_drv_spi_evt_handler_t    handler,
                             void *                       p_context);


#ifdef __cplusplus
}
#endif

#endif /* __DW_PDOA_NODE_COMMON__H__ */
