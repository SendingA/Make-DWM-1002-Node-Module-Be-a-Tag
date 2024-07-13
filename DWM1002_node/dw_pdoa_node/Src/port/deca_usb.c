/*! ----------------------------------------------------------------------------
 * @file    deca_usb.c
 * @brief   HW specific definitions and functions for USB Interface
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


#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "nrf.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"

#include "app_error.h"
#include "app_util.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"

#include "boards.h"
#include "port_platform.h"


#include "dw_pdoa_node_common.h"
#include "circ_buf.h"

#define LED_USB_RESUME      (BSP_BOARD_LED_0)
#define LED_CDC_ACM_OPEN    (BSP_BOARD_LED_1)
#define LED_CDC_ACM_RX      (BSP_BOARD_LED_2)
#define LED_CDC_ACM_TX      (BSP_BOARD_LED_3)

/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1

/**
 * @brief CDC_ACM class instance
 * */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250
);

#define READ_SIZE 1

static char m_rx_buffer[NRF_DRV_USBD_EPSIZE];
static char m_tx_buffer[NRF_DRV_USBD_EPSIZE];
static bool m_send_flag = 0;
extern bool gProd_test_enable;


/**
 * @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t (headphones)
 * */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
            bsp_board_led_on(LED_CDC_ACM_OPEN);

            /*Setup first transfer*/
            ret_code_t ret = app_usbd_cdc_acm_read_any(&m_app_cdc_acm,
                                                   m_rx_buffer,
                                                   sizeof(m_rx_buffer));
            UNUSED_VARIABLE(ret);
            break;
        }
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            bsp_board_led_off(LED_CDC_ACM_OPEN);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            if (!gProd_test_enable)
            {
              bsp_board_led_invert(LED_CDC_ACM_TX);
            }
            break;
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            /*Get amount of data transfered*/
            size_t len = app_usbd_cdc_acm_rx_size(p_cdc_acm);

            /*Setup next transfer*/
            ret_code_t ret = app_usbd_cdc_acm_read_any(&m_app_cdc_acm,
                                                   m_rx_buffer,
                                                   sizeof(m_rx_buffer));

          /* [USB_HOST] =LongData==> [USB_DEVICE USBRX_IRQ (me)]===> UserRxBufferFS ===> CDC_Receive_FS(Buf:Len) ===> app.usbbuf
           *                                                      ^                                               |            |
           *                                                      |                                               |            |
           *                                                      +--------<--------<---------<---------<---------+            +->signal.usbRx
           *
           * */

            //we need intermediate buffer app.usbRx.buf to receive "long" USB packet inside ISR.
            //Alternatively we can define long APP_RX_DATA_SIZE, but this will affect on USB RX descriptor length (and the length of associated buffers).
            //Better to keep CDC_DATA_FS_MAX_PACKET_SIZE in range from 16 to 64.

            int head, tail, size;

            head = app.usbRx.head;
            tail = app.usbRx.tail;
            size = sizeof(app.usbRx.buf);

            if (CIRC_SPACE(head, tail, size) > len)
            {
                for(int i = 0; i<len; i++)
                {
                    app.usbRx.buf[head] = m_rx_buffer[i];
                    head = (head + 1) & (size - 1);
                }

                app.usbRx.head = head;
            }
            else
            {
                /* USB RX packet can not fit free space in the buffer */
            }

            if(app.ctrlTask.Handle) //RTOS : ctrlTask could be not started yet
            {
                osSignalSet(app.ctrlTask.Handle, app.ctrlTask.Signal);    //signal to the ctrl thread : USB data ready
            }
            break;
        }
        default:
            break;
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
            bsp_board_led_off(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_DRV_RESUME:
            bsp_board_led_on(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_STARTED:
            break;
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            bsp_board_leds_off();
            break;
		case APP_USBD_EVT_POWER_DETECTED:

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            app_usbd_start();
            break;
        default:
            break;
    }
}

int deca_usb_transmit(char *tx_buffer, int size)
{
    ret_code_t ret;

    for(int i=0; i<1; i++)
    {
      ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, tx_buffer, size);
      if (ret != NRF_SUCCESS)
      {
         break;
      }
    }
    return ret;
}

int pp_usb_init(void)
{
    ret_code_t ret;
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };

    int  frame_counter = 0;

    app_usbd_serial_num_generate();

    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);

    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);

    if (USBD_POWER_DETECTION)
    {
        ret = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret);
    }
    else
    {
        app_usbd_enable();
        app_usbd_start();
    } 
}
/** @} */
