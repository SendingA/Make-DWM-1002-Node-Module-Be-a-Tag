/*! ----------------------------------------------------------------------------
 * @file    deca_uart.c
 * @brief   HW specific definitions and functions for UART Interface
 *
 * @attention
 *
 * Copyright 2016 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Deepa Gopinath
 */

#include "port_platform.h"
#include "nrf_uart.h"
#include "app_uart.h"
#include "dw_pdoa_node_common.h"
#include "circ_buf.h"

/******************************************************************************
 *
 *                              Uart Configuration
 *
 ******************************************************************************/

/* @fn  uart_error_handle
 *
 * @param[in] p_event Pointer to UART event.
 * */
void deca_uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
    /* This event indicates that UART data has been received  */
    else if (p_event->evt_type == APP_UART_DATA_READY)
    {
        deca_uart_receive();
    }
}

/* @fn  deca_uart_init
 *
 * @brief Function for initializing the UART module.
 *
 * @param[in] void
 * */
void deca_uart_init(void)
{
    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
    {
        UART_0_RX_PIN,
        UART_0_TX_PIN,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       deca_uart_error_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    UNUSED_VARIABLE(err_code); 
}

/* @fn  deca_uart_transmit
 *
 * @brief Function for transmitting data on UART
 *
 * @param[in] ptr Pointer is contain base address of data.
 * */
void deca_uart_transmit(char *ptr)
{
    uint32_t bit=0;
    for(bit=0; ptr[bit] != '\0'; bit++)
    {
        nrf_delay_ms(10);
        while(app_uart_put(ptr[bit]) != NRF_SUCCESS);
    }
    while(app_uart_put('\n') != NRF_SUCCESS);
    while(app_uart_put('\r') != NRF_SUCCESS);
}

/* @fn  deca_uart_receive
 *
 * @brief Function for receive data from UART and store into rx_buf
 *        (global array).
 *
 * @param[in] void
 * */
void deca_uart_receive(void)
{
    uint32_t err_code;
    uint8_t rx_data;
    int head, tail, size;

    head = app.uartRx.head;
    tail = app.uartRx.tail;
    size = sizeof(app.uartRx.buf);

    err_code = app_uart_get(&rx_data);
    if (CIRC_SPACE(head, tail, size) > 0)
    {
        app.uartRx.buf[head] = rx_data;
        head = (head + 1) & (size - 1);
        app.uartRx.head = head;
    }
    else
    {
        /* USB RX packet can not fit free space in the buffer */
    }

    if(app.ctrlTask.Handle) //RTOS : ctrlTask could be not started yet
    {
        osSignalSet(app.ctrlTask.Handle, app.ctrlTask.Signal);    //signal to the ctrl thread : USB data ready
    }
}
/****************************************************************************//**
 *
 *                          End of UART Configuration
 *
 *******************************************************************************/
