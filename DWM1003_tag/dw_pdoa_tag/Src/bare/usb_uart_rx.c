/*
 * @file     usb_uart_rx.c
 * @brief    This file supports Decawave USB-TO-SPI and Control modes.
 *           Functions can be used in both bare-metal and RTOS implementations.
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */


/* Includes */

#include "dw_pdoa_tag_common.h"
#include "circ_buf.h"
#include "error.h"

#include "usb_uart_rx.h"
#include "usb_uart_tx.h"
#include <cmd.h>
#include <usb2spi.h>
#include <string.h>

#define USB_UART_ENTER_CRITICAL()	taskENTER_CRITICAL()
#define USB_UART_EXIT_CRITICAL()	taskEXIT_CRITICAL()
//-----------------------------------------------------------------------------
// IMPLEMENTATION

/*
 * @brief    Waits only commands from incoming stream.
 *             The binary interface (deca_usb2spi stream) is not allowed.
 *
 * @return  COMMAND_READY : the data for future processing can be found in app.local_buff : app.local_buff_len
 *          NO_DATA : no command yet
 */
usb_data_e waitForCommand(uint8_t *pBuf, uint16_t len)
{
    usb_data_e ret;
    static uint8_t cmdLen = 0;
    static uint8_t cmdBuf[COM_RX_BUF_SIZE]; /**< slow command buffer : small size */

    ret = NO_DATA;

    if (len <= 2)
    {/* "slow" command mode: Human interface. Wait until '\r' or '\n' */
        if (cmdLen == 0)
        {
            memset(cmdBuf, 0, sizeof(cmdBuf));
        }

        if (cmdLen < (sizeof(app.local_buff) - 1))
        {
            port_tx_msg(pBuf, len);    //ECHO

            if (*pBuf == '\n' || *pBuf == '\r')
            {
                if (cmdLen > 0)
                {
                    memcpy(app.local_buff, cmdBuf, cmdLen);

                    app.local_buff_length = cmdLen;
                    app.local_buff[cmdLen] = 0;

                    ret = COMMAND_READY;
                    cmdLen = 0;
                }
            }
            else if (*pBuf == '\b') //erase of a char in the terminal
            {
                if (cmdLen > 0)
                {
                    --cmdLen;
                    cmdBuf[cmdLen] = 0;
                    port_tx_msg((uint8_t*) "\033[K", 3);
                }

            }
            else
            {
                cmdBuf[cmdLen] = *pBuf;
                cmdLen++;
            }
        }
        else
        {
            /* error in command protocol : flush everything */
            port_tx_msg((uint8_t*) "\r\n", 2);
            cmdLen = 0;
        }
    }
    else
    {/* "fast" command mode : assume every data buffer is "COMMAND_READY" */

        if (len < (sizeof(app.local_buff) - 1))
        {
            memcpy(app.local_buff, pBuf, len);

            app.local_buff_length = len;
            app.local_buff[len] = 0;
            cmdLen = 0;

            ret = COMMAND_READY;
        }
        else
        { /* overflow in protocol : flush everything */
            port_tx_msg((uint8_t*) "Error: \r\n", 2);
            cmdLen = 0;
        }
    }

    return (ret);
}

/*
 * @brief    Waits for binary interface (deca_usb2spi stream) from incoming stream.
 *           The command interface command "STOP" is the only from allowed commands.
 *
 * @return  DATA_READY : the data is ready for future processing in usb2spi application
 *                       data can be found in app.local_buff : app.local_buff_len
 *          NO_DATA    : no valid data yet
 */
usb_data_e waitForData(uint8_t *pBuf, uint16_t len)
{
    static uint16_t dataLen = 0;
    usb_data_e ret;

    ret = NO_DATA;

    /* wait for valid usb2spi message from pBuf */

    if ((len + dataLen) < sizeof(app.local_buff) - 1)
    {
        memcpy(&app.local_buff[dataLen], pBuf, len);

        dataLen += len;
        app.local_buff[dataLen] = 0;

        ret = usb2spi_protocol_check(app.local_buff, dataLen);

        if (ret == DATA_READY)
        {
            app.local_buff_length = dataLen;
            dataLen = 0;
        }
    }
    else
    { /* overflow in usb2spi protocol : flush everything */
        dataLen = 0;
    }

    return (ret);
}

/* @fn     usb_uart_rx
 * @brief  this should be calling on a reception of a data from UART or USB.
 *         uses platform-dependent
 *
 * */
usb_data_e usb_uart_rx(void)
{
    usb_data_e ret;

    int uartLen;
    int headUart, tailUart, sizeUart;

    ret = NO_DATA;

    /* USART control prevails over USB control if both at the same time */

    USB_UART_ENTER_CRITICAL();
    headUart = app.uartRx.head;
    tailUart = app.uartRx.tail;
    sizeUart = sizeof(app.uartRx.buf);

    USB_UART_EXIT_CRITICAL();

    uartLen = CIRC_CNT(headUart, tailUart, sizeUart);

    if ((uartLen > 0) && (app.pConfig->s.uartEn == 1))
    {
        /* copy from circular buffer to linear buffer */
        uint8_t ubuf[sizeof(app.uartRx.buf)];

        for (int i = 0; i < uartLen; i++)
        {
            ubuf[i] = app.uartRx.buf[tailUart];
            tailUart = (tailUart + 1) & (sizeUart - 1);
        }

        USB_UART_ENTER_CRITICAL();
        app.uartRx.tail = tailUart;
        USB_UART_EXIT_CRITICAL();

        ret = waitForCommand(ubuf, uartLen);
    }
    else
    {

    }

    return ret;
}

