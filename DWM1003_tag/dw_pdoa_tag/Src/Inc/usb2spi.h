/*
 * @file     usb2spi.h
 * @brief      header file for bare-metal usb2spi.c
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef __INC_USB2SPI_H_
#define __INC_USB2SPI_H_    1

#ifdef __cplusplus
 extern "C" {
#endif

usb_data_e usb2spi_protocol_check(uint8_t *p, uint16_t len);
error_e    usb2spi_process_init(dw_name_e    chip);
void usb2spi_process_run(void);
void usb2spi_process_terminate(void);

#ifdef __cplusplus
}
#endif

#endif /* __INC_USB2SPI_H_ */
