/*
 * @file   task_usb2spi.h
 * @brief  header file for task_usb2spi.c
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef __INC_TASK_USB2SPI_H_
#define __INC_TASK_USB2SPI_H_    1

#ifdef __cplusplus
 extern "C" {
#endif

void StartUsb2SpiTask(void const * arg);
void usb2spi_terminate_tasks(void);

#ifdef __cplusplus
}
#endif

#endif /* __INC_TASK_USB2SPI_H_ */
