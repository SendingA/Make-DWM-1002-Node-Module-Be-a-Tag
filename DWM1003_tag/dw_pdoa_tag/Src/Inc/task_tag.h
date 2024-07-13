/*
 *  @file    task_tag.h
 *  @brief   header for task_tag.c
 *           DecaWave Application Layer
 *           RTOS tag implementation
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef __TASK_TAG_H__
#define __TASK_TAG_H__ 1

#ifdef __cplusplus
 extern "C" {
#endif


 typedef struct {
    uint16_t    addr;
    uint16_t    node_addr;
    uint8_t     rNum;
    int16_t     x_cm;
    int16_t     y_cm;
    int16_t     clkOffset_pphm;
 }twr_res_ext_t;

#include "tag.h"

void tag_helper(void const *argument);
void tag_terminate_tasks(void);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_TAG_H__ */
