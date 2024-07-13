/*
 * @file    task_flush.h
 * @brief
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef __FLUSH_TASK__H__
#define __FLUSH_TASK__H__ 1

#ifdef __cplusplus
 extern "C" {
#endif

#include "port_platform.h"
#include "dw_pdoa_node_common.h"

void FlushTask(void const * argument);
void reset_FlushTask(void);

#ifdef __cplusplus
}
#endif

#endif /* __FLUSH_TASK__H__ */
