/*
 * @file      task_tcfm.h
 *
 * @brief     Header file for task_tcfm.c
 *
 * @author    Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef __INC_TASK_TCFM_H_
#define __INC_TASK_TCFM_H_    1

#ifdef __cplusplus
 extern "C" {
#endif

void StartTcfmTask(void const * arg);
void tcfm_terminate_tasks(void);

#ifdef __cplusplus
}
#endif

#endif /* __INC_TASK_TCFM_H_ */
