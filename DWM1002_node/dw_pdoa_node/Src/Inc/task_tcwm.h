/*
 * @file       task_tcwm.h
 *
 * @brief      Header file for task_tcwm.c
 *
 * @author     Decawave Software
 *
 * @attention  Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *             All rights reserved.
 *
 */

#ifndef __INC_TASK_TCWM_H_
#define __INC_TASK_TCWM_H_    1

#ifdef __cplusplus
 extern "C" {
#endif

void StartTcwmTask(void const * arg);
void tcwm_terminate_tasks(void);

#ifdef __cplusplus
}
#endif

#endif /* __INC_TASK_TCWM_H_ */
