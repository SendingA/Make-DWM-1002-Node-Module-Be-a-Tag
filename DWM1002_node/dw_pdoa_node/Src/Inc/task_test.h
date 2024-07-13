/*
 * @file      task_test.h
 *
 * @brief     Header file for task_test.c
 *
 * @author    Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef __TASK_TEST_H_
#define __TASK_TEST_H_    1

#ifdef __cplusplus
 extern "C" {
#endif

void StartTestTask(void const * arg);
void test_terminate_tasks(void);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_TEST_H_ */
