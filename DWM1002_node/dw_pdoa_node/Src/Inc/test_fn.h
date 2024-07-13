/*
 * @file      test_fn.h
 * @brief     Header file for bare-metal test_fn.c
 *
 * @author    Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef __TEST_FN_H_
#define __TEST_FN_H_    1

/* Definitions for compatibility */
#define TEST_ENTER_CRITICAL()    taskENTER_CRITICAL()
#define TEST_EXIT_CRITICAL()     taskEXIT_CRITICAL()

#ifdef __cplusplus
 extern "C" {
#endif

error_e    test_process_init(void);
void test_process_run(void);
void test_process_terminate(void);

#ifdef __cplusplus
}
#endif

#endif /* __TEST_FN_H_ */
