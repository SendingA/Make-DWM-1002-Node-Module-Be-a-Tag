/*
 * @file      task_test.c
 *
 * @brief     Production Test Application implementation
 *
 * @author    Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#include "port_platform.h"
#include "dw_pdoa_node_common.h"
#include <test_fn.h>

//-----------------------------------------------------------------------------

/* @fn       test_terminate_tasks 
 * @brief    Kill all tasks related to Production test 
 * 
 */
void test_terminate_tasks(void)
{
    if(app.testTask.Handle)
    {
        osMutexWait(app.testTask.MutexId, osWaitForever);

        if(osThreadTerminate(app.testTask.Handle) == osOK)
        {
            test_process_terminate();

            osMutexDelete(app.testTask.MutexId);
            app.testTask.Handle = NULL;
        }
        else
        {
            error_handler(1, _ERR_Cannot_Delete_testTask);
        }
    }

}


/* @fn        StartTestTask
 * @brief     This starts the Production Test application functionality.
 *
 */
void StartTestTask(void const *argument)
{
    osMutexDef(testMutex);
    app.testTask.MutexId = osMutexCreate(osMutex(testMutex));

    test_process_init();

    while(1)
    {
        osMutexRelease(app.testTask.MutexId);

        osDelay(1000 / portTICK_PERIOD_MS);

        osMutexWait(app.testTask.MutexId, 0);

        test_process_run();

        osDelay(5000);
    }
}

