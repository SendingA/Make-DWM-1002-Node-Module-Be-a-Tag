/*
 * @file cmd_fn.h
 *
 * @brief  header file for cmd_fn.c
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */
#ifndef INC_CMD_FN_H_
#define INC_CMD_FN_H_    1

#ifdef __cplusplus
 extern "C" {
#endif

#include "port_platform.h"
#include "dw_pdoa_node_common.h"

//-----------------------------------------------------------------------------
/* module DEFINITIONS */
#define MAX_STR_SIZE            255

#define CMD_MALLOC              pvPortMalloc
#define CMD_FREE                vPortFree
#define CMD_ENTER_CRITICAL()    taskENTER_CRITICAL()
#define CMD_EXIT_CRITICAL()     taskEXIT_CRITICAL()

//-----------------------------------------------------------------------------
/* All cmd_fn functions have unified input: (char *text, param_block_t *pbss, int val) */
/* use REG_FN(x) macro */
#define REG_FN(x) const char *x(char *text, param_block_t *pbss, int val)

 /* command table structure definition */
 typedef struct {
     const char     *name;      /**< Command name string */
     const mode_e   mode;       /**< allowed execution operation mode */
     REG_FN         ((*fn));    /**< function() */
 }command_t;

extern const command_t known_commands[];

extern error_e port_tx_msg(uint8_t* str, int len);

#ifdef __cplusplus
}
#endif


#endif /* INC_CMD_FN_H_ */
