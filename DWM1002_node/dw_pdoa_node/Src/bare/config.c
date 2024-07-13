/*
 * @file       config.c
 *
 * @brief      supports NVM and bss configuration sections:
 *             defaultFConfig : section in RAM, where default parameters are saved and is not re-writabele.
 *              FCONFIG_ADDR  : section in NVM, where current parameters are saved and this is re-writabele.
 *                 bssConfig  : section in RAM, which is representing config data exist in FCONFIG_ADDR.
 *
 *             application on startup shall init_bssConfig() : this will copy data from FCONFIG_ADDR -> bssConfig
 *             Accessing to variables is by pointer get_pbssConfig();
 *
 *             if application wants to re-write data in FCONFIG_ADDR, use save_bssConfig(*newRamParametersBlock);
 *
 *             NOTE: The code is very MCU dependent and save will work with nRF52840 only
 *
 * @author     Decawave Software
 *
 * @attention  Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *             All rights reserved.
 *
 */

#include <stdint.h>
#include <string.h>

#include "deca_device_api.h"
#include "deca_version.h"
#include "version.h"

#include "config.h"
#include "nrf_nvmc.h"

#include "dw_pdoa_node_common.h"

//------------------------------------------------------------------------------

const param_block_t config_data = DEFAULT_CONFIG;

const param_block_t FConfig __attribute__((section(".fConfig"))) \
                                   __attribute__((aligned(FCONFIG_SIZE))) = DEFAULT_CONFIG;

/* Section ".default_config" is defined in a linker file */
const param_block_t defaultFConfig __attribute__((section(".default_config"))) \
                                    __attribute__((aligned(FCONFIG_SIZE))) = DEFAULT_CONFIG;

/* run-time parameters block.
 *
 * This is the RAM image of the FCONFIG_ADDR .
 *
 * Accessible from application by app.pConfig pointer after init_bssConfig()
 *
 * */
static param_block_t tmpConfig __attribute__((aligned(FCONFIG_SIZE)));

//------------------------------------------------------------------------------
// Implementation

/*
 * @brief get pointer to run-time bss param_block_t block
 *
 * */
param_block_t *get_pbssConfig(void)
{
    return app.pConfig;
}


/* @fn      load_bssConfig
 * @brief   copy parameters from NVM to RAM structure.
 *
 *          assumes that memory model in the MCU of .text and .bss are the same
 * */
void load_bssConfig(void)
{
    memcpy(&tmpConfig, &FConfig, sizeof(tmpConfig));

    app.pConfig = &tmpConfig;
}

/* @fn      restore_bssConfig
 * @brief   copy parameters from default RAM section to RAM structure.
 *
 *          assumes that memory model in the MCU of .text and .bss are the same
 * */
void restore_bssConfig(void)
{
    __disable_irq();
    nrf_nvmc_page_erase((uint32_t)&FConfig);
    nrf_nvmc_write_bytes((uint32_t)&FConfig,  (const uint8_t *)&defaultFConfig, FCONFIG_SIZE);

    load_bssConfig();
    __enable_irq();
}

/* @brief    save pNewRamParametersBlock to FCONFIG_ADDR
 * @return  _NO_ERR for success and error_e code otherwise
 * */
error_e save_bssConfig(param_block_t * pNewRamParametersBlock)
{
    __disable_irq();
    nrf_nvmc_page_erase((uint32_t)&FConfig);

    nrf_nvmc_write_bytes((uint32_t)&FConfig, (const uint8_t*)pNewRamParametersBlock, FCONFIG_SIZE);
    __enable_irq();

    return (_NO_ERR);
}
