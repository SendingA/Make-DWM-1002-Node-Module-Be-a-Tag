/*
 * @file default_config.h
 *
 * @brief  default config fie for NVM initialization
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef __DEFAULT_CONFIG_H__H__
#define __DEFAULT_CONFIG_H__H__ 1

#ifdef __cplusplus
 extern "C" {
#endif

#include <inttypes.h>
#include <deca_device_api.h>

/* UWB config */
#define DEFAULT_CHANNEL             5
#define DEFAULT_PRF                 DWT_PRF_64M
#define DEFAULT_TXPREAMBLENGTH      DWT_PLEN_128
#define DEFAULT_RXPAC               DWT_PAC8
#define DEFAULT_PCODE               9
#define DEFAULT_NSSFD               1
#define DEFAULT_DATARATE            DWT_BR_6M8
#define DEFAULT_PHRMODE             DWT_PHRMODE_STD
#define DEFAULT_SFDTO               (128 + 1 + 16 - 8)

/* run-time config */
#define DEFAULT_UART                1       /**< Do not output to the UART by default */
#define DEFAULT_AUTOS               1       /**< Activate auto start of the Tag top-level application */
#define DEFAULT_REPORT_LEVEL        1       /**< Send TWR & IMU reports back to PC by default */
#define DEFAULT_SMARTTX             0       /**< Do not use Smart Tx on Tag by default */
#define DEFAULT_DEBUG               0       /**< if 1, then the LED_RED used to show an error, if any */
#define DEFAULT_FAULTY_RANGES_NUM   3       /**< after this number of (sequential) ranges faults the tag will return to the discovery phase */
#define DEFAULT_USB2SPI_EMULATE_EVB 0       /**< to emulate EVB USB2SPI string */
#define DEFAULT_STATIONARY_MS       200    /**< Time of consecutive readings under the threshold needed to consider that the device is stationary */
#define DEFAULT_MOVING_MS           200    /**< Time of consecutive readings above the threshold needed to consider that the device is moving */
#define ACCEL_NORMALIZED_THRESHOLD  300      /**< Acceleration values difference threshold to consider stationary or moving state.
                                                 This normalized to a resolution in "g" values, we will use a 5% threshold as 1g == 1000 */

#define DEFAULT_ANTD                (515.836f * 1e-9 / DWT_TIME_UNITS)

 /* This configures the delay between end of Tag's Blink_TX and Tag start Rx of Ranging Config message
  * Should be the same for Node and Tag
  * */
#define DEFAULT_TAG_BLINK_TX_RC_RX_US   (1000)
#define DEFAULT_RC_RX_TIMEOUT_US        (300)

/* EEPROM SIZE used for store configuration */
#define FCONFIG_SIZE            0x100   /**< See NVM/EEPROM linker map and save_bssConfig() */


/* Default configuration initialization */
#define    DEFAULT_CONFIG \
{\
    .s.uartEn                   = DEFAULT_UART, \
    .s.autoStartEn              = DEFAULT_AUTOS, \
    .s.reportLevel              = DEFAULT_REPORT_LEVEL, \
    .s.smartTxEn                = DEFAULT_SMARTTX, \
    .s.debugEn                  = DEFAULT_DEBUG, \
    .s.faultyRanges             = DEFAULT_FAULTY_RANGES_NUM, \
    .s.emuEVB                   = DEFAULT_USB2SPI_EMULATE_EVB, \
    .s.acc_stationary_ms        = DEFAULT_STATIONARY_MS, \
    .s.acc_moving_ms            = DEFAULT_MOVING_MS, \
    .s.acc_threshold            = ACCEL_NORMALIZED_THRESHOLD, \
    \
    .s.ant_rx_a                 = (uint16_t)(0.45* DEFAULT_ANTD), \
    .s.ant_tx_a                 = (uint16_t)(0.55* DEFAULT_ANTD), \
    .s.rcDelay_us               = (uint16_t)DEFAULT_TAG_BLINK_TX_RC_RX_US, \
    .s.rcRxTo_us                = (uint16_t)DEFAULT_RC_RX_TIMEOUT_US, \
    \
    .dwt_config.chan            = DEFAULT_CHANNEL, \
    .dwt_config.prf             = DEFAULT_PRF, \
    .dwt_config.txPreambLength  = DEFAULT_TXPREAMBLENGTH, \
    .dwt_config.rxPAC           = DEFAULT_RXPAC, \
    .dwt_config.txCode          = DEFAULT_PCODE, \
    .dwt_config.rxCode          = DEFAULT_PCODE, \
    .dwt_config.nsSFD           = DEFAULT_NSSFD, \
    .dwt_config.dataRate        = DEFAULT_DATARATE, \
    .dwt_config.phrMode         = DEFAULT_PHRMODE, \
    .dwt_config.sfdTO           = DEFAULT_SFDTO \
}

/* struct which holding a run-time parameters */
typedef struct {
    uint8_t     uartEn;         /**< activate UART output */
    uint8_t     autoStartEn;    /**< activate auto start */
    uint8_t     reportLevel;    /**< 0: no output 1: JSON output Reports from Tag to PC */
    uint8_t     smartTxEn;      /**< activate Smart Tx  */
    uint8_t     debugEn;        /**< 0: will not stop in error_handler() */
    uint8_t     faultyRanges;   /**< Number of consecutive faulty ranges to consider return back to blinking mode */
    uint8_t     emuEVB;
    uint16_t    acc_stationary_ms;
    uint16_t    acc_moving_ms;
    uint16_t    acc_threshold;
    uint16_t    ant_rx_a;       /**< antenna delay values DW_A */
    uint16_t    ant_tx_a;       /**< antenna delay values DW_A */
    uint16_t    rcDelay_us;     /**< Ranging Config RX on delay: shall be the same to Node */
    uint16_t    rcRxTo_us;      /**< Ranging Config RX timeout: save energy */
}__attribute__((packed)) run_t;


/* The structure, holding the changeable configuration of the application
 * */

typedef struct {
    run_t           s;                  /**< Run-time parameters */
    dwt_config_t    dwt_config;        /**< Standard Decawave driver config */
    uint8_t         free[FCONFIG_SIZE - (sizeof(run_t) + sizeof(dwt_config_t))];
}__attribute__((packed)) param_block_t;

#ifdef __cplusplus
}
#endif

#endif /* __DEFAULT_CONFIG__H__ */
