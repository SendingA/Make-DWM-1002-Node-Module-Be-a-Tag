/*! ----------------------------------------------------------------------------
 * @file    dw_pdoa_node_common.c
 * @brief   Defines Commom functionalities of Node Application
 *
 * @author  Decawave 
 *
 * @attention
 *
 * Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */


/* Includes ------------------------------------------------------------------*/

#include "app_error.h"
#include "app_util.h"
#include "app_usbd_cdc_acm.h"

#include "nrf_drv_twi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_clock.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h" 

#include "port_platform.h"

#include "dw_pdoa_node_common.h"
#include "circ_buf.h"
#include "nrf_drv_wdt.h"

#include "lsm6dsl.h"
#include "lps22hb.h"
#include "lis2mdl.h"

const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(0); /**< Declaring an instance of nrf_drv_rtc for RTC0. */

nrf_drv_twi_t _twi = NRF_DRV_TWI_INSTANCE(1);

#define SPI_INSTANCE  0 /**< SPI instance index. */
const nrf_drv_spi_t imu_spi_inst = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);
nrf_drv_spi_config_t imu_spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
void interrupts_init(void);
void wdt_init(void);
bool imu_spi_init_flag = 0;
bool imu_twi_init_flag = 0;

/**@brief RTC driver instance control block structure. */
typedef struct
{
    nrfx_drv_state_t state;        /**< Instance state. */
    bool             reliable;     /**< Reliable mode flag. */
    uint8_t          tick_latency; /**< Maximum length of interrupt handler in ticks (max 7.7 ms). */
} nrfx_rtc_cb_t;

// User callbacks local storage.
static nrfx_rtc_handler_t rtc_handlers[NRFX_RTC_ENABLED_COUNT];
static nrfx_rtc_cb_t      rtc_cb[NRFX_RTC_ENABLED_COUNT];

static nrf_drv_spi_evt_handler_t m_handlers[SPIM_COUNT];
static void *                    m_contexts[SPIM_COUNT];

#ifdef SPIM_PRESENT
static void spim_evt_handler(nrfx_spim_evt_t const * p_event,
                             void *                  p_context)
{
    uint32_t inst_idx = (uint32_t)p_context;
    nrf_drv_spi_evt_t const event =
    {
        .type = (nrf_drv_spi_evt_type_t)p_event->type,
        .data =
        {
            .done =
            {
                .p_tx_buffer = p_event->xfer_desc.p_tx_buffer,
                .tx_length   = p_event->xfer_desc.tx_length,
                .p_rx_buffer = p_event->xfer_desc.p_rx_buffer,
                .rx_length   = p_event->xfer_desc.rx_length,
            }
        }
    };
    m_handlers[inst_idx](&event, m_contexts[inst_idx]);
}
#endif // SPIM_PRESENT

// Control block - driver instance local data.
typedef struct
{
    nrfx_spim_evt_handler_t handler;
    void *                  p_context;
    nrfx_spim_evt_t         evt;  // Keep the struct that is ready for event handler. Less memcpy.
    nrfx_drv_state_t        state;
    volatile bool           transfer_in_progress;

#if NRFX_CHECK(NRFX_SPIM_EXTENDED_ENABLED)
    bool                    use_hw_ss;
#endif

    // [no need for 'volatile' attribute for the following members, as they
    //  are not concurrently used in IRQ handlers and main line code]
    bool            ss_active_high;
    uint8_t         ss_pin;
    uint8_t         miso_pin;
    uint8_t         orc;

#if NRFX_CHECK(NRFX_SPIM_NRF52_ANOMALY_109_WORKAROUND_ENABLED)
    size_t          tx_length;
    size_t          rx_length;
#endif
} spim_control_block_t;
extern spim_control_block_t m_cb[NRFX_SPIM_ENABLED_COUNT];  


void deca_irq_handler(nrf_drv_gpiote_pin_t irqPin, nrf_gpiote_polarity_t irq_action)
{
    process_deca_irq();
}

nrf_drv_wdt_channel_id m_channel_id;

uint32_t wdt_reset_cnt = 0;

/**
 * @brief WDT events handler.
 */
void wdt_event_handler(void)
{
    //NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
    wdt_reset_cnt = wdt_reset_cnt + 1;
}

/* @fn  peripherals_init
 *
 * @param[in] void
 * */
void peripherals_init(void)
{
  ret_code_t ret;
  ret_code_t err_code;

  err_code = nrf_drv_clock_init();
  APP_ERROR_CHECK(err_code);

  nrf_drv_clock_lfclk_request(NULL);

#ifndef ENABLE_USB_PRINT
    ret = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(ret);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\n\rDeca Test Example......");
    NRF_LOG_FLUSH();
#endif

    interrupts_init();
    deca_uart_init();

    /*WDT Initilization*/
    wdt_init();

}


void interrupts_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_PULLDOWN; 

    err_code = nrf_drv_gpiote_in_init(DW1000_IRQ_A_Pin, &in_config, deca_irq_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(DW1000_IRQ_A_Pin, true);

    err_code = nrf_drv_gpiote_in_init(DW1000_IRQ_B_Pin, &in_config, deca_irq_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(DW1000_IRQ_B_Pin, true);

}

void wdt_init(void)
{
    ret_code_t err_code;

    nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
	
	/* WDT Timer is configured for 60Secs*/
    config.reload_value = 60000;
    err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();
}

void imusensor_twi_init(void)
{
   ret_code_t err_code;

    nrf_drv_twi_config_t twi_conf = {
        .frequency = NRF_TWI_FREQ_400K,
        .scl = ARDUINO_SCL_PIN,
        .sda = ARDUINO_SDA_PIN,
        .clear_bus_init=true,
        .hold_bus_uninit=true
    };

   /* Init I2C for the intertial sensors */
   err_code = nrf_drv_twi_init(&_twi, &twi_conf, NULL, NULL);
   APP_ERROR_CHECK(err_code);

   imu_twi_init_flag = 1;
}

void spi2_three_wire_read(int en)
{
    nrf_drv_spi_config_t spi_read_config = NRF_DRV_SPI_DEFAULT_CONFIG;

    spi_read_config.sck_pin   = SPI0_CONFIG_SCK_PIN;
    spi_read_config.miso_pin  = SPI0_CONFIG_MOSI_PIN;
    spi_read_config.mosi_pin  = NRF_DRV_SPI_PIN_NOT_USED;
    spi_read_config.ss_pin    = NRF_DRV_SPI_PIN_NOT_USED;
    spi_read_config.mode      = NRF_DRV_SPI_MODE_3;
    spi_read_config.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;
    spi_read_config.frequency = NRF_DRV_SPI_FREQ_8M;

    nrf_gpio_cfg_input(SPI0_CONFIG_SCK_PIN, NRF_GPIO_PIN_PULLUP);
    nrf_drv_spi_uninit(&imu_spi_inst);

    if (en) {
        /* Reconfig spi for reading from 3wire */
        nrf_drv_spi_reinit(&imu_spi_inst, &spi_read_config, NULL, NULL);
    } else {
        /* Normal 4 wire config */
        nrf_drv_spi_reinit(&imu_spi_inst, &imu_spi_config, NULL, NULL);
    }
}

void imusensor_spi_init(void)
{
    nrf_gpio_cfg_input(LSM6DSL_LPS22HB_SDO_PIN, NRF_GPIO_PIN_PULLUP);
	
    nrf_gpio_cfg_output(LSM6DSL_CS_PIN);nrf_gpio_pin_set(LSM6DSL_CS_PIN);
    nrf_gpio_cfg_output(LIS2MDL_CS_PIN);nrf_gpio_pin_set(LIS2MDL_CS_PIN);
    nrf_gpio_cfg_output(LPS22HB_CS_PIN);nrf_gpio_pin_set(LPS22HB_CS_PIN);

    imu_spi_config.sck_pin  = SPI0_CONFIG_SCK_PIN;
    imu_spi_config.mosi_pin = SPI0_CONFIG_MOSI_PIN;
    imu_spi_config.miso_pin = LSM6DSL_LPS22HB_SDO_PIN;
    imu_spi_config.mode = NRF_DRV_SPI_MODE_3;
    imu_spi_config.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;
    imu_spi_config.frequency = NRF_DRV_SPI_FREQ_8M;

    APP_ERROR_CHECK(nrf_drv_spi_init(&imu_spi_inst, &imu_spi_config, NULL, NULL));

    imu_spi_init_flag = 1;
}

void error_handler(int block, error_e err)
{
    app.lastErrorCode = err;

    if(app.pConfig->s.debugEn)
    {
        if(block)
        {
            /* Flash Error Led*/
            while(block)
            {
                for(int i = err; i>0; i--)
                {

                    nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh

                    nrf_gpio_pin_write(LED_ERROR, 1);
                    nrf_delay_ms(250);
                    nrf_gpio_pin_write(LED_ERROR, 0);
                    nrf_delay_ms(250);
                }

                nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh
                nrf_delay_ms(5000);
                nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh
                nrf_delay_ms(5000);
                nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh

            }
        }
    }
}

ret_code_t nrf_drv_spi_reinit(nrf_drv_spi_t const * const p_instance,
                            nrf_drv_spi_config_t const * p_config,
                            nrf_drv_spi_evt_handler_t    handler,
                            void *                       p_context)
{
    uint32_t inst_idx = p_instance->inst_idx;
    m_handlers[inst_idx] = handler;
    m_contexts[inst_idx] = p_context;

    ret_code_t result = 0;
    if (NRF_DRV_SPI_USE_SPIM)
    {
#ifdef SPIM_PRESENT
        nrfx_spim_config_t config_spim = NRFX_SPIM_DEFAULT_CONFIG;
        config_spim.sck_pin        = p_config->sck_pin;
        config_spim.mosi_pin       = p_config->mosi_pin;
        config_spim.miso_pin       = p_config->miso_pin;
        config_spim.ss_pin         = p_config->ss_pin;
        config_spim.irq_priority   = p_config->irq_priority;
        config_spim.orc            = p_config->orc;
        config_spim.frequency      = (nrf_spim_frequency_t)p_config->frequency;
        config_spim.mode           = (nrf_spim_mode_t)p_config->mode;
        config_spim.bit_order      = (nrf_spim_bit_order_t)p_config->bit_order;
        result = nrfx_spim_init(&p_instance->u.spim,
                                &config_spim,
                                handler ? spim_evt_handler : NULL,
                                (void *)inst_idx);
        if ( result == NRFX_ERROR_INVALID_STATE ) 
        {
            nrfx_spim_uninit(&p_instance->u.spim);
            nrfx_spim_init(&p_instance->u.spim,
                                &config_spim,
                                handler ? spim_evt_handler : NULL,
                                (void *)inst_idx);
        }
#endif
    }
    return result;
}

