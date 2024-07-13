/*! ----------------------------------------------------------------------------
 * @file    dw_pdoa_tag_common.c
 * @brief   Defines Common functionalities of PDOA Tag Application
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
//#include "app_usbd_cdc_acm.h"

#include "nrf_drv_twi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_uart.h"
#include "nrf_drv_wdt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_clock.h"

#include "port_platform.h"

#include "dw_pdoa_tag_common.h"
#include "imusensor_10dof_interface.h"

const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(0); /**< Declaring an instance of nrf_drv_rtc for RTC0. */

nrf_drv_twi_t _twi = NRF_DRV_TWI_INSTANCE(1);  
nrf_drv_twi_config_t twi_conf = {
    .frequency = NRF_TWI_FREQ_400K,
    .scl = ARDUINO_SCL_PIN,
    .sda = ARDUINO_SDA_PIN,
    .clear_bus_init=true,
    .hold_bus_uninit=true
};

#define SPI_INSTANCE  2 /**< SPI instance index. */
const nrf_drv_spi_t imu_spi_inst = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);
nrf_drv_spi_config_t imu_spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

void interrupts_init(void);
void wdt_init(void);
static void imusensor_twi_init(void);
static void imusensor_spi_init(void);

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
    switch(irqPin)
    {
      case DW1000_IRQ_A_Pin:
        process_deca_irq();
        break;
    }
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

  /* Configure board. */
   bsp_board_init(BSP_INIT_LEDS);

#ifndef ENABLE_USB_PRINT
    ret = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(ret);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\n\rDeca Test Example......");
    NRF_LOG_FLUSH();
#endif

    interrupts_init();
    deca_uart_init();
    Restart_UART_timer();

    /*WDT Initilization*/
    wdt_init();

#if defined(PDOA_V2_BOARD) && defined(IMU_SPI_ENABLE)
      imusensor_spi_init();
#else
      imusensor_twi_init();
#endif

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

    err_code = nrf_drv_gpiote_in_init(DW1000_RST_A_Pin, &in_config, deca_irq_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(DW1000_IRQ_A_Pin, true);

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

/* @fn  sensor_init
 *
 * @param[in] void
 * */
static void imusensor_twi_init(void)
{
   ret_code_t err_code;

   /* Init I2C for the intertial sensors */
   err_code = nrf_drv_twi_init(&_twi, &twi_conf, NULL, NULL);
   APP_ERROR_CHECK(err_code);
}

static void imusensor_spi_init(void)
{
    nrf_gpio_cfg_input(LSM6DSL_LPS22HB_SDO_PIN, NRF_GPIO_PIN_PULLUP);
	
    nrf_gpio_cfg_output(LSM6DSL_CS_PIN);nrf_gpio_pin_set(LSM6DSL_CS_PIN);
    nrf_gpio_cfg_output(LIS2MDL_CS_PIN);nrf_gpio_pin_set(LIS2MDL_CS_PIN);
    nrf_gpio_cfg_output(LPS22HB_CS_PIN);nrf_gpio_pin_set(LPS22HB_CS_PIN);

    imu_spi_config.sck_pin  = SPI2_CONFIG_SCK_PIN;
    imu_spi_config.mosi_pin = SPI2_CONFIG_MOSI_PIN;
    imu_spi_config.miso_pin = LSM6DSL_LPS22HB_SDO_PIN;
    imu_spi_config.mode = NRF_DRV_SPI_MODE_3;
    imu_spi_config.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;
    imu_spi_config.frequency = NRF_DRV_SPI_FREQ_8M;

    APP_ERROR_CHECK(nrf_drv_spi_init(&imu_spi_inst, &imu_spi_config, NULL, NULL));
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

// To Test Low power mode - Set configUSE_IDLE_HOOK as '1' in FreeRTOSConfig.h
void vApplicationIdleHook( void )
{
    __WFI();
}

/* Added for pdoa_tag application */

nrfx_err_t nrfx_rtc_reinit(nrfx_rtc_t const * const  p_instance,
                         nrfx_rtc_config_t const * p_config,
                         nrfx_rtc_handler_t        handler)
{
    NRFX_ASSERT(p_config);
    NRFX_ASSERT(handler);
    nrfx_err_t err_code;

    rtc_handlers[p_instance->instance_id] = handler;

    NRFX_IRQ_PRIORITY_SET(p_instance->irq, p_config->interrupt_priority);
    NRFX_IRQ_ENABLE(p_instance->irq);
    nrf_rtc_prescaler_set(p_instance->p_reg, p_config->prescaler);
    rtc_cb[p_instance->instance_id].reliable     = p_config->reliable;
    rtc_cb[p_instance->instance_id].tick_latency = p_config->tick_latency;
    rtc_cb[p_instance->instance_id].state        = NRFX_DRV_STATE_INITIALIZED;

    err_code = NRFX_SUCCESS;
    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}
