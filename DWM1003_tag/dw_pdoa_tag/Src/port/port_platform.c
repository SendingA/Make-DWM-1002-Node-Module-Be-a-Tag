/*! ----------------------------------------------------------------------------
 * @file    port_platform.c
 * @brief   HW specific definitions and functions for portability
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

#include "port_platform.h"
#include "deca_device_api.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "dw_pdoa_tag_common.h"
#include "sdk_config.h"
#include "nrf_drv_gpiote.h"

/******************************************************************************
 *
 *                              APP global variables
 *
 ******************************************************************************/

spi_handle_t spiA_handler = {0};

static spi_handle_t *pgSpiHandler = &spiA_handler;

int gRangingStart = 0;

dw_t dw_chip_A
=
{
    .irqPin    = DW1000_IRQ_A_Pin,
    .rstPin    = DW1000_RST_A_Pin,
    .wkupPin   = DW1000_WUP_A_Pin,
    .pSpi      = &spiA_handler,
};

const dw_t *pDwMaster = &dw_chip_A; /**< by default chip 0 (A) is the "MASTER" */
uint32_t time32_incr = 0;
uint32_t timer_val = 0;

static uint32_t UART_timeout;
static bool UART_is_down = false;

int readfromspi_uni(uint16 headerLength,
                    const uint8 *headerBuffer,
                    uint32 readlength,
                    uint8 *readBuffer,
                    spi_handle_t *pgSpiHandler);

int writetospi_uni(uint16 headerLength,
                   const uint8 *headerBuffer,
                   uint32 bodylength,
                   const uint8 *bodyBuffer,
                   spi_handle_t *pgSpiHandler);

void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context);

static int port_init_device(dw_name_e chip_id);


/******************************************************************************
 *
 *                              Time section
 *
 ******************************************************************************/

/* @fn    portGetTickCnt
 * @brief wrapper for to read a SysTickTimer, which is incremented with
 *        CLOCKS_PER_SEC frequency.
 *        The resolution of time32_incr is usually 1/1000 sec.
 * */
__INLINE uint32_t
portGetTickCount(void)
{
    return time32_incr;
}

/* @brief     manually configuring and enabling of EXTI priority
 * */
void init_dw1000_irq(void)
{
    /* DW_IRQ_A_IRQn interrupt configuration for DW_MASTER */
    NVIC_SetPriority((IRQn_Type)pDwMaster->irqPin, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
//    enable_dw1000_irq();
}

void disable_dw1000_irq(void)
{
    NVIC_DisableIRQ((IRQn_Type)pDwMaster->irqPin);
}

void enable_dw1000_irq(void)
{
    NVIC_EnableIRQ((IRQn_Type)pDwMaster->irqPin);
}

/* @fn      reset_DW1000
 * @brief   DW_RESET pin on DW1000 has 2 functions
 *          In general it is output, but it also can be used to reset the
 *          digital part of DW1000 by driving this pin low.
 *          Note, the DW_RESET pin should not be driven high externally.
 * */
void reset_DW1000(void)
{
    nrf_gpio_cfg_output(DW1000_RST_A_Pin);   
    nrf_gpio_pin_clear(DW1000_RST_A_Pin);  
    nrf_delay_us(200); 
    nrf_gpio_cfg_input(DW1000_RST_A_Pin, NRF_GPIO_PIN_NOPULL); 
    nrf_delay_ms(7); 
}

/* @fn      port_wakeup_dw1000
 * @brief   "slow" waking up of DW1000 using DW_CS only
 * */
void port_wakeup_dw1000(void)
{
    nrf_gpio_pin_clear(DW1000_WUP_A_Pin);
    nrf_delay_ms(1);
    nrf_gpio_pin_set(DW1000_WUP_A_Pin);
    nrf_delay_ms(7);
}


void port_disable_wake_init_dw(void)
{
    taskENTER_CRITICAL();         

    pDwMaster = &dw_chip_A;

    disable_dw1000_irq();             /**< disable NVIC IRQ until we configure the device */

    port_reinit_dw_chips();

    //this is called here to wake up the device (i.e. if it was in sleep mode before the restart)
    port_wakeup_dw1000();  

    if (port_init_device(DW_MASTER) != 0x00)
    {
        error_handler(1,  _ERR_INIT);
    }

    taskEXIT_CRITICAL();
}

void port_reinit_dw_chips(void)
{
    nrf_gpio_pin_set(DW1000_CS_A_Pin);
    nrf_gpio_cfg_output(DW1000_CS_A_Pin);

    nrf_gpio_pin_clear(DW1000_WUP_A_Pin);

     /* Setup DW1000 IRQ pin for Master Chip A */
    nrf_gpio_cfg_input(DW1000_IRQ_A_Pin, NRF_GPIO_PIN_PULLDOWN);     //irq


    nrf_gpio_pin_set(DW1000_RST_A_Pin);
    nrf_delay_ms(50);
    nrf_gpio_cfg_input(DW1000_RST_A_Pin, NRF_GPIO_PIN_NOPULL);

}

void init_SPI_master()
{
    nrf_drv_spi_t   *spi_inst;
    nrf_drv_spi_config_t  *spi_config;

    spi_inst = &spiA_handler.spi_inst;
    spi_config = &spiA_handler.spi_config;

    spi_inst->inst_idx = SPI0_INSTANCE_INDEX;
    spi_inst->use_easy_dma = SPI0_USE_EASY_DMA;
    spi_inst->u.spim.p_reg = NRF_SPIM0;
    spi_inst->u.spim.drv_inst_idx = NRFX_SPIM0_INST_IDX;

    spiA_handler.frequency_slow = NRF_DRV_SPI_FREQ_2M;
    spiA_handler.frequency_fast = NRF_DRV_SPI_FREQ_8M;

    spi_config->sck_pin = SPI0_CONFIG_SCK_PIN;
    spi_config->mosi_pin = SPI0_CONFIG_MOSI_PIN;
    spi_config->miso_pin = SPI0_CONFIG_MISO_PIN;
    spi_config->ss_pin = SPI_0_CS_PIN;
    spi_config->irq_priority = APP_IRQ_PRIORITY_MID;
    spi_config->orc = 0xFF;
    spi_config->frequency = NRF_DRV_SPI_FREQ_2M;
    spi_config->mode = NRF_DRV_SPI_MODE_0;
    spi_config->bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

    spiA_handler.lock = DW_HAL_TAG_UNLOCKED;
}

void set_SPI_master(void)
{
    pgSpiHandler = pDwMaster->pSpi;
    dwt_setlocaldataptr(0);
}

void port_init_dw_chips(void)
{
    init_SPI_master();
}


///* @fn      port_set_dw1000_slowrate_first_time
// * @brief   set 2MHz
// *          n
// * */
//void port_set_dw1000_slowrate_first_time(dw_name_e chip_id)
//{
//
//    if (chip_id == DW_MASTER)
//    {
//        set_SPI_master();
//
//    }
//
//    APP_ERROR_CHECK( nrf_drv_spi_init(&pgSpiHandler->spi_inst, 
//                                      &pgSpiHandler->spi_config, 
//                                      spi_event_handler,
//                                      NULL) );
//    nrf_delay_ms(2);
//
//}

void set_dw_spi_slow_rate(dw_name_e chip_id)
{

    if (chip_id == DW_MASTER)
    {
        set_SPI_master();
        pgSpiHandler->spi_config.frequency = spiA_handler.frequency_slow;

    }

    if( pgSpiHandler->spi_init_stat == SPI_SPEED_SLOW)
    {
        return;
    }
    else 
    {
        if(pgSpiHandler->spi_init_stat == SPI_SPEED_FAST )
        {

            nrf_drv_spi_uninit(&pgSpiHandler->spi_inst);
        }

        APP_ERROR_CHECK( nrf_drv_spi_init(&pgSpiHandler->spi_inst, 
                                          &pgSpiHandler->spi_config, 
                                          spi_event_handler,
                                          NULL) );
        pgSpiHandler->spi_init_stat = SPI_SPEED_SLOW;

    nrf_delay_ms(2);
}

}

/* @fn      set_dw_spi_fast_rate
 * @brief   
 * */
void set_dw_spi_fast_rate(dw_name_e chip_id)
{

    if (chip_id == DW_MASTER)
    {
        set_SPI_master();
        pgSpiHandler->spi_config.frequency = spiA_handler.frequency_fast;

    }

    if(pgSpiHandler->spi_init_stat == SPI_SPEED_FAST )
    {
        return;
    }
    else 
    {
        if(pgSpiHandler->spi_init_stat == SPI_SPEED_SLOW )
        {
            nrf_drv_spi_uninit(&pgSpiHandler->spi_inst);
        }

        APP_ERROR_CHECK( nrf_drv_spi_init(&pgSpiHandler->spi_inst, 
                                          &pgSpiHandler->spi_config, 
                                          spi_event_handler,
                                          NULL) );
        pgSpiHandler->spi_init_stat = SPI_SPEED_FAST;

    nrf_delay_us(100);
    }

}

/**
 *  @brief     Bare-metal level
 *          initialise master/slave DW1000 (check if can talk to device and wake up and reset)
 */
static int
port_init_device(dw_name_e chip_id)
{
    set_dw_spi_slow_rate(chip_id);

    //this is called here to wake up the device (i.e. if it was in sleep mode before the restart)
    uint32   devID0 = dwt_readdevid() ;

    if(DWT_DEVICE_ID != devID0) //if the read of device ID fails, the DW1000 could be asleep
    {
        port_wakeup_dw1000();

        devID0 = dwt_readdevid();
        // SPI not working or Unsupported Device ID
        if(DWT_DEVICE_ID != devID0)
            return (-1) ;
    }
    //clear the sleep bit in case it is set - so that after the hard reset below the DW does not go into sleep
    dwt_softreset();

    return 0;
}

void port_stop_all_UWB(void)
{
    decaIrqStatus_t s = decamutexon();

    set_dw_spi_slow_rate(DW_MASTER);

    dwt_forcetrxoff();

    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_ARFE | DWT_INT_RFSL |\
                       DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO, 0);

    dwt_softreset();

    dwt_setcallbacks(NULL, NULL, NULL, NULL);

    decamutexoff(s);
}

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
}

//==============================================================================

void close_spi(nrf_drv_spi_t *p_instance)
{

    NRF_SPIM_Type * p_spi = p_instance->u.spim.p_reg;
    nrf_spim_disable(p_spi);
}

void open_spi(nrf_drv_spi_t *p_instance)
{

    NRF_SPIM_Type * p_spi = p_instance->u.spim.p_reg;
    nrf_spim_enable(p_spi);
}

int readfromspi(uint16 headerLength,
                const uint8 *headerBuffer,
                uint32 readlength,
                uint8 *readBuffer)
{
    return readfromspi_uni(headerLength, headerBuffer,
                            readlength, readBuffer, pgSpiHandler);

}

int writetospi( uint16 headerLength,
                const uint8 *headerBuffer,
                uint32 bodylength,
                const uint8 *bodyBuffer)
{
    return writetospi_uni(headerLength, headerBuffer,
                          bodylength, bodyBuffer, pgSpiHandler);

}


int readfromspi_uni(uint16 headerLength,
                    const uint8 *headerBuffer,
                    uint32 readlength,
                    uint8 *readBuffer,
                    spi_handle_t *pgSpiHandler)
{
    uint8 idatabuf[DATALEN1]={0};
    uint8 itempbuf[DATALEN1]={0};

    uint8 * p1;
    uint32 idatalength=0;

    while(pgSpiHandler->lock);

    __HAL_LOCK(pgSpiHandler);

    open_spi(&pgSpiHandler->spi_inst);

    memset(idatabuf, 0, DATALEN1);
    memset(itempbuf, 0, DATALEN1);

    p1=idatabuf;
    memcpy(p1,headerBuffer, headerLength);

    p1 += headerLength;
    memset(p1,0x00,readlength);

    idatalength= headerLength + readlength;

    spi_xfer_done = false;
    nrf_drv_spi_transfer(&pgSpiHandler->spi_inst, idatabuf, idatalength, itempbuf, idatalength);
    while(!spi_xfer_done);
    p1=itempbuf + headerLength;
    memcpy(readBuffer, p1, readlength);

    close_spi(&pgSpiHandler->spi_inst);

    __HAL_UNLOCK(pgSpiHandler);

    return 0;
}

int writetospi_uni(uint16 headerLength,
                   const uint8 *headerBuffer,
                   uint32 bodylength,
                   const uint8 *bodyBuffer,
                   spi_handle_t *pgSpiHandler)
{
    uint8 idatabuf[DATALEN1]={0};
    uint8 itempbuf[DATALEN1]={0};

    uint8 * p1;
    uint32 idatalength=0;

    while(pgSpiHandler->lock);

    __HAL_LOCK(pgSpiHandler);

    open_spi(&pgSpiHandler->spi_inst);

    memset(idatabuf, 0, DATALEN1);
    memset(itempbuf, 0, DATALEN1);

    p1=idatabuf;
    memcpy(p1,headerBuffer, headerLength);
    p1 += headerLength;
    memcpy(p1,bodyBuffer,bodylength);

    idatalength= headerLength + bodylength;

    spi_xfer_done = false;
    nrf_drv_spi_transfer(&pgSpiHandler->spi_inst, idatabuf, idatalength, itempbuf, idatalength);
    while(!spi_xfer_done);

    close_spi(&pgSpiHandler->spi_inst);

     __HAL_UNLOCK(pgSpiHandler);

    return 0;
}

/**@brief Systick handler
 *
 * @param[in] void
 */
void SysTick_Handler (void) {
        time32_incr++;
}
/******************************************************************************
 *
 *                              END OF Time section
 *
 ******************************************************************************/

/******************************************************************************
 *
 *                          DW1000 port section
 *
 ******************************************************************************/

void port_SPIx_set_chip_select()
{
    nrf_gpio_pin_set(DW1000_CS_A_Pin);
}


void port_SPIx_clear_chip_select()
{
    nrf_gpio_pin_clear(DW1000_CS_A_Pin);
}

#define NOMINAL_RSTN_TIME   (2200)
#define WAKEUP_TMR_US   (10000-NOMINAL_RSTN_TIME)

/* @fn      port_wakeup_dw1000_fast
 * @brief   waking up of DW1000 using DW_CS and DW_RESET pins.
 *          The DW_RESET signalling that the DW1000 is in the INIT state.
 *          the total fast wakeup takes ~2.2ms and depends on crystal startup
 *          time
 * */
error_e port_wakeup_dw1000_fast(void)
{

    uint32_t x = 0;
    uint32_t timestamp = 0; //protection
    error_e  ret = _Err_Timeout;
 
    // reenable interrupt pin - was disabled to conserve 480uA during sleep
    nrf_drv_gpiote_in_event_enable(DW1000_IRQ_A_Pin, true);

    port_SPIx_clear_chip_select();  //CS low

    /* need to poll to check when the DW1000 is in the IDLE, the CPLL interrupt
       is not reliable
       when RSTn goes high the DW1000 is in INIT, it will enter IDLE after PLL
       lock (in 5 us) */
    // usually it takes 1.3-1.7ms to RSTn to go high - to minimize uncertainty lets wait for the fixed time before (nominal+10%)
    nrf_delay_us(NOMINAL_RSTN_TIME);   
    while((timestamp) < WAKEUP_TMR_US)
    {
        //when DW1000 will switch to an IDLE state RSTn pin will high
        if ( nrf_gpio_pin_read(DW1000_RST_A_Pin) )
        {
            ret = _NO_ERR;
            break;
        }
        nrf_delay_us(10); 
        timestamp += 10;
    }

    port_SPIx_set_chip_select();    //CS high
    /* it takes ~35us in total for the DW1000 to lock the PLL, download AON and
       go to IDLE state */
    nrf_delay_us(35);

    return ret;
}

void deca_sleep(unsigned int time_ms)
{
    nrf_delay_ms(time_ms);
}

/**@brief timer_event_handler
 *
 * @param[in] void
 */
void timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    timer_val++;
    // Enable SysTick Interrupt
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
}
/******************************************************************************
 *
 *                          End APP port section
 *
 ******************************************************************************/



/******************************************************************************
 *
 *                              IRQ section
 *
 ******************************************************************************/
/*! ----------------------------------------------------------------------------
 * Function: decamutexon()
 *
 * Description: This function should disable interrupts.
 *
 *
 * input parameters: void
 *
 * output parameters: uint16
 * returns the state of the DW1000 interrupt
 */

decaIrqStatus_t decamutexon(void)
{
    uint32_t s = NVIC_GetPendingIRQ((IRQn_Type)DW1000_IRQ_A_Pin);
    if(s)
    {
        NVIC_DisableIRQ((IRQn_Type)DW1000_IRQ_A_Pin);
    }
    return 0;
}
/*! ----------------------------------------------------------------------------
 * Function: decamutexoff()
 *
 * Description: This function should re-enable interrupts, or at least restore
 *              their state as returned(&saved) by decamutexon
 * This is called at the end of a critical section
 *
 * input parameters:
 * @param s - the state of the DW1000 interrupt as returned by decamutexon
 *
 * output parameters
 *
 * returns the state of the DW1000 interrupt
 */
void decamutexoff(decaIrqStatus_t s)
{
    if(s)
    {
        NVIC_EnableIRQ((IRQn_Type)DW1000_IRQ_A_Pin);
    }
}

/* @fn      port_CheckEXT_IRQ
 * @brief   wrapper to read DW_IRQ input pin state
 * */
uint32_t port_CheckEXT_IRQ(void)
{
    return nrf_gpio_pin_read(DW1000_IRQ_A_Pin);
}

/* @fn      process_deca_irq
 * @brief   main call-back for processing of DW1000 IRQ
 *          it re-enters the IRQ routing and processes all events.
 *          After processing of all events, DW1000 will clear the IRQ line.
 * */
void process_deca_irq(void)
{
    set_SPI_master();
    set_dw_spi_fast_rate(DW_MASTER);

    while(port_CheckEXT_IRQ() != 0)
    {
        dwt_isr();

    } //while DW1000 IRQ line active
    if(app.DwCanSleep)
    {
        dwt_entersleep();
        nrf_drv_gpiote_in_event_disable(DW1000_IRQ_A_Pin); // saving 480uA during sleep
    }
}


/* @fn    GetLPtimerTickCount
 * @brief wrapper for to read a timer_tick_val, which is incremented by the RTC ISR
 *        The resolution of timer_tick_val is roughly 1/1000 sec.
 * */
__STATIC_INLINE uint32_t GetLPtimerTickCount(void)
{
    return (uint32_t)xTaskGetTickCount();
}

/**
 * @brief Renew current timestamp
 * @param [out] p_timestamp - poiner to timestamp instance
 */
void start_timer(uint32_t *p_timestamp)
{
  *p_timestamp = GetLPtimerTickCount();
}

/**
 * @brief Check if timeout is expired
 * @param [in] timestamp -instance
 * @param [in] time - timeout to check
 * @return true - timeout is expired
 * false - timeout is not yet expired
 */
bool check_timer(uint32_t timestamp, uint32_t time)
{
  bool res = false;

  uint32_t temp_tick_time = GetLPtimerTickCount();

  uint32_t time_passing;
  if (temp_tick_time >= timestamp)
  {
    time_passing = temp_tick_time - timestamp;
  }
  else
  {
    time_passing = 0xffffffff - timestamp + temp_tick_time;
  }
  if (time_passing >= time)
  {
    res = true;
  }
  return res;
}

__STATIC_INLINE void nrf_gpio_cfg_input_sense_low(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config)
{
    nrf_gpio_cfg(
        pin_number,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        pull_config,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_SENSE_LOW);
}


__STATIC_INLINE void nrf_gpio_cfg_input_sense_high(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config)
{
    nrf_gpio_cfg(
        pin_number,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        pull_config,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_SENSE_HIGH);
}

__STATIC_INLINE void nrf_gpio_cfg_input_sense_none(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config)
{
    nrf_gpio_cfg(
        pin_number,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        pull_config,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_NOSENSE);
}

///* Define the function that is called by portSUPPRESS_TICKS_AND_SLEEP(). */

void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
{
    /*
     * Implementation note:
     *
     * To help debugging the option configUSE_TICKLESS_IDLE_SIMPLE_DEBUG was presented.
     * This option would make sure that even if program execution was stopped inside
     * this function no more than expected number of ticks would be skipped.
     *
     * Normally RTC works all the time even if firmware execution was stopped
     * and that may lead to skipping too much of ticks.
     */

    TickType_t enterTime;

    /* Make sure the SysTick reload value does not overflow the counter. */
    if ( xExpectedIdleTime > portNRF_RTC_MAXTICKS - configEXPECTED_IDLE_TIME_BEFORE_SLEEP )
    {
        xExpectedIdleTime = portNRF_RTC_MAXTICKS - configEXPECTED_IDLE_TIME_BEFORE_SLEEP;
    }
    /* Block all the interrupts globally */
#ifdef SOFTDEVICE_PRESENT
    do{
        uint8_t dummy = 0;
        uint32_t err_code = sd_nvic_critical_region_enter(&dummy);
        APP_ERROR_CHECK(err_code);
    }while (0);
#else
    __disable_irq();
#endif

    if ( UART_is_down ) {
    }else{
      if ( check_timer(UART_timeout, UART_OFF_TIMEOUT) ){
          app_uart_close();
          // RX_PIN_NUMBER is now latched 
          nrf_gpio_cfg_input_sense_low(UART_0_RX_PIN, NRF_GPIO_PIN_NOPULL);
          UART_is_down = true;
      }
    }

    enterTime = nrf_rtc_counter_get(portNRF_RTC_REG);

    if ( eTaskConfirmSleepModeStatus() != eAbortSleep )
    {
        TickType_t xModifiableIdleTime;
        TickType_t wakeupTime = (enterTime + xExpectedIdleTime) & portNRF_RTC_MAXTICKS;

        /* Stop tick events */
        nrf_rtc_int_disable(portNRF_RTC_REG, NRF_RTC_INT_TICK_MASK);

        /* Configure CTC interrupt */
        nrf_rtc_cc_set(portNRF_RTC_REG, 0, wakeupTime);
        nrf_rtc_event_clear(portNRF_RTC_REG, NRF_RTC_EVENT_COMPARE_0);
        nrf_rtc_int_enable(portNRF_RTC_REG, NRF_RTC_INT_COMPARE0_MASK);

        __DSB();

        /* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
         * set its parameter to 0 to indicate that its implementation contains
         * its own wait for interrupt or wait for event instruction, and so wfi
         * should not be executed again.  However, the original expected idle
         * time variable must remain unmodified, so a copy is taken. */
        xModifiableIdleTime = xExpectedIdleTime;
        configPRE_SLEEP_PROCESSING( xModifiableIdleTime );
        if ( xModifiableIdleTime > 0 )
        {
#if 0  // With FreeRTOS sd_app_evt_wait increases power consumption with FreeRTOS compared to _WFE (NRFFOSDK-11174)
#ifdef SOFTDEVICE_PRESENT
            if (nrf_sdh_is_enabled())
            {
                uint32_t err_code = sd_app_evt_wait();
                APP_ERROR_CHECK(err_code);
            }
            else
#endif
#endif // (NRFFOSDK-11174)
        NVIC_ClearPendingIRQ(RTC0_IRQn);
        NVIC_ClearPendingIRQ(RTC1_IRQn);

// errata #87, 3.12 Errata_v1.6
#if (__FPU_USED == 1)
            __set_FPSCR(__get_FPSCR() & ~(0x0000009F));
            (void) __get_FPSCR();
            NVIC_ClearPendingIRQ(FPU_IRQn);
#endif // errata #87, 3.12 Errata_v1.6
            {
                /* No SD -  we would just block interrupts globally.
                * BASEPRI cannot be used for that because it would prevent WFE from wake up.
                */
                do{
                    __WFI();
                } while (0 == (NVIC->ISPR[0] | NVIC->ISPR[1]));
            }
        }

        configPOST_SLEEP_PROCESSING( xExpectedIdleTime );

        nrf_rtc_int_disable(portNRF_RTC_REG, NRF_RTC_INT_COMPARE0_MASK);
        nrf_rtc_event_clear(portNRF_RTC_REG, NRF_RTC_EVENT_COMPARE_0);

        /* Correct the system ticks */
        {
            TickType_t diff;
            TickType_t exitTime;

            nrf_rtc_event_clear(portNRF_RTC_REG, NRF_RTC_EVENT_TICK);
            nrf_rtc_int_enable (portNRF_RTC_REG, NRF_RTC_INT_TICK_MASK);

            exitTime = nrf_rtc_counter_get(portNRF_RTC_REG);
            diff =  (exitTime - enterTime) & portNRF_RTC_MAXTICKS;

            /* It is important that we clear pending here so that our corrections are latest and in sync with tick_interrupt handler */
            NVIC_ClearPendingIRQ(portNRF_RTC_IRQn);

            if ((configUSE_TICKLESS_IDLE_SIMPLE_DEBUG) && (diff > xExpectedIdleTime))
            {
                diff = xExpectedIdleTime;
            }

            if (diff > 0)
            {
                vTaskStepTick(diff);
            }
        }
    }
#ifdef SOFTDEVICE_PRESENT
    uint32_t err_code = sd_nvic_critical_region_exit(0);
    APP_ERROR_CHECK(err_code);
#else

    if ( UART_is_down ) {
      if ( nrf_gpio_pin_latch_get(UART_0_RX_PIN) ) {
          nrf_gpio_cfg_input_sense_none(UART_0_RX_PIN, NRF_GPIO_PIN_NOPULL);
          nrf_gpio_pin_latch_clear(UART_0_RX_PIN);
          UART_is_down = false;
          Restart_UART_timer();
          deca_uart_init();
          app_uart_flush();
          deca_discard_next_symbol();
      }
    }

  __enable_irq();
#endif
}

void Restart_UART_timer()
{
     start_timer( &UART_timeout );
}


/******************************************************************************
 *
 *                              END OF IRQ section
 *
 ******************************************************************************/
