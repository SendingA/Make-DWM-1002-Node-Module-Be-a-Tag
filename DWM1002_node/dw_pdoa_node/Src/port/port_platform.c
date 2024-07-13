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
#include "dw_pdoa_node_common.h"
#include "sdk_config.h"

/******************************************************************************
 *
 *                              APP global variables
 *
 ******************************************************************************/

static spi_handle_t spiA_handler = {0};
static spi_handle_t spiB_handler = {0};

static spi_handle_t *pgSpiHandler = &spiA_handler;

int gRangingStart = 0;

dw_t dw_chip_A
=
{
    .irqPin    = DW1000_IRQ_A_Pin,
    .rstPin    = DW1000_RST_A_Pin,
    .wkupPin   = DW1000_WUP_A_Pin,
    .csPin     = DW1000_CS_A_Pin,
    .pSpi      = &spiA_handler,
};

dw_t dw_chip_B
=
{
    .irqPin    = DW1000_IRQ_B_Pin,
    .rstPin    = DW1000_RST_B_Pin,
    .wkupPin   = DW1000_WUP_B_Pin,
    .csPin     = DW1000_CS_B_Pin,
    .pSpi      = &spiB_handler,
};

const dw_t *pDwMaster = &dw_chip_A; /**< by default chip 0 (A) is the "MASTER" */
const dw_t *pDwSlave  = &dw_chip_B; /**< by default chip 1 (B) is the "SLAVE" */

static volatile uint32_t signalResetDone;

static volatile bool spi_xfer_done;
static uint8 idatabuf[SPI_BUFFER_DATALEN];
static uint8 itempbuf[SPI_BUFFER_DATALEN];

uint32_t time32_incr = 0;
uint32_t timer_val = 0;

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

void disable_dw1000_irq(void)
{
    NVIC_DisableIRQ((IRQn_Type)pDwMaster->irqPin);
    NVIC_DisableIRQ((IRQn_Type)pDwSlave->irqPin);
}

void enable_dw1000_irq(void)
{
    NVIC_EnableIRQ((IRQn_Type)pDwMaster->irqPin);
    NVIC_EnableIRQ((IRQn_Type)pDwSlave->irqPin);
}

/* @fn      reset_DW1000
 * @brief   DW_RESET pin on DW1000 has 2 functions
 *          In general it is output, but it also can be used to reset the
 *          digital part of DW1000 by driving this pin low.
 *          Note, the DW_RESET pin should not be driven high externally.
 * */
void reset_DW1000(dw_name_e chip_id)
{
    uint32 reset_gpio_pin;
    
    if (chip_id == DW_MASTER)
    {
        reset_gpio_pin = pDwMaster->rstPin;
    }
    else
    {
        reset_gpio_pin = pDwSlave->rstPin;
    }
    nrf_gpio_pin_clear(reset_gpio_pin);
    nrf_gpio_cfg_output(reset_gpio_pin);
    nrf_delay_ms(50);
    nrf_gpio_cfg_input(reset_gpio_pin, NRF_GPIO_PIN_NOPULL);
    nrf_delay_ms(5);
}

/* @fn      port_wakeup_dw1000
 * @brief   "slow" waking up of DW1000 using DW_CS only
 * */
void port_wakeup_dw1000(dw_name_e chip_id)
{
    uint32 wkup_gpio_pin;
    
    if (chip_id == DW_MASTER)
    {
        wkup_gpio_pin = pDwMaster->wkupPin;
    }
    else
    {
        wkup_gpio_pin = pDwSlave->wkupPin;
    }

    nrf_gpio_pin_clear(wkup_gpio_pin);
    nrf_delay_ms(1);
    nrf_gpio_pin_set(wkup_gpio_pin);
    nrf_delay_ms(7);
}

void port_set_master_chip(dw_name_e master_chip)
{
    if(master_chip == DW_MASTER )
    {
        pDwMaster = &dw_chip_A;
        pDwSlave  = &dw_chip_B;
    }
    else
    {
        pDwMaster = &dw_chip_B;
        pDwSlave  = &dw_chip_A;
    }
}

void port_disable_wake_init_dw(void)
{
    taskENTER_CRITICAL();         

    disable_dw1000_irq();             /**< disable NVIC IRQ until we configure the device */

    port_reinit_dw_chips();

    //this is called here to wake up the device (i.e. if it was in sleep mode before the restart)
    port_wakeup_dw1000(DW_MASTER);  
    port_wakeup_dw1000(DW_SLAVE);   

    if( (port_init_device(DW_SLAVE) != 0x00) || \
        (port_init_device(DW_MASTER) != 0x00))
    {
        error_handler(1,  _ERR_INIT);
    }

    taskEXIT_CRITICAL();
}

void port_reinit_dw_chips(void)
{
    nrf_gpio_pin_set(DW1000_CS_A_Pin);
    nrf_gpio_cfg_output(DW1000_CS_A_Pin);

    nrf_gpio_pin_set(DW1000_CS_B_Pin);
    nrf_gpio_cfg_output(DW1000_CS_B_Pin);

    nrf_gpio_pin_clear(DW1000_WUP_A_Pin);
    nrf_gpio_pin_clear(DW1000_WUP_B_Pin);

    /* Setup DW1000 IRQ pin for Master Chip A */
    nrf_gpio_cfg_input(pDwMaster->irqPin, NRF_GPIO_PIN_PULLDOWN);     //irq

    nrf_gpio_cfg_input(pDwSlave->irqPin, NRF_GPIO_PIN_NOPULL);     //irq

    nrf_gpio_pin_set(DW1000_RST_A_Pin);
//    nrf_delay_ms(50);
    nrf_gpio_cfg_input(DW1000_RST_A_Pin, NRF_GPIO_PIN_NOPULL);

    nrf_gpio_pin_set(DW1000_RST_B_Pin);
//    nrf_delay_ms(50);
    nrf_gpio_cfg_input(DW1000_RST_B_Pin, NRF_GPIO_PIN_NOPULL);

}

void init_SPI_master()
{
// master uses SPI3
    nrf_drv_spi_t   *spi_inst;
    nrf_drv_spi_config_t  *spi_config;

    spi_handle_t *pSPImaster_handler = pDwMaster->pSpi;

    spi_inst = &pSPImaster_handler->spi_inst;
    spi_config = &pSPImaster_handler->spi_config;
    spi_inst->inst_idx = SPI3_INSTANCE_INDEX;
    spi_inst->use_easy_dma = SPI3_USE_EASY_DMA;
    spi_inst->u.spim.p_reg = NRF_SPIM3;
    spi_inst->u.spim.drv_inst_idx = NRFX_SPIM3_INST_IDX;

    pSPImaster_handler->frequency_slow = NRF_SPIM_FREQ_2M;
    pSPImaster_handler->frequency_fast = NRF_SPIM_FREQ_16M;

    spi_config->sck_pin = DW1000_CLK_Pin;
    spi_config->mosi_pin = DW1000_MOSI_Pin;
    spi_config->miso_pin = DW1000_MISO_Pin;
    spi_config->ss_pin = pDwMaster->csPin;
    spi_config->irq_priority = (APP_IRQ_PRIORITY_MID - 2);
    spi_config->orc = 0xFF;
    spi_config->frequency = NRF_SPIM_FREQ_2M;
    spi_config->mode = NRF_DRV_SPI_MODE_0;
    spi_config->bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

    pSPImaster_handler->lock = DW_HAL_NODE_UNLOCKED;
}

void init_SPI_slave()
{
// master uses SPI2
    nrf_drv_spi_t   *spi_inst;
    nrf_drv_spi_config_t  *spi_config;

    spi_handle_t *pSPIslave_handler = pDwSlave->pSpi;

    spi_inst = &pSPIslave_handler->spi_inst;
    spi_config = &pSPIslave_handler->spi_config;

    spi_inst->inst_idx = SPI2_INSTANCE_INDEX;
    spi_inst->use_easy_dma = SPI2_USE_EASY_DMA;
    spi_inst->u.spim.p_reg = NRF_SPIM2;
    spi_inst->u.spim.drv_inst_idx = NRFX_SPIM2_INST_IDX;

    pSPIslave_handler->frequency_slow = NRF_SPIM_FREQ_2M;
    pSPIslave_handler->frequency_fast = NRF_SPIM_FREQ_8M;

    spi_config->sck_pin = DW1000_CLK_Pin;
    spi_config->mosi_pin = DW1000_MOSI_Pin;
    spi_config->miso_pin = DW1000_MISO_Pin;
    spi_config->ss_pin = pDwSlave->csPin;
    spi_config->irq_priority = (APP_IRQ_PRIORITY_MID - 1);
    spi_config->orc = 0xFF;
    spi_config->frequency = NRF_SPIM_FREQ_2M;
    spi_config->mode = NRF_DRV_SPI_MODE_0;
    spi_config->bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

    pSPIslave_handler->lock = DW_HAL_NODE_UNLOCKED;
}

void set_SPI_master(void)
{
    pgSpiHandler = pDwMaster->pSpi;
    dwt_setlocaldataptr(0);
}

void set_SPI_slave(void)
{
    pgSpiHandler = pDwSlave->pSpi;
    dwt_setlocaldataptr(1);
}

void port_init_dw_chips(void)
{
    init_SPI_master();
    init_SPI_slave();
}


/* @fn      set_dw_spi_slow_rate
 * @brief   set 2MHz
 *          n
 * */

void set_dw_spi_slow_rate(dw_name_e chip_id)
{

    if (chip_id == DW_MASTER)
    {
        set_SPI_master();
    }
    else
    {
        set_SPI_slave();
    }
    pgSpiHandler->spi_config.frequency = pgSpiHandler->frequency_slow;

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
 * @brief   set 2MHz
 *          n
 * */
void set_dw_spi_fast_rate(dw_name_e chip_id)
{

    if (chip_id == DW_MASTER)
    {
        set_SPI_master();
    }
    else
    {
        set_SPI_slave();
    }
    pgSpiHandler->spi_config.frequency = pgSpiHandler->frequency_fast;

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
        nrf_delay_ms(2);
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
        port_wakeup_dw1000(chip_id);

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

    set_dw_spi_slow_rate(DW_SLAVE);
    dwt_forcetrxoff();
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_ARFE | DWT_INT_RFSL |\
                       DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO, 0);
    dwt_softreset();
    dwt_setcallbacks(NULL, NULL, NULL, NULL);

    set_dw_spi_slow_rate(DW_MASTER);
    dwt_forcetrxoff();
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_ARFE | DWT_INT_RFSL |\
                       DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO, 0);

    dwt_softreset();
    dwt_setcallbacks(NULL, NULL, NULL, NULL);

    decamutexoff(s);
}

//-----------------------------------------------------------------------------
// Sync section
/**
 * @brief Used to synchronise the system clocks of two DW1000 ICs
 */
void port_set_syncenable(int enable)
{
    nrf_gpio_cfg_output(DW1000_SYNC_EN_Pin);

    if(enable)
    {
        nrf_gpio_pin_write(DW1000_SYNC_EN_Pin, 1);
    }
    else
    {
        nrf_gpio_pin_write(DW1000_SYNC_EN_Pin, 0);
    }
}

void port_set_sync(int enable)
{
    nrf_gpio_cfg_output(DW1000_SYNC_Pin);

    if(enable)
    {
        nrf_gpio_pin_write(DW1000_SYNC_Pin, 1);
    }
    else
    {
        nrf_gpio_pin_write(DW1000_SYNC_Pin, 0);
    }
}

void port_set_syncclear(int enable)
{
    nrf_gpio_cfg_output(DW1000_SYNC_CLR_Pin);

    if(enable)
    {
        nrf_gpio_pin_write(DW1000_SYNC_CLR_Pin, 1);
    }
    else
    {
        nrf_gpio_pin_write(DW1000_SYNC_CLR_Pin, 0);
    }
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
    uint32 idatalength = headerLength + readlength;

    if ( idatalength > SPI_BUFFER_DATALEN ) {
      return NRF_ERROR_NO_MEM;
    }

    while(pgSpiHandler->lock);

    __HAL_LOCK(pgSpiHandler);

    open_spi(&pgSpiHandler->spi_inst);

    memcpy(idatabuf, headerBuffer, headerLength);

    spi_xfer_done = false;

    nrfx_spim_xfer_desc_t const spim_xfer_desc =
    {
        .p_tx_buffer = idatabuf,
        .tx_length   = idatalength,
        .p_rx_buffer = itempbuf,
        .rx_length   = idatalength,
    };
    nrfx_spim_xfer(&pgSpiHandler->spi_inst.u.spim, &spim_xfer_desc, 0);

    while(!spi_xfer_done);

    memcpy(readBuffer, itempbuf + headerLength, readlength);

    close_spi(&pgSpiHandler->spi_inst);

    __HAL_UNLOCK(pgSpiHandler);

    return 0;
}

int writetospi(uint16 headerLength,
                   const uint8 *headerBuffer,
                   uint32 bodylength,
                   const uint8 *bodyBuffer)
{
    uint32 idatalength = headerLength + bodylength;

    if ( idatalength > SPI_BUFFER_DATALEN ) {
      return NRF_ERROR_NO_MEM;
    }

    memcpy(idatabuf, headerBuffer, headerLength);
    memcpy(idatabuf + headerLength, bodyBuffer, bodylength);

    while(pgSpiHandler->lock);

    __HAL_LOCK(pgSpiHandler);

    open_spi(&pgSpiHandler->spi_inst);

    spi_xfer_done = false;

    nrfx_spim_xfer_desc_t const spim_xfer_desc =
    {
        .p_tx_buffer = idatabuf,
        .tx_length   = idatalength,
        .p_rx_buffer = NULL,
        .rx_length   = 0,
    };
    nrfx_spim_xfer(&pgSpiHandler->spi_inst.u.spim, &spim_xfer_desc, 0);

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
    uint32_t s = NVIC_GetPendingIRQ((IRQn_Type)pDwMaster->irqPin);
    if(s)
    {
        NVIC_DisableIRQ((IRQn_Type)pDwMaster->irqPin);
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
        NVIC_EnableIRQ((IRQn_Type)pDwMaster->irqPin);
    }
}

/* @fn      port_CheckEXT_IRQ
 * @brief   wrapper to read DW_IRQ input pin state
 * */
uint32_t port_CheckEXT_IRQ(void)
{
    return nrf_gpio_pin_read(pDwMaster->irqPin);
}

/* @fn      process_deca_irq
 * @brief   main call-back for processing of DW1000 IRQ
 *          it re-enters the IRQ routing and processes all events.
 *          After processing of all events, DW1000 will clear the IRQ line.
 * */
void process_deca_irq(void)
{
    while(port_CheckEXT_IRQ() != 0)
    {
        set_SPI_master();
        dwt_isr();

    } //while DW1000 IRQ line active
}

/******************************************************************************
 *
 *                              END OF IRQ section
 *
 ******************************************************************************/
