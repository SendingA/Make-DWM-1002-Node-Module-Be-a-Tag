/*
 * @file       test_fn.c
 * @brief      Production Test Application
 *             Used to test components of Node
 *
 * @author     Decawave Software
 *
 * @attention  Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *             All rights reserved.
 *
 */


#include "port_platform.h"
#include "dw_pdoa_node_common.h"
#include "nrf_drv_twi.h"
#include "nrf_uart.h"
#include "bsp.h"

#include "lsm6dsl.h"
#include "lsm6dsr.h"
#include "lps22hb.h"
#include "lis2mdl.h"

#include "app_timer.h"
#include "nrf_drv_clock.h"

#include "test_fn.h"
#include "cmd_fn.h"
#include "nrf_drv_wdt.h"
#include "deca_regs.h"

#define SPI_INSTANCE  0 /**< SPI instance index. */

extern char m_tx_buffer[NRF_DRV_USBD_EPSIZE];
extern nrf_drv_wdt_channel_id m_channel_id;

bool gProd_test_enable = 0;
volatile bool gPush_button_flag = 0;
uint8_t gTimeout = 0;  /** Timeout for button press test based on RTC timer **/

static const uint8_t bsp_led_list[LEDS_NUMBER] = LEDS_LIST;

nrf_drv_twi_t imu_twi_inst = NRF_DRV_TWI_INSTANCE(1);
nrf_drv_twi_config_t imu_twi_cfg =
{
    .frequency = NRF_TWI_FREQ_400K,
    .scl = ARDUINO_SCL_PIN,
    .sda = ARDUINO_SDA_PIN,
    .clear_bus_init=true,
    .hold_bus_uninit=true
};

static const nrf_drv_spi_t imu_spi_inst = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);
static nrf_drv_spi_config_t imu_spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

void i2c_init()
{
    APP_ERROR_CHECK(nrf_drv_twi_init(&imu_twi_inst, &imu_twi_cfg, NULL, NULL));
}

void i2c_uninit()
{
    nrf_drv_twi_uninit(&imu_twi_inst);
}

void spi_init()
{
    imu_spi_config.sck_pin  = SPIM2_SCK_PIN;
    imu_spi_config.mosi_pin = SPIM2_MOSI_PIN;
    imu_spi_config.miso_pin = LSM6DSL_LPS22HB_SDO_PIN;
    imu_spi_config.mode = NRF_DRV_SPI_MODE_3;
    imu_spi_config.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;
    imu_spi_config.frequency = NRF_DRV_SPI_FREQ_4M;

    APP_ERROR_CHECK(nrf_drv_spi_init(&imu_spi_inst, &imu_spi_config, NULL, NULL));
}

static void test_spi2_three_wire_read(int en)
{
    nrf_drv_spi_config_t spi_read_config = NRF_DRV_SPI_DEFAULT_CONFIG;

    spi_read_config.sck_pin   = SPIM2_SCK_PIN;
    spi_read_config.miso_pin  = SPIM2_MOSI_PIN;
    spi_read_config.mosi_pin  = NRF_DRV_SPI_PIN_NOT_USED;
    spi_read_config.ss_pin    = NRF_DRV_SPI_PIN_NOT_USED;
    spi_read_config.mode      = NRF_DRV_SPI_MODE_3;
    spi_read_config.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;
    spi_read_config.frequency = NRF_DRV_SPI_FREQ_4M;

    nrf_gpio_cfg_input(SPIM2_SCK_PIN, NRF_GPIO_PIN_PULLUP);
    nrf_drv_spi_uninit(&imu_spi_inst);

    if (en) {
        /* Reconfig spi for reading from 3wire */
        nrf_drv_spi_reinit(&imu_spi_inst, &spi_read_config, NULL, NULL);
    } else {
        /* Normal 4 wire config */
        nrf_drv_spi_reinit(&imu_spi_inst, &imu_spi_config, NULL, NULL);
    }
}

error_e test_dwt_read_devID(char *str, dw_name_e chip_id)
{
    int        devID = 0;
    error_e    retVal = _NO_ERR;

    if(chip_id == 0)
    {
        sprintf(str, "\n\n\r DW1000 Test - Chip-A");
    }
    else
    {
        sprintf(str, "\n\n\r DW1000 Test - Chip-B");
    }

    set_dw_spi_slow_rate(chip_id);

    devID = dwt_readdevid();

    sprintf(&str[strlen(str)], "\n\r Decawave DeviceID: %x", devID);

    if(devID == 0xdeca0130)
    {
        sprintf(&str[strlen(str)], "\n\r DW1000 Test Pass");
    }
    else
    {
        sprintf(&str[strlen(str)], "\n\r DW1000 Test Fail");
        retVal = _Err;
    }

    port_tx_msg((uint8_t*)str, strlen(str));
    osDelay(100);

    return retVal;
}

error_e test_10DOFSensor_read_devID_i2c_spi(char *str, bool i2c_enable)
{
    int status = 0;
    ret_code_t err_code;
    error_e retVal = _NO_ERR;

    if(i2c_enable == 1)
    {
        sprintf(str, "\n\n\r 10DOF Sensor Test - I2C");
    }
    else
    {
        sprintf(str, "\n\n\r 10DOF Sensor Test - SPI");
    }

    port_tx_msg((uint8_t*)str, strlen(str));
    osDelay(100);

    nrf_gpio_cfg_input(LSM6DSL_LPS22HB_SDO_PIN, NRF_GPIO_PIN_PULLUP);
	
    if(i2c_enable)
    {
        nrf_gpio_cfg_input(LSM6DSL_LPS22HB_SDO_PIN, NRF_GPIO_PIN_PULLUP);
    }
    else
    {
        nrf_gpio_cfg_input(LSM6DSL_LPS22HB_SDO_PIN, NRF_GPIO_PIN_PULLUP);
    }

    nrf_gpio_cfg_output(LSM6DSL_CS_PIN);nrf_gpio_pin_set(LSM6DSL_CS_PIN);
    nrf_gpio_cfg_output(LIS2MDL_CS_PIN);nrf_gpio_pin_set(LIS2MDL_CS_PIN);
    nrf_gpio_cfg_output(LPS22HB_CS_PIN);nrf_gpio_pin_set(LPS22HB_CS_PIN);

    if(i2c_enable)
    {
        i2c_init();
    }
    else
    {
	spi_init();
    }
	
    struct lps22hb_cfg lps_cfg = LPS22HB_CFG_DEFAULTS;

    if(i2c_enable)
    {
        lps_cfg.spi = 0;
        lps_cfg.twi = &imu_twi_inst;
        lps_cfg.twi_addr = LPS22HB_I2C_ADDR;
    }
    else
    {
        lps_cfg.spi = &imu_spi_inst;
        lps_cfg.spi_cs_pin = LPS22HB_CS_PIN;
    }

    lps22hb_init(&lps_cfg);

    struct lis2mdl_cfg lis_cfg = LIS2MDL_CFG_DEFAULTS;

    if(i2c_enable)
    {
        lis_cfg.spi = 0;
        lis_cfg.twi = &imu_twi_inst;
        lis_cfg.twi_addr = LIS2MDL_I2C_ADDR;
    }
    else
    {
        lis_cfg.spi = &imu_spi_inst;
        lis_cfg.spi_rd_cb = test_spi2_three_wire_read;
        lis_cfg.spi_cs_pin = LIS2MDL_CS_PIN;
    }

    lis2mdl_init(&lis_cfg);

    struct lsm6dsl_cfg lsm_cfg = LSM6DSL_CFG_DEFAULTS;

    if(i2c_enable)
    {
        lsm_cfg.spi = 0;
        lsm_cfg.twi = &imu_twi_inst;
        lsm_cfg.twi_addr = LSM6DSL_I2C_ADDR;
    }
    else
    {
        lsm_cfg.spi = &imu_spi_inst;
        lsm_cfg.spi_cs_pin = LSM6DSL_CS_PIN;
    }
    lsm6dsl_init(&lsm_cfg);

    if (lps22hb_hwtest())
    {
        status = 1;

        sprintf(str, "\n\r lps22hb Sensor Test Fail");
    }
    else
    {

        sprintf(str, "\n\r lps22hb Sensor Test Pass");
    }

    if (lsm6dsl_hwtest())
    {
        status = 1;

        sprintf(&str[strlen(str)], "\n\r lsm6dsl Sensor Test Fail");
    }
    else
    {
        sprintf(&str[strlen(str)], "\n\r lsm6dsl Sensor Test Pass");
    }
	
    if (lis2mdl_hwtest())
    {
        status = 1;

        sprintf(&str[strlen(str)], "\n\r lis2mdl Sensor Test Fail");
    }
    else
    {
        sprintf(&str[strlen(str)], "\n\r lis2mdl Sensor Test Pass");
    }

    if(status == 0)
    {
        sprintf(&str[strlen(str)], "\n\r Sensor Test Pass\n");
    }
    else
    {
        sprintf(&str[strlen(str)], "\n\r Sensor Test Fail\n");
        retVal = _Err;
    }

    port_tx_msg((uint8_t*)str, strlen(str));
    osDelay(100);

    if(i2c_enable)
    {
        i2c_uninit();
    }
    else
    {
        nrf_drv_spi_uninit(&imu_spi_inst);
    }

    return retVal;
}

void dwt_setleds_on_test()
{
    uint32 reg;

    reg = dwt_read32bitoffsetreg(GPIO_CTRL_ID, GPIO_MODE_OFFSET);
    reg &= ~(GPIO_MSGP2_MASK | GPIO_MSGP3_MASK);
    reg |= 0x0;
    dwt_write32bitoffsetreg(GPIO_CTRL_ID, GPIO_MODE_OFFSET, reg);

    dwt_setgpiodirection(GxM2, 0x0);
    dwt_setgpiovalue(GxM2, 0x4);

    dwt_setgpiodirection(GxM3, 0x0);
    dwt_setgpiovalue(GxM3, 0x8);
}

void dwt_setleds_off_test()
{
    dwt_setgpiodirection(GxM2, 0x0);
    dwt_setgpiovalue(GxM2, 0x0);

    dwt_setgpiodirection(GxM3, 0x0);
    dwt_setgpiovalue(GxM3, 0x0);
}

void test_led_on(char *str)
{
    uint32_t i;

    sprintf(str, "\n\n\r LED Test");

    sprintf(&str[strlen(str)], "\n\n\r All LED's ON");

    sprintf(&str[strlen(str)], "\n\n\r Delay for 1sec - To Check LED Output\n\r");

    port_tx_msg((uint8_t*)str, strlen(str));
    osDelay(100);

    for (i = 0; i < LEDS_NUMBER; ++i)
    {
        nrf_gpio_pin_write(bsp_led_list[i], 1);
    }

    set_SPI_master();
    dwt_setleds_on_test();
    set_SPI_slave();
    dwt_setleds_on_test();

    nrf_delay_ms(1000);
}

void test_led_off(char *str)
{
    uint32_t i;

    sprintf(str, "\n\n\r All LED's OFF");

    sprintf(&str[strlen(str)], "\n\n\r Delay for 1sec - To Check LED Output\n\r");

    port_tx_msg((uint8_t*)str, strlen(str));

    osDelay(100);

    for (i = 0; i < LEDS_NUMBER; ++i)
    {
        nrf_gpio_pin_write(bsp_led_list[i], 0);
    }

    set_SPI_master();
    dwt_setleds_off_test();
    set_SPI_slave();
    dwt_setleds_off_test();

    nrf_delay_ms(1000);
}

void bsp_event_callback(bsp_event_t event)
{
    switch (event)
    {
        case BSP_EVENT_KEY_0:
            gPush_button_flag = 1;
            gTimeout = 0;
    }
}

error_e test_leds(char *str)
{
     error_e    retVal = _NO_ERR;
     test_led_on(str);

     sprintf(str, "\n\n\r Press Button to turn off All LED's");
     port_tx_msg((uint8_t*)str, strlen(str));
     osDelay(100);

     gTimeout = 1;

     while(!gPush_button_flag)
     {
        nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh
        if(gTimeout >= 50)                         // 5 seconds timeout
        {
            retVal = _Err;
            gTimeout = 0;

            sprintf(str, "\n\n\r Button Test failed");
            sprintf(&str[strlen(str)], "\n\n\r Explicitly turning off the LED's");
            port_tx_msg((uint8_t*)str, strlen(str));
            osDelay(100);
			
            break;
        }
     }
     gPush_button_flag = 0;

     test_led_off(str);

     return retVal;
}

void test_nfc(char *str)
{
    uint32_t err_code = NRF_SUCCESS;
    uint32_t gpio_toggle_flag = 0;

    sprintf(str, "\n\n\r NFC Test");

    nrf_drv_clock_lfclk_request(NULL);

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    sprintf(&str[strlen(str)], "\n\n\r Delay - To Check NFC Output\n\r");
    sprintf(&str[strlen(str)], "\n\n\r Press Button to stop NFC test");

    port_tx_msg((uint8_t*)str, strlen(str));
    osDelay(100);

    gTimeout = 1;

    while ( true )
    {
        if(gpio_toggle_flag == 0)
        {
            nrf_gpio_cfg_input(NFC_PIN_1, NRF_GPIO_PIN_PULLDOWN);
            nrf_gpio_cfg_input(NFC_PIN_2, NRF_GPIO_PIN_PULLDOWN);
            gpio_toggle_flag =  1;
        }
        else
        {
            nrf_gpio_cfg_input(NFC_PIN_1, NRF_GPIO_PIN_PULLUP);
            nrf_gpio_cfg_input(NFC_PIN_2, NRF_GPIO_PIN_PULLUP);
            gpio_toggle_flag =  0;
        }

        nrf_delay_ms(500);

        if(gPush_button_flag)  // checking for key press
        {
            gPush_button_flag = false;
            break;
        }

        if(gTimeout >= 20)               // 2 seconds timeout
        {
            gTimeout = 0;
            break;
        }

        nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh
    }
}

void test_uart(char *str)
{
    ret_code_t ret;
    uint8_t rx_var;
    uint32_t err_code;

    sprintf(str, "\n\n\r UART Test");

    sprintf(&str[strlen(str)], "\n\r Enter input in UART console ");

    sprintf(&str[strlen(str)], "\n\n\r Delay - To Check UART Output\n\r");

    sprintf(&str[strlen(str)], "\n\n\r Press Button to stop UART test");

    port_tx_msg((uint8_t*)str, strlen(str));
    osDelay(100);

    gTimeout = 1;

    while(!gPush_button_flag)
    {
       nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh
       osDelay(100);

       if(gTimeout >= 20)               // 2 seconds timeout
        {
            gTimeout = 0;
            break;
        }
    }
    gPush_button_flag = false;
}

void test_reset(char *str)
{
    sprintf(str, "\n\n\r To do Reset Test press reset button");

    sprintf(&str[strlen(str)], "\n\n\r Delay for 1sec to do reset test");

    port_tx_msg((uint8_t*)str, strlen(str));
    osDelay(100);

    nrf_delay_ms(1000);

    sprintf(str, "\n\n\r End of Reset Test");

    port_tx_msg((uint8_t*)str, strlen(str));
    osDelay(100);

}

/* @fn      test_process_init
 * @brief   Function to initialise peripherals for production test.
 *
 * */
error_e test_process_init()
{
    error_e    ret = _NO_ERR;
    error_e    err_code;

    port_init_dw_chips();

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = bsp_init(BSP_INIT_BUTTONS, bsp_event_callback);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_buttons_enable();
    APP_ERROR_CHECK(err_code);

    return (ret);
}

/* @fn        test_prod_tests
 * @brief     The "test_prod_tests" function implements Production test Functionality -
 *            DW1000, MEMS Sensor, LED, NFC, UART, RESET
 * 
 */
void test_prod_tests(void)
{
   error_e    retVal = _NO_ERR;
   bool       i2c_enable;
   char *str = CMD_MALLOC(MAX_STR_SIZE);

    retVal |= test_dwt_read_devID(str, DW_MASTER);

    retVal |= test_dwt_read_devID(str, DW_SLAVE);

    i2c_enable = 1;
    retVal |= test_10DOFSensor_read_devID_i2c_spi(str, i2c_enable);

    i2c_enable = 0;
    retVal |= test_10DOFSensor_read_devID_i2c_spi(str, i2c_enable);

    retVal |= test_leds(str);

    if (retVal != _NO_ERR)
    {
        sprintf(str, "\n\n\r One of the Test Failed");       // Blue LED
        nrf_gpio_pin_write(bsp_led_list[0], 1);
    }
    else
    {
        sprintf(str, "\n\n\r All Tests Passed");             // Green LED
        nrf_gpio_pin_write(bsp_led_list[2], 1);
    }

    sprintf(&str[strlen(str)], "\n\n\r End of Test");
    port_tx_msg((uint8_t*)str, strlen(str));
    osDelay(100);

    CMD_FREE(str);
}

/* @fn        test_process_run
 * @brief     The "test_process_run" function implements Production test Functionality
 *            Run all the Production Tests
 *
 */
void test_process_run(void)
{
    gProd_test_enable = 1;
    gPush_button_flag = 0;

    test_prod_tests();
}

/* @fn     test_process_terminate
 *
 */
void test_process_terminate(void)
{
    gProd_test_enable = 0;
    error_e    ret = _NO_ERR;
}
