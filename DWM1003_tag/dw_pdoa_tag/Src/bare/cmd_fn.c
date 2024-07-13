/*
 * @file     cmd_fn.c
 * @brief    collection of executables functions from defined known_commands[]
 *
 * @author   Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#include <cmd.h>
#include <cmd_fn.h>
#include <config.h>
#include <translate.h>
#include <usb_uart_tx.h>
#include <version.h>
#include "deca_version.h"

#include "event_groups.h"

#include "string.h"

//-----------------------------------------------------------------------------
const char CMD_FN_RET_OK[] = "ok\r\n";

#if !defined(DYNAMIC_STR_DEBUG_ME)
char staticstr[MAX_STR_SIZE];
#endif

/****************************************************************************//**
 *
 *                          f_xx "command" FUNCTIONS
 *
 * REG_FN(f_tag) macro will create a function
 *
 * const char *f_tag(char *text, param_block_t *pbss, int val)
 *
 * */

//-----------------------------------------------------------------------------
// Operation Mode change section

/* @brief    helperTask will start Tag
 * app.mode will be mTwr
 * */
REG_FN(f_tag)
{
    xEventGroupSetBits(app.xStartTaskEvent, Ev_Tag_Task);
    return (CMD_FN_RET_OK);
}

/* @brief    helperTask will start
 * USB2SPI thread using chip A
 * app.mode will be mUspi
 * */
REG_FN(f_uspi)
{
    xEventGroupSetBits(app.xStartTaskEvent, Ev_Usb2spi_A_Task);
    return(CMD_FN_RET_OK);
}

/* @brief    helperTask will start
 * TCWM thread using chip A
 * app.mode will be mTcwm
 *
 * */
REG_FN(f_tcwm)
{
    xEventGroupSetBits(app.xStartTaskEvent, Ev_Tcwm_A_Task);
    return (CMD_FN_RET_OK);
}

/* @brief    helperTask will start
 * 0 - TCFM thread using chip A
 * app.mode will be mTcfm
 * */
REG_FN(f_tcfm)
{
    xEventGroupSetBits(app.xStartTaskEvent, Ev_Tcfm_A_Task);
    return (CMD_FN_RET_OK);
}

/* @brief    helperTask will stop all working threads
 * app.mode will be mIDLE
 * */
REG_FN(f_stop)
{
    reset_FlushTask();

    command_stop_received();
    return (CMD_FN_RET_OK);
}

//communication to the user application
void command_stop_received(void)
{
    xEventGroupSetBits(app.xStartTaskEvent, Ev_Stop_All);
}
//-----------------------------------------------------------------------------
// Parameters change section : allowed only in app.mode = mIdle

REG_FN(f_ant_tx_a)
{
    pbss->s.ant_tx_a = (uint16_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_ant_rx_a)
{
    pbss->s.ant_rx_a = (uint16_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_uart)
{
    pbss->s.uartEn = (uint8_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_autos)
{
    pbss->s.autoStartEn = (uint8_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_twr_report)
{
    pbss->s.reportLevel = (uint8_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_smart_tx_en)
{
    pbss->s.smartTxEn = (uint8_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_dbg)
{
    pbss->s.debugEn = (uint8_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_faulty)
{
    pbss->s.faultyRanges = (uint8_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_rc_delay)
{
    pbss->s.rcDelay_us = (uint16_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_rc_timeout)
{
    pbss->s.rcRxTo_us = (uint16_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_acc_threshold)
{
    pbss->s.acc_threshold = (uint16_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_acc_stat_sense)
{
    pbss->s.acc_stationary_ms = (uint16_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_acc_moving_sens)
{
    pbss->s.acc_moving_ms = (uint16_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_emuEVB)
{
    pbss->s.emuEVB = (uint8_t)(val);
    return (CMD_FN_RET_OK);
}

//-----------------------------------------------------------------------------
//Service/debug commands
REG_FN(f_system_prf_mode)
{
    if(val == 16)
    {
        pbss->dwt_config.prf = DWT_PRF_16M;
        pbss->dwt_config.rxCode = 3;
        pbss->dwt_config.txCode = 3;
    }
    else if(val == 64)
    {
        pbss->dwt_config.prf = DWT_PRF_64M;
        pbss->dwt_config.rxCode = 9;
        pbss->dwt_config.txCode = 9;
    }
    return (CMD_FN_RET_OK);
}
//-----------------------------------------------------------------------------
// Communication /  user statistics section

/* @brief
 * */
REG_FN(f_decaPDOA)
{
    const char *ret = NULL;
    const char ver[] = FULL_VERSION;

    char *str = CMD_MALLOC(MAX_STR_SIZE);

    if(str)
    {
        int  hlen;

        hlen = sprintf(str,"JS%04X", 0x5A5A);    // reserve space for length of JS object

        sprintf(&str[strlen(str)],"{\"Info\":{\r\n");
        sprintf(&str[strlen(str)],"\"Device\":\"PDOA Tag\",\r\n");
        sprintf(&str[strlen(str)],"\"Version\":\"%s\",\r\n", ver);
        sprintf(&str[strlen(str)],"\"Build\":\"%s %s\",\r\n", __DATE__, __TIME__ );
        sprintf(&str[strlen(str)],"\"Driver\":\"%s\"}}", DW1000_DEVICE_DRIVER_VER_STRING );

        sprintf(&str[2],"%04X",strlen(str)-hlen);   //add formatted 4X of length, this will erase first '{'
        str[hlen]='{';                            //restore the start bracket
        port_tx_msg((uint8_t*)str, strlen(str));
        port_tx_msg((uint8_t*)"\r\n", 2);

        CMD_FREE(str);
        ret = CMD_FN_RET_OK;
    }

    return (ret);
}

//-----------------------------------------------------------------------------


/*
 * @brief   show current mode of operation,
 *          version, and the configuration in JSON format
 *          Should be executed from STOP as
 *
 * */
REG_FN(f_jstat)
{
    const char *ret = NULL;

    char *str = CMD_MALLOC(MAX_STR_SIZE);

    if(str)
    {
        CMD_ENTER_CRITICAL();

        int  hlen;

        /* System Config object */
        hlen = sprintf(str,"JS%04X", 0x5A5A);    // reserve space for length of JS object
        sprintf(&str[strlen(str)],"{\"System Config\":{\r\n");
        sprintf(&str[strlen(str)],"\"F_NUM\":%d,\r\n",pbss->s.faultyRanges);
        sprintf(&str[strlen(str)],"\"RCDEL\":%d,\r\n",pbss->s.rcDelay_us);
        sprintf(&str[strlen(str)],"\"RCTO\":%d,\r\n",pbss->s.rcRxTo_us);
        sprintf(&str[strlen(str)],"\"SMTX\":%d}}",pbss->s.smartTxEn);

        sprintf(&str[2],"%04X",strlen(str)-hlen);//add formatted 4X of length, this will erase first '{'
        str[hlen]='{';                            //restore the start bracket
        sprintf(&str[strlen(str)],"\r\n");
        port_tx_msg((uint8_t*)str, strlen(str));

        /* Run Time object */
        hlen = sprintf(str,"JS%04X", 0x5A5A);    // reserve space for length of JS object
        sprintf(&str[strlen(str)],"{\"Run Time\":{\r\n");
        sprintf(&str[strlen(str)],"\"UART\":%d,\r\n",pbss->s.uartEn);
        sprintf(&str[strlen(str)],"\"AUTO\":%d,\r\n",pbss->s.autoStartEn);
        sprintf(&str[strlen(str)],"\"PCREP\":%d}}",pbss->s.reportLevel);

        sprintf(&str[2],"%04X",strlen(str)-hlen);//add formatted 4X of length, this will erase first '{'
        str[hlen]='{';                            //restore the start bracket
        sprintf(&str[strlen(str)],"\r\n");
        port_tx_msg((uint8_t*)str, strlen(str));

        /* Calibration object */
        hlen = sprintf(str,"JS%04X", 0x5A5A);    // reserve space for length of JS object
        sprintf(&str[strlen(str)],"{\"Calibration\":{\r\n");
        sprintf(&str[strlen(str)],"\"ANTTX\":%d,\r\n",pbss->s.ant_tx_a);
        sprintf(&str[strlen(str)],"\"ANTRX\":%d,\r\n",pbss->s.ant_rx_a);
        sprintf(&str[strlen(str)],"\"ACCTHR\":%d,\r\n",pbss->s.acc_threshold);
        sprintf(&str[strlen(str)],"\"ACCSTAT\":%d,\r\n",pbss->s.acc_stationary_ms);
        sprintf(&str[strlen(str)],"\"ACCMOVE\":%d}}",pbss->s.acc_moving_ms);

        sprintf(&str[2],"%04X",strlen(str)-hlen);//add formatted 4X of length, this will erase first '{'
        str[hlen]='{';                            //restore the start bracket
        sprintf(&str[strlen(str)],"\r\n");
        port_tx_msg((uint8_t*)str, strlen(str));

        CMD_FREE(str);

        CMD_EXIT_CRITICAL();

        ret = CMD_FN_RET_OK;
    }
    return (ret);
}


/*
 * @brief   show current UWB parameters in JSON format
 *
 * */
REG_FN(f_uwb)
{
    const char *ret = NULL;

    char *str = CMD_MALLOC(MAX_STR_SIZE);

    if(str)
    {
        int  hlen;

        hlen = sprintf(str,"JS%04X", 0x5A5A);    // reserve space for length of JS object
        sprintf(&str[strlen(str)],"{\"UWB PARAM\":{\r\n");

        sprintf(&str[strlen(str)],"\"CHAN\":%d,\r\n",deca_to_chan(pbss->dwt_config.chan));
        sprintf(&str[strlen(str)],"\"PRF\":%d,\r\n", deca_to_prf (pbss->dwt_config.prf));
        sprintf(&str[strlen(str)],"\"PLEN\":%d,\r\n",deca_to_plen(pbss->dwt_config.txPreambLength));
        sprintf(&str[strlen(str)],"\"PAC\":%d,\r\n", deca_to_pac (pbss->dwt_config.rxPAC));
        sprintf(&str[strlen(str)],"\"TXCODE\":%d,\r\n",pbss->dwt_config.txCode);
        sprintf(&str[strlen(str)],"\"DATARATE\":%d}}",deca_to_bitrate(pbss->dwt_config.dataRate));

        sprintf(&str[2],"%04X",strlen(str)-hlen);//add formatted 4X of length, this will erase first '{'
        str[hlen]='{';                            //restore the start bracket
        sprintf(&str[strlen(str)],"\r\n");
        port_tx_msg((uint8_t*)str, strlen(str));

        CMD_FREE(str);
        ret = CMD_FN_RET_OK;
    }
    return (ret);
}

/*
 * @brief show current mode of operation,
 *           version, and the configuration
 *
 * */
REG_FN(f_stat)
{
    const char * ret = CMD_FN_RET_OK;

    char *str = CMD_MALLOC(MAX_STR_SIZE);

    if(str)
    {
        sprintf(str,"MODE: %s\r\n"
                    "LAST ERR CODE: %d\r\n"
                    "MAX MSG LEN: %d\r\n",
          ((app.mode==mIDLE)?("STOP"):
          (app.mode==mTWR)?( "TAG"):
          (app.mode==mTCWM)?("TCWM"):
          (app.mode==mTCFM)?("TCFM"):
          (app.mode==mUSB2SPI)?("USB2SPI"):
          ("unknown")),
          app.lastErrorCode,
          app.maxMsgLen);

        port_tx_msg((uint8_t*)str, strlen(str));

        CMD_FREE(str);

        app.lastErrorCode = 0;
        app.maxMsgLen = 0;

        f_decaPDOA(text, pbss, val);
        f_jstat(text, pbss, val);
    }

    ret = CMD_FN_RET_OK;
    return (ret);
}


REG_FN(f_help_app)
{
    int        indx = 0;
    const char * ret = NULL;
    char *str = CMD_MALLOC(MAX_STR_SIZE);

    if(str)
    {
        while (known_commands[indx].name != NULL)
        {
            sprintf(str,"%s \r\n", known_commands[indx].name);

            port_tx_msg((uint8_t*)str, strlen(str));

            indx++;
        }

        CMD_FREE(str);
        ret = CMD_FN_RET_OK;
    }

    return (ret);
}

//-----------------------------------------------------------------------------
// Communication change section

/*
 * @brief save configuration
 *
 * */
REG_FN(f_save)
{
    error_e    err_code;

    taskENTER_CRITICAL();
    err_code = save_bssConfig(pbss);
    taskEXIT_CRITICAL();

    if(err_code > 0)
    {
        error_handler(0, err_code);
        return (NULL);    //ERROR
    }

    return (CMD_FN_RET_OK);
}

REG_FN(f_restore)
{
    restore_bssConfig();
    return (CMD_FN_RET_OK);
}
//-----------------------------------------------------------------------------


/* @brief
 *  Send Service message from Tag:
 *  Currently sending to the PC Stationary flag and accelerometer's data.
 *
 *  'JSxxxx{"ST":
 *    {     "a16": %04X, //addr16 of the Tag
            "V":%d //service message from the Tag (stationary is bit 0)
            "X":%d //Normalized accel data X from the Tag, mg
            "Y":%d //Normalized accel data Y from the Tag, mg
            "Z":%d //Normalized accel data Z from the Tag, mg
 *    }
 *   }'
 * */
void send_to_pc_stationary(stationary_res_t *p)
{
    int hlen;

    char *str = CMD_MALLOC(MAX_STR_SIZE);

    if(str)
    {
        {
            /* use JSON type of output during a normal operation */
            hlen = sprintf(str,"JS%04X", 0x5A5A);    // reserve space for length of JS object

            sprintf(&str[strlen(str)],"{\"ST\": ");

            sprintf(&str[strlen(str)],
                    "{\"a16\":\"%04X\","
                    "\"V\":%d," //service message data from the Tag: (stationary is bit 0)
                    "\"X\":%d," //Normalized accel data X from the Tag, mg
                    "\"Y\":%d," //Normalized accel data Y from the Tag, mg
                    "\"Z\":%d"  //Normalized accel data Z from the Tag, mg
                    "}}",
                    (int)(p->addr),
                    (int)(p->flag),
                    (int)(p->acc_x),
                    (int)(p->acc_y),
                    (int)(p->acc_z));

            sprintf(&str[2],"%04X",strlen(str)-hlen);//add formatted 4X of length, this will kill first '{'
            str[hlen]='{';                           //restore the start bracket

            port_tx_msg((uint8_t*)str, strlen(str));
            port_tx_msg((uint8_t*)"\r\n", 2);
        }

        CMD_FREE(str);
    }
}

/*
 * @brief This is not a command, but just a report of twr from a tag back to pc/uart
 *
 * JSON (default):
 *  'JSxxxx{"TAR":
 *    {     "a16": %04X, //addr16: self
 *          "n16": %04X, //addr16: node
 *          "R":%d,"     //range number
 *          "Xcm":%f,    //node to tag X coord report, float as int32_t, cm
 *          "Ycm":%f     //node to tag Y coord report, float as int32_t, cm
 *          "O":%f       //node to tag offset report,float as int32_t, 1000 = 1ppm
 *    }
 *   }'
 * */


void send_to_pc_tag_location(twr_res_ext_t *p)
{

    int     hlen;
    char    *str = CMD_MALLOC(MAX_STR_SIZE);

    if(str)
    {
        {
            /* use JSON type of output during a normal operation */
            hlen = sprintf(str,"JS%04X", 0x5A5A);    // reserve space for length of JS object

            sprintf(&str[strlen(str)],"{\"TAR\": ");

            sprintf(&str[strlen(str)],
                    "{\"a16\":\"%04X\","
                    "\"n16\":\"%04X\","
                    "\"R\":%d," //range number
                    "\"Xcm\":%d," //X as int
                    "\"Ycm\":%d," //Y as int
                    "\"O\":%d"  //offset as int
                    "}}",
                    (int)(p->addr),
                    (int)(p->node_addr),
                    (int)(p->rNum),
                    (int)(p->x_cm),
                    (int)(p->y_cm),
                    (int)(p->clkOffset_pphm));

            sprintf(&str[2],"%04X",strlen(str)-hlen);   //add formatted 04X length, this will kill first '{'
            str[hlen]='{';                              //restore the start bracket

            port_tx_msg((uint8_t*)str, strlen(str));
            port_tx_msg((uint8_t*)"\r\n", 2);
        }

        CMD_FREE(str);
    }
}


/* end f_xx command functions */

//-----------------------------------------------------------------------------
/* list of known commands:
 * NAME, allowed_MODE,     REG_FN(fn_name)
 * */
const command_t known_commands []= {
    /* CMDNAME   MODE   fn     */
    /* 1. commands with configurable parameters */
    {"AUTO",    mIDLE,  f_autos},
    {"UART",    mIDLE,  f_uart},
    {"RCDEL",   mIDLE,  f_rc_delay},
    {"RCTO",    mIDLE,  f_rc_timeout},
    {"FNUM",    mIDLE,  f_faulty},
    {"ANTTX",   mIDLE,  f_ant_tx_a},
    {"ANTRX",   mIDLE,  f_ant_rx_a},
    {"ACCTHR",  mIDLE,  f_acc_threshold},
    {"ACCSTAT", mIDLE,  f_acc_stat_sense},
    {"ACCMOVE", mIDLE,  f_acc_moving_sens},

    /* 2. anytime commands */
    {"DECA$",   mANY,   f_decaPDOA},
    {"JSTAT",   mANY,   f_jstat},
    {"STOP",    mANY,   f_stop},
    {"STAT",    mANY,   f_stat},
    {"HELP",    mANY,   f_help_app},
    {"?",       mANY,   f_help_app},
    {"SAVE",    mANY,   f_save},

    /* 3. app start commands */
    {"TAG",     mIDLE,  f_tag},
    //{"USPI",    mIDLE,  f_uspi},    // This feature is not yet supported
    {"TCWM",    mIDLE,  f_tcwm},
    {"TCFM",    mIDLE,  f_tcfm},

    /* Service Commands */
    {"RESTORE", mIDLE,  f_restore},
    {"PCREP",   mIDLE,  f_twr_report},
    {"SMTX",    mIDLE,  f_smart_tx_en},
    {"DEBUG",   mIDLE,  f_dbg},
    {"SPRFMODE", mIDLE, f_system_prf_mode},
    {"EMUEVB",  mIDLE,  f_emuEVB},

    {NULL,      mANY,   NULL}
};
