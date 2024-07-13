/*
 * @file     json_2pc.c
 * @brief    collection of JSON formatted functions which used
 *           to report from Node application to the PC
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */
#include "node.h"
#include "cmd_fn.h"
#include "task_imu.h"

/*
 * @brief function to report to PC a new tag was discovered
 *
 * 'JSxxxx{"NewTag":
 *             <string>//address64, string
 *        }'
 *
 * */
void signal_to_pc_new_tag_discovered(uint64_t addr64)
{
    char *str = CMD_MALLOC(MAX_STR_SIZE);

    if(str)
    {
        int  hlen;

        hlen = sprintf(str,"JS%04X", 0x5A5A);    // reserve space for length of JS object

        sprintf(&str[strlen(str)],"{\"NewTag\":\"%08lX%08lX\"}", (uint32_t)(addr64>>32), (uint32_t)addr64);

        sprintf(&str[2],"%04X",strlen(str)-hlen);//add formatted 4X of length, this will kill first '{'
        str[hlen]='{';                            //restore the start bracket

        sprintf(&str[strlen(str)],"\r\n");
        port_tx_msg((uint8_t*)str, strlen(str));

        CMD_FREE(str);
    }
}


/*
 * @brief This is a report of twr to pc
 *
 * There are two modes of operation: JSON(long output) or plain(short output)
 * JSON (default):
 *  'JSxxxx{"TWR":
 *    {     "a16":%04X, //addr16
 *          "R":%d,//range num
 *          "T":%d,//sys timestamp of Final WRTO Node's SuperFrame start, us
 *          "D":%f,//distance
 *          "P":%f,//raw pdoa
 *          "A":%f,//corrected angle
 *          "O":%f,//clock offset in hundreds part of ppm
 *          "V":%d //service message data from the tag: (stationary, etc)
 *          "X":%d //service message data from the tag: (stationary, etc)
 *          "Y":%d //service message data from the tag: (stationary, etc)
 *          "Z":%d //service message data from the tag: (stationary, etc)
 *    }
 *   }'
 *
 * Plain:
 * used if any from below is true:
 * diag, acc,
 * */
void send_to_pc_twr(result_t *pRes)
{
    char *str = CMD_MALLOC(MAX_STR_SIZE);
    int  hlen;

    if(str)
    {
        if (app.pConfig->s.accEn ==1 || \
            app.pConfig->s.diagEn ==1 || \
            app.pConfig->s.reportLevel > 1)
        {
            if(app.pConfig->s.reportLevel == 3)
            {
                /* shortest "AR" output: 18 chars per location: ~640 locations per second
                 * */
                sprintf(str, "AR%04X%02X%08lX%08lX",
                        (uint16_t)(pRes->addr16),
                        (uint8_t) (pRes->rangeNum),
                        (long int)(pRes->x_cm),
                        (long int)(pRes->y_cm));
            }
            else
            {
                /* optimum "RA" output: 58 chars per location: ~200 locations per second
                 * */
                sprintf(str, "RA%04X %02X %08lX %08lX %08lX %1X X:%04X Y:%04X Z:%04X",
                        (uint16_t)(pRes->addr16),
                        (uint8_t) (pRes->rangeNum),
                        (long int)(pRes->x_cm),
                        (long int)(pRes->y_cm),
                        (long int)(pRes->clockOffset_pphm),
                        (uint8_t) (pRes->flag),
                        (uint16_t)(pRes->acc_x),
                        (uint16_t)(pRes->acc_y),
                        (uint16_t)(pRes->acc_z));
            }
            sprintf(&str[strlen(str)],"\r\n");
            port_tx_msg((uint8_t*)str, strlen(str));
        }
        else if (app.pConfig->s.reportLevel == 1)
        {
            /* use JSON type of output during a normal operation
             *
             * This is not very efficient, as one TWR location is 107 chars, as per format below
             * JS  62{"TWR": {"a16":"2E5C","R":3,"T":8605,"D":343,"P":1695,"O":14,"V":1,"X":53015,"Y":60972,"Z":10797}}
             *
             * For pure UART, with limit of 115200b/s, the channel can handle ~100 locations per second,
             * i.e. 10 tags ranging on maximum rate of 10 times a second.
             * For higher throughput limit amount of JSON TWR object or use plain output instead.
             *
             * */

            /* Floating point values are standard for JSON objects, however the floating point printing
             * is not used in current application.
             * If the fixed point printing required, will need to add "-u _printf_float" to the
             * linker string, to include "floating printf" to the nano.spec of the stdlib.
             * This will increase the size of application by ~6kBytes and the floating printing also requires
             * much more stack space.
             * Use this with caution, as this might result unpredictable stack overflow / hard fault.
             * */
            hlen = sprintf(str,"JS%04X", 0x5A5A);    // reserve space for length of JS object

            sprintf(&str[strlen(str)],"{\"TWR\": ");

            sprintf(&str[strlen(str)],
                    "{\"a16\":\"%04X\","
                    "\"R\":%d," //range number
                    "\"T\":%d,",//sys timestamp of Final WRTO Node's SuperFrame start, us
                    (int)(pRes->addr16),
                    (int)(pRes->rangeNum),
                    (int)(pRes->resTime_us));

            if(app.pConfig->s.debugEn)
            {
                sprintf(&str[strlen(str)],
                    "\"Tm\":%d,"    //Master's temperature, in degree centigrade
                    "\"Ts\":%d,",   //Slave's temperature, in degree centigrade
                    (int)(pRes->tMaster_C),
                    (int)(pRes->tSlave_C));
            }

            sprintf(&str[strlen(str)],
                    "\"D\":%d," //distance as int
                    "\"P\":%d," //pdoa  as int in milli-radians
                    "\"Xcm\":%d,"   //X distance wrt Node in cm
                    "\"Ycm\":%d,",  //Y distance wrt Node in cm
                    (int)(pRes->dist_cm),
                    (int)(pRes->pdoa_raw_deg),
                    (int)(pRes->x_cm),
                    (int)(pRes->y_cm));

            sprintf(&str[strlen(str)],
                    "\"O\":%d,"//clock offset as int
                    "\"V\":%d," //service message data from the tag: (bitmask: bit0 = stationary, bit15 = zeroed pdoaOffset used; bit14 = zeroed rngOffset used)
                    "\"X\":%d," //Normalized accel data X from the Tag, mg
                    "\"Y\":%d," //Normalized accel data Y from the Tag, mg
                    "\"Z\":%d"  //Normalized accel data Z from the Tag, mg
                    "}",
                    (int)(pRes->clockOffset_pphm),
                    (int)(pRes->flag),
                    (int)(pRes->acc_x),
                    (int)(pRes->acc_y),
                    (int)(pRes->acc_z));

            sprintf(&str[strlen(str)],"}");

            sprintf(&str[2],"%04X",strlen(str)-hlen);//add formatted 4X of length, this will kill first '{'
            str[hlen]='{';                            //restore the start bracket

            sprintf(&str[strlen(str)],"\r\n");
            port_tx_msg((uint8_t*)str, strlen(str));
        }
        else
        {
            //no output
        }

        CMD_FREE(str);
    }
}


/* @brief input "str" must be a null-terminated string with enough space in it.
 *        this is slow output of accumulator from the chip, starting with ACC_OFFSET value.
 *        To be used solely for debug purposes.
 * */
static void send_acc(char       *str,
                     uint16_t   maxLen,
                     uint8_t    *pAcc)
{
    int     n;
    int16   cmplex_m[2];

    cmplex_m[0] = 0;
    cmplex_m[1] = 0;

    n = strlen(str);

    for(int i = 1; i <= (FULL_ACC_LEN*4); i+=4)
    {
        if(n >= (maxLen - 4))
        {
            while(port_tx_msg((uint8_t*)str, n) !=_NO_ERR)
            {
                osThreadYield();//force switch content
                osDelay(5);     //wait 5ms for Flush thread freed the buffer
            }
            n = 0;
        }

        if(i > (ACC_OFFSET*4))
        {
            memcpy(&cmplex_m[0], &pAcc[i], 4);
        }

        n += sprintf(&str[n], "%04X%04X", (cmplex_m[0] & 0xFFFF), (cmplex_m[1]&0xFFFF));
    }

    n += sprintf(&str[n], "\r\n");

    while(port_tx_msg((uint8_t*)str, n) !=_NO_ERR)
    {
        osThreadYield();    //force switch content
        osDelay(5);         //wait 5ms for Flush thread freed the buffer
    }
}

/* @brief input "str" must be a null-terminated string with enough space in it.
 *        To be used solely for debug purposes.
 * */
static void send_diag(char    *str,
                      uint16_t maxLen,
                      uint8_t *pDiag,
                      uint8_t *acc5,
                      uint8_t sfdangle)
{
    if ((strlen(str)+ 2*sizeof(diag_v5_t) + 1 + 10 + 6) < maxLen)
    {
        for(int i=0; i<sizeof(diag_v5_t); i++)
        {
            sprintf(&str[strlen(str)], "%02X", pDiag[i]);
        }

        sprintf(&str[strlen(str)], " ");

        for(int i=0; i<5; i++)
        {
            sprintf(&str[strlen(str)], "%02X", acc5[i]);
        }

        sprintf(&str[strlen(str)], " %02X;\r\n", sfdangle);

        while(port_tx_msg((uint8_t*)str, strlen(str)) !=_NO_ERR)
        {
            osThreadYield();    //force switch content
            osDelay(5);         //wait 5ms for Flush thread freed the buffer
        }
    }
}

/* @brief send acc & diagnostics information
 *           these are blocking operations
 *
 * */
void send_to_pc_diag_acc(rx_mail_t *pRxMailPckt)
{
    static int logNum = 0;

    char *str = CMD_MALLOC(MAX_STR_SIZE);
    uint8_t *p;

    if(str)
    {
        //send the Accumulator information from the pRxMailPckt
        if(app.pConfig->s.accEn == 1)
        {
            /* "master chip" */
            p = (uint8_t*)&pRxMailPckt->acc[0];

            sprintf(str, "\r\nAM%04X %02X CLKOFF: %d\r\n", logNum, pRxMailPckt->res.rangeNum,
                    (int)(pRxMailPckt->res.clockOffset_pphm));

            send_acc(str, MAX_STR_SIZE, p);

            /* "slave chip" */
            p = (uint8_t*)&pRxMailPckt->acc[1];
            sprintf(str, "\r\nAS%04X %02X\r\n", logNum, pRxMailPckt->res.rangeNum);

            send_acc(str, MAX_STR_SIZE, p);
        }

        //send the Diagnostics information from the pRxMailPckt
        if(app.pConfig->s.diagEn == 1)
        {
            /* "master chip" */
            p = (uint8_t*)&pRxMailPckt->diagnostics[0];
            sprintf(str, "DM%04X %02X ", logNum, pRxMailPckt->res.rangeNum);
            send_diag(str, MAX_STR_SIZE, p,
                      pRxMailPckt->pdoa_info.acc_master,
                      pRxMailPckt->pdoa_info.sfdangle_master);

            /* "slave chip" */
            p = (uint8_t*)&pRxMailPckt->diagnostics[1];
            sprintf(str, "DS%04X %02X ", logNum, pRxMailPckt->res.rangeNum);
            send_diag(str, MAX_STR_SIZE, p,
                    pRxMailPckt->pdoa_info.acc_slave,
                    pRxMailPckt->pdoa_info.sfdangle_slave);
        }

        CMD_FREE(str);
    }

    logNum++;
}


/* @brief
 *  Send Service message from Node:
 *  Currently sending stationary to the PC
 *
 *  'JSxxxx{"SN":
 *    {     "a16": %04X, //addr16 of the Node
 *            "V":%d //service message from the Node (stationary is bit 0)
 *          "X":%d //Normalized accel data X from the Node, mg
 *          "Y":%d //Normalized accel data Y from the Node, mg
 *          "Z":%d //Normalized accel data Z from the Node, mg
 *    }
 *   }'
 * */
void send_to_pc_stationary(stationary_res_t *p)
{
    int hlen;

    char *str = CMD_MALLOC(MAX_STR_SIZE);

    if(str)
    {
        if (app.pConfig->s.accEn == 1 || \
            app.pConfig->s.diagEn == 1 || \
            app.pConfig->s.reportLevel > 1)
        {

            /* shortest service "SN" output */
            sprintf(str, "SN%1X %04X%04X%04X\r\n",
                    (uint8_t)  (p->flag),
                    (uint16_t) (p->acc_x),
                    (uint16_t) (p->acc_y),
                    (uint16_t) (p->acc_z));
            port_tx_msg((uint8_t*)str, strlen(str));
        }
        else if (app.pConfig->s.reportLevel == 1)
        {
            /* use JSON type of output during a normal operation */
            hlen = sprintf(str,"JS%04X", 0x5A5A);    // reserve space for length of JS object

            sprintf(&str[strlen(str)],"{\"SN\": ");

            sprintf(&str[strlen(str)],
                    "{\"a16\":\"%04X\","
                    "\"V\":%d," //service message data from the Node: (stationary is bit 0)
                    "\"X\":%d," //Normalized accel data X from the Node, mg
                    "\"Y\":%d," //Normalized accel data Y from the Node, mg
                    "\"Z\":%d"  //Normalized accel data Z from the Node, mg
                    "}}",
                    (int)(p->addr),
                    (int)(p->flag),
                    (int)(p->acc_x),
                    (int)(p->acc_y),
                    (int)(p->acc_z));

            sprintf(&str[2],"%04X",strlen(str)-hlen);//add formatted 4X of length, this will kill first '{'
            str[hlen]='{';                          //restore the start bracket

            sprintf(&str[strlen(str)],"\r\n");
            port_tx_msg((uint8_t*)str, strlen(str));
        }
        else
        {
            //no output
        }
        CMD_FREE(str);
    }
}
