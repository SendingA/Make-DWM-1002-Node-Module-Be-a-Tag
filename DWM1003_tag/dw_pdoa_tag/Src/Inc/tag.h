/*! ---------------------------------------------------------------------------
 * @file    tag.h
 * @brief   DecaWave
 *          bare implementation layer
 *
 * @attention
 *
 * Copyright 2016-2017 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author
 */

#ifndef __TAG__H__
#define __TAG__H__ 1

#ifdef __cplusplus
 extern "C" {
#endif


#include "uwb_frames.h"
#include "msg_time.h"

#include "port_platform.h"
#include "dw_pdoa_tag_common.h"

#define BLINK_PERIOD_MS            (500)    /* range init phase - blink sends period, ms */

#define DWT_DIAGNOSTIC_LOG_REV_5    (5)

/* Rx Events circular buffer.
 * 0x02, 0x04, 0x08, 0x10, etc.
 * The size of the buffer at any given time should be < 2 */
#define EVENT_BUF_SIZE                 (0x02)

#define DIAG_READ_SUPPORT (0)

#define POLL_ENTER_CRITICAL()     vPortEnterCritical()
#define POLL_EXIT_CRITICAL()      vPortExitCritical()
//-----------------------------------------------------------------------------
// Typedefs

/* Standard Diagnostics v5 */
typedef struct
{
    //NOTE: diagnostics data format rev 5 (DWT_DIAGNOSTIC_LOG_REV)
    uint8_t        header;    //00 this could be a header (format version number)
    uint8_t        r0F[ 5];//01 register 0xF - length 5 bytes
    uint8_t        r10[ 4];//06 register 0x10 - length 4 bytes
    uint8_t        r12[ 8];//10 register 0x12 - length 8 bytes
    uint8_t        r13[ 4];//18 register 0x13 - length 4 bytes
    uint8_t        r14[ 5];//22 register 0x14 - length 5 bytes
    uint8_t        r15[14];//27 register 0x15 - length 14 bytes (5 TS, 2 FP, 2 Diag, 5 TSraw)
    uint8_t        r25[16];//41 register 0x25 @FP (first path) -> 16 bytes starting at FP + 1 dummy
    uint8_t        r2E[ 2];//58 register 0x2E (0x1000) - 2 bytes
    uint8_t        r27[ 4];//60 register 0x27 (0x28)   - 4 bytes
    uint8_t        r2E2[2];//64 register 0x2E (0x1002) - 2 bytes
    uint8_t        dummy;
    //66 total
}__attribute__((packed))
diag_v5_t;


/* TxPckt */
typedef struct {
    int16_t        psduLen;// Length of msg to send

    union {
        std_msg_t             stdMsg;
        twr_msg_t             twrMsg;
        blink_msg_t           blinkMsg;
        poll_msg_t            pollMsg;
        final_msg_imuData_t   finalMsg;
    } msg;

    uint8_t        txFlag;  // Holds Tx sending parameters:
                            // DWT_START_TX_IMMEDIATE DWT_START_TX_DELAYED & DWT_RESPONSE_EXPECTED

    uint32_t    delayedTxTimeH_sy;  // DWT_START_TX_DELAYED  : Delayed transmit time (high32)
    uint32_t    delayedRxTime_sy;   // DWT_RESPONSE_EXPECTED : Delay after Tx when to switch on receiver
    uint16_t    delayedRxTimeout_sy;// DWT_RESPONSE_EXPECTED : For how long the receiver will be switched on after Tx

}tx_pckt_t;


/* RxPckt */
typedef struct {
    int16_t        rxDataLen;

    union {
        std_msg_t           stdMsg;
        std_msg_ss_t        ssMsg;
        std_msg_ls_t        lsMsg;
        twr_msg_t           twrMsg;
        blink_msg_t         blinkMsg;
        rng_cfg_msg_t       rngCfgMsg;
        rng_cfg_upd_msg_t   rngCfgUpdMsg;
        resp_ext_msg_t      respExtMsg;
    } msg;

    uint8_t     rxTimeStamp[TS_40B_SIZE];   /* Full TimeStamp */
    uint32_t    rxRtcTimeStamp;             /* MCU RTC timestamp */
    uint16_t    firstPath;                  /* First path (raw 10.6) */

#if (DIAG_READ_SUPPORT==1)
    diag_v5_t   diagnostics;                /* 66 bytes*/
#endif
}rx_pckt_t;


/* This structure holds application parameters:
 * eui64
 * txAntennaDelay
 * rxAntennaDelay
 * timestamps for every phase's IRQ:
 *             initiator: blinkTx_ts, pollTx_ts, respRX_ts, finalTx_ts, (reportRx_ts)
 *             responder: blinkRx_ts, pollRx_ts, respTx_ts, finalRx_ts, (reportTx_ts)
 *
 * */
typedef struct
{
    /* Unique long Address, used at the discovery phase before Range Init reception */
       union    {
           uint8_t  euiLong[8];
           uint64_t eui64;
       };

    /* circular Buffer of received Rx packets :
     * uses in transferring of the data from ISR to APP level.
     * */
    struct {
        rx_pckt_t   buf[EVENT_BUF_SIZE];
        uint16_t    head;
        uint16_t    tail;
    } rxPcktBuf;

    /* ranging variables */
    struct {
        /* MAC sequence number, increases on every tx_start */
        uint8_t        seqNum;

        /* Discovery phase : Tx time structures for DW_TX_IRQ callback */
        struct {
            uint8_t	 blinkTx_ts[TS_40B_SIZE];   /**< tag: blinkTx_ts, blinkRtcTimeStamp */
            uint32_t blinkRtcTimeStamp;         /**< handles the MCU RTC time at the DW_IRQ */

        };

        /* Ranging phase : Tx time structures for DW_TX_IRQ callback */
        struct {
            uint8_t  pollTx_ts[TS_40B_SIZE];    /**< tag:  pollTx_ts, pollRtcTimeStamp */
            uint32_t pollRtcTimeStamp;          /**< handles the MCU RTC time at the DW_IRQ */


            uint8_t  finalTx_ts[TS_40B_SIZE];   /**< tag:  finalTx_ts, finalRtcTimeStamp */
            uint32_t finalRtcTimeStamp;         /**< handles the MCU RTC time at the DW_IRQ */
        };

       /* Application DW_TX_IRQ source indicator */
        enum {
            Twr_Tx_Blink_Sent,          //tag sends blink
            Twr_Tx_Ranging_Config_Sent, //node sends range init
            Twr_Tx_Poll_Sent,           //tag sends poll
            Twr_Tx_Resp_Sent,           //node sends response
            Twr_Tx_Final_Sent,          //tag sends final
            Twr_Tx_Report_Sent          //not used
        }
        txState;
    };

    /* pre-calculated times for different messages */
    struct {
        msg_time_t    ranging_config;
        msg_time_t    poll;
        msg_time_t    response;
        msg_time_t    final;
    }msg_time;

    uint32_t        gRtcSFrameZeroCnt;        //Local SuperFrame start, Timestamp

    /* Environment - configured from Range init structure.
     * slotCorr_us is used to adjust slot every reception as part of Response
     */
    struct    env
    {
        uint8_t     version;

        mac_header_ss_t twr_mac_header;

        uint16_t    panID;
        uint8_t     tagAddr  [ADDR_BYTE_SIZE_S];
        uint8_t     nodeAddr[ADDR_BYTE_SIZE_S];

        uint32_t    sframePeriod_us;        //Superframe Period, us
        uint32_t    pollTx2FinalTxDelay32;  //This is delay used in TWR between Poll and Final rough sending: from Ranging Config message

        uint16_t    responseRxTo_sy; //pre-calculated Rx timeout for Response Msg
        uint16_t    delayRx_sy;      //Rx timeout from Ranging Config for Response Msg to the Node

        int32_t     slotCorr_us;     //Slot correction from current reception, us

        uint16_t    pollMultFast;    //multiplier for fast ranging in Superframe durations
        uint16_t    pollMultSlow;    //multiplier for slow ranging in Superframe durations

        union {
            uint16_t     mode;       //additional service: IMU on/off, etc.
            bool        imuOn : 1;   //tag shall use IMU to slow down its ranging
        };

    }env;

    struct acc
    {
        uint8_t     acc_x[2];
        uint8_t     acc_y[2];
        uint8_t     acc_z[2];
    }acc;

    bool        stationary_imu    : 1; //IMU report that the Tag is stationary
    bool        stationary        : 1; //IMU report that the Tag is stationary

    /* The number of range sequence, increases on every poll */
    uint16_t    rangeNum;

    /* Tag's crystal clock offset trimming */
    int16_t     clkOffset_pphm;     //
    uint8_t     xtaltrim;           //Tag crystal trim value

    volatile
    uint16_t    faultyRangesCnt;

    uint16_t    lateTX;             //used for Debug to count any lateTX

} twr_info_t;

typedef struct {
    dwt_config_t *pdwCfg;
    uint16_t     frameFilter;
    uint16_t     txAntDelay;
    uint16_t     rxAntDelay;
    uint16_t     panId;
    uint16_t     shortadd;
}rxtx_tag_configure_t;

//* enumeration of function codes used in TWR protocol */
typedef enum {
    Twr_Fcode_Not_Defined       = 0xFF, // Special : nothing
    Twr_Fcode_Blink             = 0xEE, // Special : Blink
    Twr_Fcode_Rng_Config        = 0x20, // Responder (Node) Ranging Config message          : reply to blink
    Twr_Fcode_Tag_Poll          = 0x84, // Initiator (Tag)  Poll message                    : twr start message
    Twr_Fcode_Resp_Ext          = 0x72, // Responder (Node) Response Extended               : reply to Poll with X & Y
    Twr_Fcode_Tag_Final         = 0x88, // Initiator (Tag)  Final message back to Responder : reply to Response
    Twr_Fcode_Tag_Accel_Final   = 0x89, // Initiator (Tag)  Final message back to Responder : reply to Response + Accelerometer data
}fcode_e;


enum {
    Head_Msg_BLINK      =   0xC5,
    Head_Msg_STD        =   (0x40 | 0x01),
    Head_Msg_STD1       =   (0x40 | 0x20 | 0x01),
    Frame_Ctrl_SS       =   (0x80 | 0x08),    //Message addressing SS
    Frame_Ctrl_LS       =   (0x80 | 0x0C),    //Dest addr long, Source address short
    Frame_Ctrl_MASK     =   0xCC
};


//-----------------------------------------------------------------------------
// exported functions prototypes

/* initiator (tag) */
twr_info_t * getTwrInfoPtr(void);

error_e tag_process_init(void);
void    tag_process_start(void);
void    tag_process_terminate(void);

error_e twr_initiator_algorithm_rx(rx_pckt_t *prxPckt, twr_info_t *ptwrInfo);

void    tag_wakeup_dw1000_blink_poll(twr_info_t *ptwrInfo);
void    initiator_wake_and_send_poll(twr_info_t *ptwrInfo);
error_e initiator_send_blink(twr_info_t *ptwrInfo);
error_e initiator_received_ranging_config(rx_pckt_t *prxPckt, twr_info_t *ptwrInfo);
error_e initiator_received_response(rx_pckt_t *prxPckt, twr_info_t *ptwrInfo);

void trim_tag_proc(twr_info_t *pTwrInfo);

#ifdef __cplusplus
}
#endif

#endif /* __TAG__H__ */
