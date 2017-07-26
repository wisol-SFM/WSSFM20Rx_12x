#ifndef __CFG_DBG_LOG_H__
#define __CFG_DBG_LOG_H__
#include <stdarg.h>
#include "SEGGER_RTT.h"
#include "cfg_board_def.h"
#include "cfg_twis_board_control.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CDBG_NUM2MASK(num) (0x00000001 << num)

#define CDBG_MAIN_LOG       0
#define CDBG_COMMON_LOG     1
//flow control
#define CDBG_FCTRL_INFO     2
#define CDBG_FCTRL_DBG      3
#define CDBG_FCTRL_ERR      4
//flash memory control
#define CDBG_FLASH_INFO     5
#define CDBG_FLASH_DBG      6
#define CDBG_FLASH_ERR      7
//BLE control
#define CDBG_BLE_INFO       8
#define CDBG_BLE_DBG        9
#define CDBG_BLE_ERR        10
//WIFI control
#define CDBG_WIFI_INFO      11
#define CDBG_WIFI_DBG       12
#define CDBG_WIFI_ERR       13
//GPS control
#define CDBG_GPS_INFO       14
#define CDBG_GPS_DBG        15
#define CDBG_GPS_ERR        16
//SIGFOX control
#define CDBG_SIGFOX_INFO    17
#define CDBG_SIGFOX_DBG     18
#define CDBG_SIGFOX_ERR     19
//G SENSOR control
#define CDBG_GSEN_INFO      20
#define CDBG_GSEN_DBG       21
#define CDBG_GSEN_ERR       22
//TWO WIRE (I2C) board control
#define CDBG_TBC_INFO       23
#define CDBG_TBC_DBG        24
#define CDBG_TBC_ERR        25

#define CDBG_EXT_SEN_INFO   26
#define CDBG_EXT_SEN_DBG    27
#define CDBG_EXT_SEN_ERR    28

#ifdef FEATURE_CFG_DEBUG_PRINT_OUT

#define CDBG_mask_val_get() CDBGOutMask
#define CDBG_mask_val_set(val) CDBGOutMask=val
#define CDBG_mask_set(mask) CDBGOutMask=(CDBGOutMask|mask)
#define CDBG_mask_clear(mask) CDBGOutMask=(CDBGOutMask&(~mask))

#ifdef CDBG_LOG_INSTANCE
#include <stdarg.h>
unsigned int CDBGOutMask= 0 \
    | CDBG_NUM2MASK(CDBG_MAIN_LOG) \
    | CDBG_NUM2MASK(CDBG_COMMON_LOG) \
    | CDBG_NUM2MASK(CDBG_FCTRL_INFO) \
    | CDBG_NUM2MASK(CDBG_FCTRL_ERR) \
    | CDBG_NUM2MASK(CDBG_FLASH_INFO) \
    | CDBG_NUM2MASK(CDBG_FLASH_ERR) \
    | CDBG_NUM2MASK(CDBG_BLE_INFO) \
    | CDBG_NUM2MASK(CDBG_BLE_ERR) \
    | CDBG_NUM2MASK(CDBG_WIFI_INFO) \
    | CDBG_NUM2MASK(CDBG_WIFI_ERR) \
    | CDBG_NUM2MASK(CDBG_GPS_INFO) \
    | CDBG_NUM2MASK(CDBG_GPS_ERR) \
    | CDBG_NUM2MASK(CDBG_SIGFOX_INFO) \
    | CDBG_NUM2MASK(CDBG_SIGFOX_ERR) \
    | CDBG_NUM2MASK(CDBG_GSEN_INFO) \
    | CDBG_NUM2MASK(CDBG_GSEN_ERR) \
    | CDBG_NUM2MASK(CDBG_TBC_INFO) \
    | CDBG_NUM2MASK(CDBG_TBC_ERR) \
    | CDBG_NUM2MASK(CDBG_EXT_SEN_INFO) \
    | CDBG_NUM2MASK(CDBG_EXT_SEN_ERR) \
;

#if defined(NRF_LOG_USES_RTT) && (NRF_LOG_USES_RTT == 1)
extern int SEGGER_RTT_vprintf(unsigned BufferIndex, const char * sFormat, va_list * pParamList);
#endif

/**
 * @brief internal function for cPrintLog().
 */
void __cLogOut(unsigned int dbgID, const char * x_fmt, ...)
{
#if defined(NRF_LOG_USES_RTT) && (NRF_LOG_USES_RTT == 1) || defined(FEATURE_CFG_DEBUG_OUT_TO_TBC)
    va_list ParamList;
#endif
    if(CDBGOutMask & CDBG_NUM2MASK(dbgID))
    {
#if defined(NRF_LOG_USES_RTT) && (NRF_LOG_USES_RTT == 1)
        va_start(ParamList, x_fmt);
        SEGGER_RTT_vprintf(0, x_fmt, &ParamList);
#endif

#if defined(FEATURE_CFG_DEBUG_OUT_TO_TBC)
        va_start(ParamList, x_fmt);
        cTBC_vprintf(x_fmt, &ParamList);
#endif
    }
}

/**
 * @brief internal function for cDataDumpPrintOut().
 */
void __cDataDumpPrintOut(unsigned int dbgID, const unsigned char *pData, unsigned int size)
{
    int i;
    const unsigned char *p=pData;
    char buf[4];
    char dH, dL;

    if(CDBGOutMask & CDBG_NUM2MASK(dbgID))
    {
        for(i=0;i<size; i++)
        {
            dH = (p[i] >> 4);
            dL = (p[i] & 0x0f);
            if(dH < 10)
            {
                buf[0]='0'+dH;
            }
            else
            {
                buf[0]='A'+(dH-10);
            }
            if(dL < 10)
            {
                buf[1]='0'+dL;
            }
            else
            {
                buf[1]='A'+(dL-10);
            }
#if defined(NRF_LOG_USES_RTT) && (NRF_LOG_USES_RTT == 1)
            SEGGER_RTT_Write(0, buf, 2);
#endif
#if defined(FEATURE_CFG_DEBUG_OUT_TO_TBC)
            cTBC_write_tx_data(buf, 2);
#endif

        }
#if defined(NRF_LOG_USES_RTT) && (NRF_LOG_USES_RTT == 1)
        SEGGER_RTT_Write(0, "\n", 1);
#endif
#if defined(FEATURE_CFG_DEBUG_OUT_TO_TBC)
        cTBC_write_tx_data("\n", 1);
#endif
    }
}
#else
extern unsigned int CDBGOutMask; 
extern void __cLogOut(unsigned int dbgID, const char * x_fmt, ...);
extern void __cDataDumpPrintOut(unsigned int dbgID, const unsigned char *pData, unsigned int size);
#endif

/**
 * @brief internal macro for cPrintLog()
 */
#define __cPrintLog(dbgID, msg, args...) __cLogOut(dbgID, msg, ## args)\

/**
 * @brief print out debug msg (output interface:i2c slave(depend on FEATURE_CFG_DEBUG_OUT_TO_TBC), rtt(depend on NRF_LOG_USES_RTT))
 *
 * @param[in]  dbgID           Value of the msg ID (defined in cfg_dbg_log.h eg.CDBG_MAIN_LOG)
 * @param[in]  x_fmt           Pointer to format string
 * @param[in]  args...         arguments for the format string
 */
#define cPrintLog(dbgID, x_fmt, args...) __cPrintLog(dbgID, "%02d "x_fmt, dbgID, ## args)

/**
 * @brief print out dump data (output interface:i2c slave(depend on FEATURE_CFG_DEBUG_OUT_TO_TBC), rtt(depend on NRF_LOG_USES_RTT))
 *
 * @param[in]  dbgID           Value of the msg ID (defined in cfg_dbg_log.h eg.CDBG_MAIN_LOG)
 * @param[in]  data            Pointer of data
 * @param[in]  size            Size of data
 */
#define cDataDumpPrintOut(dbgID, data, size) __cDataDumpPrintOut(dbgID, (const unsigned char *)data, (unsigned int)size)

/**
 * @brief print out log filter value
 */
#define cPrintLogMask() __cLogOut(CDBG_MAIN_LOG, "CDBGOutMask:0x%08x\n", CDBGOutMask)
#else
#define cPrintLog(dbgID, x_fmt, args...)
#define cDataDumpPrintOut(dbgID, data, size)
#define cPrintLogMask()
#define CDBG_mask_val_get() 0
#define CDBG_mask_val_set(val)
#define CDBG_mask_set(mask)
#define CDBG_mask_clear(mask)
#endif

#ifdef __cplusplus
}
#endif
#endif // BLE_APP_GZLL_UI_H__
/** @} */

