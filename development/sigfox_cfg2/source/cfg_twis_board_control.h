#ifndef __CFG_TWIS_BOARD_CONTROL_H__
#define __CFG_TWIS_BOARD_CONTROL_H__
#include <stdarg.h>
#include "cfg_board_def.h"

#define CTBC_PRINTF_BUFFER_SIZE (128)  //ref SEGGER_RTT_PRINTF_BUFFER_SIZE

#define CTBC_DEVICE_ID      0x4A  //board cotrol TWI address

#define CTBC_TX_STEP_SIZE 64
#define CTBC_RX_BUF_SIZE 128
#define CTBC_TX_BUF_SIZE 2048

#define CTBC_PRINTF_BUFFER_SIZE (128)


#define CTBC_PROC_REQ_TIME_MS 10

#define CTBC_WAIT_READ_DEVICE_ID_TIME_MS 100
#define CTBC_WAIT_READ_DEVICE_ID_TIME_OUT_TICK (10 * 10)  // 10 sec

#define CTBC_WAIT_CMD_PROC_MS 100


#define CTBC_REQ_MARKER "<SC>"
#define CTBC_RESP_MARKER "<SR>"

typedef enum
{
    CTBC_CMD_COMMON             =   0x00,  
    CTBC_CMD_LOG_FILTER         =   0x01,
    CTBC_CMD_MODE_CHANGE        =   0x02,
    CTBC_CMD_BYPASS_DATA        =   0x03,
    CTBC_CMD_PARAM_SET          =   0x04,
    CTBC_CMD_PARAM_GET          =   0x05,
    CTBC_CMD_GET_LOG_FILTER     =   0x06,
    CTBC_CMD_GET_ID_INFO        =   0x07,
    CTBC_CMD_SW_RESET           =   0x08,
    CTBC_CMD_FACTORY_RESET      =   0x09,
    CTBC_CMD_GET_SW_VERSION     =   0x0A,
    CTBC_CMD_TYPE_MAX
}CTBC_CMD_TYPE;

typedef enum
{
    cfg_board_work_normal               = 0x00,
    cfg_board_work_sigfox_bypass        = 0x01,
    cfg_board_work_wifi_bypass          = 0x02,
    cfg_board_work_manual               = 0x03,
    cfg_board_work_gps_bypass           = 0x04,
    cfg_board_work_ble_crtl             = 0x05,
    cfg_board_work_acc_bypass           = 0x06,
    cfg_board_work_mode_max,
    cfg_board_work_mode_get_cur = 0xff,
}cfg_board_work_mode_e;

#define CTBC_RESP_COMMON_ERROR "<SR>0400NG"
#define CTBC_RESP_READY "<SR>0700READY"

#define CTBC_DATA_LEN_FIELD_SIZE 2
#define CTBC_DATA_CMD_FIELD_SIZE 2
#define CTBC_DATA_RESP_RESULT_FIELD_SIZE 2

#define CTBC_REQ_DATA_SIZE_MIN ((sizeof(CTBC_REQ_MARKER)-1)+CTBC_DATA_LEN_FIELD_SIZE+CTBC_DATA_CMD_FIELD_SIZE)

#define CTBC_DATA_LEN_FIELD_OFFSET ((sizeof(CTBC_REQ_MARKER)-1))
#define CTBC_DATA_CMD_FIELD_OFFSET ((sizeof(CTBC_REQ_MARKER)-1)+CTBC_DATA_LEN_FIELD_SIZE)
#define CTBC_DATA_CMD_PARAMETER_OFFSET ((sizeof(CTBC_REQ_MARKER)-1)+CTBC_DATA_LEN_FIELD_SIZE+CTBC_DATA_CMD_FIELD_SIZE)

#define CTBC_DATA_REQ_DATA_FIELD_OFFSET ((sizeof(CTBC_REQ_MARKER)-1)+CTBC_DATA_LEN_FIELD_SIZE+CTBC_DATA_CMD_FIELD_SIZE)
#define CTBC_DATA_RESP_RESULT_FIELD_OFFSET ((sizeof(CTBC_REQ_MARKER)-1)+CTBC_DATA_LEN_FIELD_SIZE+CTBC_DATA_CMD_FIELD_SIZE)
#define CTBC_DATA_RESP_DATA_OFFSET ((sizeof(CTBC_REQ_MARKER)-1)+CTBC_DATA_LEN_FIELD_SIZE+CTBC_DATA_CMD_FIELD_SIZE+CTBC_DATA_RESP_RESULT_FIELD_SIZE)

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CTBC_INSTANCE
char m_cTBC_resp_buf[128];
#else
extern char m_cTBC_resp_buf[128];
#endif

void cTBC_init(void);
bool cTBC_is_busy(void);
#ifdef FEATURE_CFG_DEBUG_OUT_TO_TBC
int cTBC_vprintf(const char * sFormat, va_list * pParamList);
#endif
bool cTBC_check_host_connected(void);
unsigned int cTBC_write_tx_data(const char *p_buffer, int num_bytes);
void cTBC_write_state_noti(const char *noti_str);
void cTBC_boot_msg_noti(const char *noti_str);
void cTBC_mode_change_msg_noti(bool is_ok, const char *noti_str);
void cTBC_put_bypass_data(const uint8_t *data, uint32_t data_size);

#ifdef __cplusplus
}
#endif
#endif  //__CFG_TWIS_BOARD_CONTROL_H__
