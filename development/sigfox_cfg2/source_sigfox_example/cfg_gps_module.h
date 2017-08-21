/* Copyright (c) 2017 WISOL Corp. All Rights Reserved.
 *
 * The information contained herein is property of WISOL Cor.
 * Terms and conditions of usage are described in detail in WISOL STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @brief control gps module.
 *
 * This file contains the control gps module.
 */

#ifndef __CFG_GPS_MODULE_H__
#define __CFG_GPS_MODULE_H__
#include "cfg_board_def.h"
#ifdef __cplusplus
extern "C" {
#endif

#include "time.h"
#include "cfg_board.h"

#define GPS_NMEA_CHECK_CHECKSUM


#define CGPS_Result_OK              (0)
#define CGPS_Result_Busy            (-1)
#define CGPS_Result_Fix_Fail        (-2)
#define CGPS_Result_NotReady        (-3)
#define CGPS_Result_NoData          (-4)
#define CGPS_Result_NotStarted      (-5)
#define CGPS_Result_NotSupported    (-6)

#define CGPS_SPI_BUF_SIZE 192
#define CGPS_NMEA_BUF_SIZE 72

#define CGPS_NMEA_CNT 2
#define CGPS_NMEA_SIZE 12
#define GPS_SEND_PAYLOAD_SIZE 12
extern uint8_t m_cGpsRxNMEA_Buf[CGPS_SPI_BUF_SIZE];

typedef enum
{
    CGPS_State_none_S = 0,

    CGPS_State_idle_S = 10,
    CGPS_State_operation_fail_S,
    CGPS_State_notAvailable_S,
    CGPS_State_suspend_S,
    CGPS_State_resume_S,
    CGPS_State_sleep_S,
    CGPS_State_wakeup_S,
    CGPS_State_poweroff_S,    

    CGPS_State_ubx_setting_S = 100,
    CGPS_State_ubx_setting_end_S,
    CGPS_State_nmea_request_S,
    CGPS_State_nmea_tracking_S,
    CGPS_State_nmea_position_fix_S,
    CGPS_State_nmea_position_not_fix_S,
    CGPS_State_nmea_data_send_S,

    CGPS_State_bypass_start_S = 200,    //FEATURE_CFG_BYPASS_CONTROL
    CGPS_State_bypass_request_S,        //FEATURE_CFG_BYPASS_CONTROL
    CGPS_State_bypass_response_S,       //FEATURE_CFG_BYPASS_CONTROL
    CGPS_State_bypass_abort_S,          //FEATURE_CFG_BYPASS_CONTROL
    CGPS_State_bypass_suspend_S,        //FEATURE_CFG_BYPASS_CONTROL
    CGPS_State_bypass_resume_S,         //FEATURE_CFG_BYPASS_CONTROL

#ifdef CGPS_SPI_CMD_TEST  //for test
    CGPS_State_test_start_S = 300,
    CGPS_State_test_work_S = 301,  //spi data wait state
#endif

    CGPS_State_e_max
}CGPS_State_e;

typedef enum
{
    CGPS_SPI_STEP_ISR_CLEAR_WAIT,
    CGPS_SPI_STEP_WRITE_DATA_LEN,
    CGPS_SPI_STEP_WRITE_DATA,
    CGPS_SPI_STEP_READ_DATA_LEN,
    CGPS_SPI_STEP_READ_DATA,
    CGPS_SPI_STEP_max
}CGPS_SPI_STEP_e;

typedef enum
{
    CGPS_INIT_CMD_ECHO_OFF,
    CGPS_INIT_CMD_GPS_MODE_READ,
    CGPS_INIT_CMD_GPS_MODE_CHK,
    CGPS_INIT_CMD_GPS_MODE_SET,
    CGPS_INIT_CMD_CWLAPOPT_SET,
    CGPS_INIT_CMD_type_max
}CGPS_AT_CMD_type_e;

typedef enum
{
    CGPS_SCAN_RESP_PARSE_1ST_CHAR,
    CGPS_SCAN_RESP_PARSE_STR,
    CGPS_SCAN_RESP_PARAM_SKIP,
    CGPS_SCAN_RESP_PARAM_RSSID,
    CGPS_SCAN_RESP_MAX
}CGPS_SCAN_RESP_e;

typedef struct
{
    const char *cmdStr;
    unsigned int cmdStrLen;
}CGPS_AT_strInfo_t;


typedef enum
{
 CGPS_NMEA_NONE = 0,
 CGPS_NMEA_GGA,
 CGPS_NMEA_RMC,
 CGPS_NMEA_GSV,
 CGPS_NMEA_GSA,
 CGPS_NMEA_VTG,
 CGPS_NMEA_PQXFI,

 CGPS_NMEA_GPGGA = 10,   /* Fix data */
 CGPS_NMEA_GPRMC,        /* recommended minimum data */
 CGPS_NMEA_GPGSV,        /* GPS SVs in view */
 CGPS_NMEA_GPGSA,        /* GPS SV dop and active SV info */
 CGPS_NMEA_GPVTG,        /* Speed and heading info */

 CGPS_NMEA_GLGSV,        /* Glonass SV in view info */
 CGPS_NMEA_GNGSA,        /* Dop and Active SV info iff Glonass SVs are used */
 CGPS_NMEA_GNGNS,        /* new GGA message for GNSS */
 CGPS_NMEA_GARMC,        /* GAL recommended minimum data */
 CGPS_NMEA_GAGSV,        /* GAL SVs in view */
 CGPS_NMEA_GAGSA,        /* GAL SV dop and active SV info */
 CGPS_NMEA_GAVTG,        /* GAL Speed and heading info */
 CGPS_NMEA_PSTIS,        /* proprietary sentence at beginning of each sess */
 CGPS_NMEA_GSV_EXTENDED, /*Enable/Disable Extended GGSV*/
 CGPS_NMEA_GAGGA,        /* GAL Fix data */
 CGPS_NMEA_PQGSA,        /* QZSS Enable PQGSA */
 CGPS_NMEA_PQGSV,        /* QZSS Enable PQGSV */

 CGPS_NMEA_GNRMC,        /* Time, date, position, course and speed data */ 
 CGPS_NMEA_GNGSV,        /* Number of GPS satellites in view, satellite ID numbers, elevation, azimuth and SNR values */ 
 CGPS_NMEA_GNGGA         /* Time, position and fix type data for GPS constellations */ 
}CGPS_NMEA_Type_e;

typedef enum
{
    CGPS_CNO_CHECK_DISABLE,
    CGPS_CNO_CHECK_ENABLE,
    CGPS_CNO_CHECK_max
}CGPS_CN0_CHECK_e;

#ifdef NOT_USED
#define NMEA_TOKS_COMPARE   (1)
#define NMEA_TOKS_PERCENT   (2)
#define NMEA_TOKS_WIDTH     (3)
#define NMEA_TOKS_TYPE      (4)

#define NMEA_CONVSTR_BUF    (256)
#define NMEA_TIMEPARSE_BUF  (256)
#define NMEA_DEF_PARSEBUFF  (1024)

typedef void (*nmeaTraceFunc)(const char *str, int str_size);
typedef void (*nmeaErrorFunc)(const char *str, int str_size);

typedef struct _nmeaPROPERTY
{
    nmeaTraceFunc   trace_func;
    nmeaErrorFunc   error_func;
    int             parse_buff_size;

} nmeaPROPERTY;


#if defined(_MSC_VER)
# define NMEA_POSIX(x)  _##x
# define NMEA_INLINE    __inline
#else
# define NMEA_POSIX(x)  x
# define NMEA_INLINE    inline
#endif


#define NMEA_SIG_BAD        (0)
#define NMEA_SIG_LOW        (1)
#define NMEA_SIG_MID        (2)
#define NMEA_SIG_HIGH       (3)

#define NMEA_FIX_BAD        (1)
#define NMEA_FIX_2D         (2)
#define NMEA_FIX_3D         (3)

#define NMEA_MAXSAT         (12)
#define NMEA_SATINPACK      (4)
#define NMEA_NSATPACKS      (NMEA_MAXSAT / NMEA_SATINPACK)

#define NMEA_DEF_LAT        (5001.2621)
#define NMEA_DEF_LON        (3613.0595)


/*
 * high level
 */

typedef struct _nmeaPARSER
{
    void *top_node;
    void *end_node;
    unsigned char *buffer;
    int buff_size;
    int buff_use;

} nmeaPARSER;


/**
 * Position data in fractional degrees or radians
 */
typedef struct _nmeaPOS
{
    double lat;         /**< Latitude */
    double lon;         /**< Longitude */

} nmeaPOS;

/**
 * Information about satellite
 * @see nmeaSATINFO
 * @see nmeaGPGSV
 */
typedef struct _nmeaSATELLITE
{
    int     id;         /**< Satellite PRN number */
    int     in_use;     /**< Used in position fix */
    int     elv;        /**< Elevation in degrees, 90 maximum */
    int     azimuth;    /**< Azimuth, degrees from true north, 000 to 359 */
    int     sig;        /**< Signal, 00-99 dB */

} nmeaSATELLITE;


/**
 * Information about all satellites in view
 * @see nmeaINFO
 * @see nmeaGPGSV
 */
typedef struct _nmeaSATINFO
{
    int     inuse;      /**< Number of satellites in use (not those in view) */
    int     inview;     /**< Total number of satellites in view */
    nmeaSATELLITE sat[NMEA_MAXSAT]; /**< Satellites information */

} nmeaSATINFO;



typedef struct _nmeaParserNODE
{
    int packType;
    void *pack;
    struct _nmeaParserNODE *next_node;

} nmeaParserNODE;

/**
 * Date and time data
 * @see nmea_time_now
 */
typedef struct _nmeaTIME
{
    int     year;       /**< Years since 1900 */
    int     mon;        /**< Months since January - [0,11] */
    int     day;        /**< Day of the month - [1,31] */
    int     hour;       /**< Hours since midnight - [0,23] */
    int     min;        /**< Minutes after the hour - [0,59] */
    int     sec;        /**< Seconds after the minute - [0,59] */
    int     hsec;       /**< Hundredth part of second - [0,99] */

} nmeaTIME;




/**
 * Summary GPS information from all parsed packets,
 * used also for generating NMEA stream
 * @see nmea_parse
 * @see nmea_GPGGA2info,  nmea_...2info
 */
typedef struct _nmeaINFO
{
    int     smask;      /**< Mask specifying types of packages from which data have been obtained */

    nmeaTIME utc;       /**< UTC of position */

    int     sig;        /**< GPS quality indicator (0 = Invalid; 1 = Fix; 2 = Differential, 3 = Sensitive) */
    int     fix;        /**< Operating mode, used for navigation (1 = Fix not available; 2 = 2D; 3 = 3D) */

    double  PDOP;       /**< Position Dilution Of Precision */
    double  HDOP;       /**< Horizontal Dilution Of Precision */
    double  VDOP;       /**< Vertical Dilution Of Precision */

    double  lat;        /**< Latitude in NDEG - +/-[degree][min].[sec/60] */
    double  lon;        /**< Longitude in NDEG - +/-[degree][min].[sec/60] */
    double  elv;        /**< Antenna altitude above/below mean sea level (geoid) in meters */
    double  speed;      /**< Speed over the ground in kilometers/hour */
    double  direction;  /**< Track angle in degrees True */
    double  declination; /**< Magnetic variation degrees (Easterly var. subtracts from true course) */

    nmeaSATINFO satinfo; /**< Satellites information */

} nmeaINFO;



/**
 * NMEA packets type which parsed and generated by library
 */
enum nmeaPACKTYPE
{
    GPNON   = 0x0000,   /**< Unknown packet type. */
    GPGGA   = 0x0001,   /**< GGA - Essential fix data which provide 3D location and accuracy data. */
    GPGSA   = 0x0002,   /**< GSA - GPS receiver operating mode, SVs used for navigation, and DOP values. */
    GPGSV   = 0x0004,   /**< GSV - Number of SVs in view, PRN numbers, elevation, azimuth & SNR values. */
    GPRMC   = 0x0008,   /**< RMC - Recommended Minimum Specific GPS/TRANSIT Data. */
    GPVTG   = 0x0010    /**< VTG - Actual track made good and speed over ground. */
};


/**
 * RMC packet information structure (Recommended Minimum sentence C)
 */
typedef struct _nmeaGPRMC
{
    nmeaTIME utc;       /**< UTC of position */
    char    status;     /**< Status (A = active or V = void) */
    double  lat;        /**< Latitude in NDEG - [degree][min].[sec/60] */
    char    ns;         /**< [N]orth or [S]outh */
    double  lon;        /**< Longitude in NDEG - [degree][min].[sec/60] */
    char    ew;         /**< [E]ast or [W]est */
    double  speed;      /**< Speed over the ground in knots */
    double  direction;  /**< Track angle in degrees True */
    double  declination; /**< Magnetic variation degrees (Easterly var. subtracts from true course) */
    char    declin_ew;  /**< [E]ast or [W]est */
    char    mode;       /**< Mode indicator of fix type (A = autonomous, D = differential, E = estimated, N = not valid, S = simulator) */

} nmeaGPRMC;
#endif /* NOT_USED */

/**
 * @brief       Function for initializing resource of gps module 
 */
void cGps_resource_init(void);

/**
 * @brief       Function for prepare gps module
 */
void cGps_prepare_start(void);

/**
 * @brief       Function for acquire tracking available check
 */
int cGps_acquire_tracking_check(void);

/**
 * @brief       Function for initializing gpio of gps
 */
void cGps_gpio_init(void);

/**
 * @brief       Function for entering gps power control
 * power control sequence
 * Power on
 * 1. PIN_DEF_2ND_POW_EN ON
 * 2. PIN_DEF_GPS_PWR_EN ON
 * Power off
 * 1. PIN_DEF_GPS_PWR_EN OFF
 * 2. PIN_DEF_2ND_POW_EN OFF
 */
void cGps_power_control(bool on, bool first);

/**
 * @brief       Function for data bus enable/disable control
 */
void cGps_bus_enable(bool enable);

/**
 * @brief       Function for status check (bus use check)
 */
bool cGps_bus_busy_check(void);

/**
 * @brief       Function for getting gps data result.
 *
 * @param[in]   pointer of getting gps data
 *
 * @return      CGPS_Result_OK on success. Otherwise an error code(eg.CGPS_Result_NoData).
 */
int cGps_nmea_get_bufPtr(uint8_t **nmea_gprmc); 

/**
 * @brief       Function for status check (GPS available check)
 */
int cGps_status_available(void);

/**
 * @brief       Function for gps nmea data acquire request
 */
void cGps_nmea_acquire_request(void);

/**
 * @brief       Function for gps position fixed check(complete of get gps data)
 */
int cGps_nmea_position_fix_check(void);

/**
 * @brief       Function for gps state
 */
int cGps_state_get(void);

/**
 * @brief       Function for gps just power control(Not control data interface)
 */
void cGps_only_power_control(bool on);

/**
 * @brief       Function for bypass state get mode change
 */
int cGps_bypass_get_mode_change(void);
/**
 * @brief       Function for bypass abort request
 */
void cGps_bypass_abort_request(void);

/**
 * @brief       Function for bypass available check
 */
int cGps_bypass_available_check(void);

/**
 * @brief       Function for bypass mode (for i2c slavle control).
 *
 * @return      CGPS_Result_OK on success.
 */
int cGps_bypass_request(void);

/**
 * @brief       Function for bypass status check
 */
int cGps_bypass_mode(void);

/**
 * @brief       Function for bypass status check
 */
bool cGps_is_bypass_mode(void);

/**
 * @brief       Function for bypass response data
 */
void cGpsState_bypass_response(const char *pData, unsigned int dataSize);

/**
 * @brief       Function for entering gps download mode power control
 */
void cGps_download_power_control(void);

/**
 * @brief       Function for bypass cmd request
 */
bool cGps_bypass_cmd_request(const char *pData, unsigned int dataSize);

/**
 * @brief       Function for initializing of gps module 
 * Initial value setting
 * GPIO configuration
 * Power control
 */
void gps_init(void);

/**
 * @brief       gps driver Function for gps tracking start
 * @return      CGPS_Result_OK on success 
 */
int start_gps_tracking(void);

/**
 * @brief       gps driver Function for gps nmea data check
 * @return      CGPS_Result_OK on success 
 */
int start_gps_nmea_data_check(void);

/**@brief Function for handling set C/NO check.
 *
 * @details This function will set enable/disable of gps C/N0 check(save current consumption)
 * module_parameter_item_e : Parameters for distinguishing the modes of GPS CN0 check
 * val : Enable(1) / disable(0) setting value
 *          
 */
void set_cn0_current_savetime_enable(module_parameter_item_e item, int val);

/**
* @brief Function for handling get C/NO check.
 */
unsigned int get_cn0_current_savetime_enable(module_parameter_item_e item);

/**@brief Function for handling the data from FS.
 * @details This function will set the data received from FS
 *          
 * @param[in] item  set module type configuration item
 * @param[in] val   interval time configuration
 */
void gps_tracking_set_interval(module_parameter_item_e item, unsigned int val);

/**@brief Function for handling the data from FS.
 * @details This function will get the data received from FS
 *
 * @param[in] item  get module type configuration item
 *
 * @return     The value returned is the set interval time.
 */
unsigned int gps_tracking_get_interval(module_parameter_item_e item);

/**
 * @brief        gps driver Function for result of NMEA data
 * @param[out]   latitude pointer of getting NMEA data
 * @param[out]   longitude pointer of getting NMEA data
 * @return       CGPS_Result_OK on success 
 */
int get_NMEA_Location(char **latitude, char **longitude);

/**
 * @brief        gps driver Function for result of NMEA data
 * @param[out]   hour pointer of getting NMEA data
 * @param[out]   minute pointer of getting NMEA data
 * @param[out]   second pointer of getting NMEA data
 * @return       CGPS_Result_OK on success 
 */
int get_NMEA_UTCTime(uint8_t *hour, uint8_t *minute, uint8_t *second);

/**
 * @brief        gps driver Function for result of NMEA data
 * @param[out]   hdop pointer of getting NMEA data
 * @return       CGPS_Result_OK on success 
 */
int get_NMEA_HDOP(char **hdop);

/**
 * @brief        gps driver Function for result of NMEA data
 * @param[out]   speed(knote) pointer of getting NMEA data
 * @return       CGPS_Result_OK on success 
 */
int get_NMEA_Speed_knot(char **speed);

/**
 * @brief        gps driver Function for result of NMEA data
 * @param[out]   ns pointer of getting NMEA data
 * @param[out]   ew pointer of getting NMEA data
 * @return       CGPS_Result_OK on success 
 */
int get_NMEA_Direction(char **ns, char **ew);

/**
 * @brief        gps driver Function for result of NMEA data
 * @param[out]   year pointer of getting NMEA data
 * @param[out]   month pointer of getting NMEA data
 * @param[out]   day pointer of getting NMEA data
 * @return       CGPS_Result_OK on success 
 */
int get_NMEA_UTCDate(uint8_t *year, uint8_t *month, uint8_t *day);

/**
 * @brief        Check status of gps tracking end.
 * @return       Return true if gps tracking end (position fixed success or fixed fail) 
 */
bool cGPS_waiting_tracking_end_check(void);

#ifdef __cplusplus
}
#endif
#endif // __WBOARD_CONFIG_DEF__
