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
 * @brief handle module scenario.
 *
 * This file contains the module scenario.
 */


#ifndef __CFG_SCENARIO_H__
#define __CFG_SCENARIO_H__
#include "cfg_board_def.h"
#include "cfg_board.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    NONE,
    ACC,
    MAIN_SCENARIO_LOOP,
    TMP,   /* TMP102 */
    BLE,   /**< Bluetooth mode, the application acts a simulated Heart Rate sensor. */
    GPS, /**< Gazell mode, the application acts as a 'Gazell Device'. */
    WIFI,
    SIGFOX,
    IDLE,
    BATTERY_CHECK
}module_mode_t;

typedef enum
{
    GPS_START,    
    GPS_WORK,
    GPS_END
}gps_module_work_e;

typedef void (*main_nfg_tag_on_callback)(void);

#ifdef __cplusplus
}
#endif

void main_schedule_timeout_handler_ihere_mini(void * p_context);
void ihere_mini_current_schedule_start(void);
void ihere_mini_fast_schedule_start(void);
void ihere_mini_normal_schedule_mode_change(void);

#endif //__CFG_SCENARIO_H__
