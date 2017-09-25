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
 * @brief driver of external gpio sense.
 *
 * This file contains external gpio.
 */


#ifndef __CFG_EXTERNAL_SENSE_GPIO_H__
#define __CFG_EXTERNAL_SENSE_GPIO_H__
#include "cfg_board_def.h"
#include "nrf_drv_spi.h"

#define GFG_MAGNETIC_DETECT_IGNORE_TIME_MS 500
#define GFG_MAGNETIC_DETECT_TICK_TIME_MS 100
#define GFG_MAGNETIC_DETECT_IGNORE_TICK_CNT (GFG_MAGNETIC_DETECT_IGNORE_TIME_MS/GFG_MAGNETIC_DETECT_TICK_TIME_MS)

#define GFG_BUTTON_SENSE_TICKE_MS 100
#define GFG_BUTTON_SENSE_PRESS_TICK_SHORT 2 // ignore time
#define GFG_BUTTON_SENSE_PRESS_TICK_LONG 30 // 3sec

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*magnetic_attach_callback)(void);
typedef void (*magnetic_detach_callback)(void);
typedef void (*wkup_detect_callback)(void);
typedef void (*usr_def_button_short_press_callback)(void);
typedef void (*usr_def_button_long_press_callback)(void);

/**
 * @brief Function for initializing the Magnetic Sensor. (attached low signal)
 * @param[in]   callback callback function for magnetic attacked
 * @param[in]   callback callback function for magnetic detached
 */
void cfg_magnetic_sensor_init(magnetic_attach_callback callback_attach, magnetic_detach_callback callback_detach);


/**
 * @brief Function for initializing the wake up gpio. (active low signal)
 * @param[in]   callback callback function for wkup key push (active high)
 */
void cfg_wkup_gpio_init(wkup_detect_callback callback);

void cfg_button_init(bool is_active_high, uint32_t pin, usr_def_button_short_press_callback short_press_CB, usr_def_button_long_press_callback long_press_CB);
void cfg_button_release(void);


#ifdef __cplusplus
}
#endif
#endif // __WBOARD_CONFIG_DEF__
