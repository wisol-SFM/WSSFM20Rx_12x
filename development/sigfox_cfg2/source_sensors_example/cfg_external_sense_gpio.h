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

#define GFG_MAGNETIC_ISR_IGNORE_TIME_MS 1000

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*magnetic_attach_callback)(void);
typedef void (*wkup_detect_callback)(void);


/**
 * @brief Function for initializing the Magnetic Sensor. (attached low signal)
 * @param[in]   callback callback function for magnetic attacked
 */
void cfg_magnetic_sensor_init(magnetic_attach_callback callback);

/**
 * @brief Function for initializing the wake up gpio. (active low signal)
 * @param[in]   callback callback function for wkup key push (active high)
 */
void cfg_wkup_gpio_init(wkup_detect_callback callback);

#ifdef __cplusplus
}
#endif
#endif // __WBOARD_CONFIG_DEF__
