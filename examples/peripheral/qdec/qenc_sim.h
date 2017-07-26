/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef QENC_SIM_H__
#define QENC_SIM_H__

#include <stdbool.h>
#include <stdint.h>
#include "boards.h"
#include "nrf_qdec.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#define QENC_CONFIG_PIO_LED         ARDUINO_A3_PIN
#define QENC_CONFIG_PIO_A           ARDUINO_A5_PIN
#define QENC_CONFIG_PIO_B           ARDUINO_A4_PIN

#define QENC_CONFIG_PIO_PULL_CFG    NRF_GPIO_PIN_NOPULL

/**@brief quadrature encoder simulator initialization.
 *
 * The simulator uses GPIOTE module to generate pulses compatible with QDEC on A,B outputs using LED input as a clocking signal
 * The sole purpose of this module is to test software driver
 *
 * @param[in] active level for LED in QDEC
 *
 */
void qenc_init(nrf_qdec_ledpol_t led_pol);


/**@brief quadrature simulator simulator set-up for pulse generation.
 * This function starts normal pulse generation if pulse_count is nonzero
 * When pulse_count is greater then zero positive pulses are generated and pulse_count is decremented till it reaches zero
 * When pulse_count is less then zero negative pulses are generated and pulse_count is incremented till it reaches zero
 *
 * @param[in] number of normal pulses
 *
 */
void qenc_pulse_count_set(int32_t pulse_count);


/**@brief quadrature simulator simulator set-up set-up for pulse and double pulse generation.
 * This function starts leading normal pulse generation if pulse_count is nonzero
 * and trailing double generation if dble_pulse_count is nonzero
 * When pulse_count is greater then zero positive pulses are generated and pulse_count is decremented till it reaches zero
 * When pulse_count is less then zero negative pulses are generated and pulse_count is incremented till it reaches zero
 * Next if dble_pulse_count is nonzero double pulses are generated and dble_pulse_count is decremented till it reaches zero
 *
 * @param[in] number of normal pulses
 * @param[in] number of double pulses
 */
void qenc_pulse_dblpulse_count_set(int32_t pulse_count, uint32_t dble_pulse_count);

#ifdef __cplusplus
}
#endif

#endif // QENC_SIM_H__
