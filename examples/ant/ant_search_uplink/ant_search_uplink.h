/*
This software is subject to the license described in the License.txt file
included with this software distribution. You may not use this file except in compliance
with this license.

Copyright (c) Dynastream Innovations Inc. 2015
All rights reserved.
*/

#ifndef ANT_SEARCH_UPLINK_H__
#define ANT_SEARCH_UPLINK_H__

#include <stdint.h>
#include "ant_stack_handler_types.h"
#include "bsp.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief Function for configuring search uplink
 *
 */
void ant_search_uplink_setup(void);


/**@brief Handles BSP events.
 *
 * @param[in] evt   BSP event.
 */
void ant_search_uplink_bsp_evt_handler(bsp_event_t evt);


/**@brief Handle ANT events
 * @param[in] p_ant_evt A pointer to the received ANT event to handle.
 */
void ant_search_uplink_event_handler(ant_evt_t * p_ant_evt);


#ifdef __cplusplus
}
#endif

#endif // ANT_SEARCH_UPLINK_H__
