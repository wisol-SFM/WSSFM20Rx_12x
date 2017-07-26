/*
This software is subject to the license described in the License.txt file
included with this software distribution. You may not use this file except in compliance
with this license.

Copyright (c) Dynastream Innovations Inc. 2014
All rights reserved.
*/

/**@file
 * @brief Device registry used in the scan and forward demo
 * This module is responsible for management of the device registry used to store information about peripheral devices that have registered wih the hub
 *
 * @defgroup ant_scan_and_forward_example ANT Scan and Forward Demo
 * @{
 * @ingroup nrf_ant_scan_and_forward
 *
 */

#ifndef __DEVICEREGISTRY_H__
#define __DEVICEREGISTRY_H__

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Public Definitions
#define MAX_DEVICES                 ((uint8_t) 16)              /**< Maximum number of devices in that can be registered */
#define NODE_ID_INVALID             ((uint8_t) 0xFF)            /**< Invalid node identifier */
#define ANT_CHANNEL_NOT_CONNECTED   ((uint8_t) 0xFF)            /**< Not directly connected to a particular device */
#define ANT_CHANNEL_SELF            ((uint8_t) 0xFE)            /**< Local node */


/**@brief Data structure for each device.
 */
typedef struct
{
    uint8_t node_id;                        /**< Node ID of the device */
    uint8_t ant_channel;                    /**< ANT channel the device is directly connected to. 0xFF if not connected. */
    uint8_t application_state;              /**< Device application state (i.e., light on/off) */
    uint8_t last_message_sequence_received; /**< Sequence number in last message received from device */
} device_t;

/**@brief Data structure for device list
 */
typedef struct
{
    uint8_t  count;                /**< Total number of registered devices */
    device_t devices[MAX_DEVICES]; /**< List of registered devices */
} deviceregistry_t;


/** @brief Initializes the device registry
 */
void dr_init(void);

/** @brief Add a device to the registry
 *
 * @param[in] node_id   The node id to add
 * @returns True if the device was added, false if it cannot be added
 */
bool dr_device_add(uint8_t node_id);

/** @brief Remove a device to the registry
 *
 * @param[in] node_id   The node id to remove
 * @returns True if the device was removed, false if it cannot be removed
 */
bool dr_device_remove(uint8_t node_id);

/** @brief Checks if device registry is full
 *
 * @returns True if the number of registered devices is the maximum allowed
 */
bool dr_is_full(void);

/** @brief Gets device with specified node id
 *
 * @param[in] node_id The id of the node to get

 * @returns Pointer the the registered device with a specified node ID.
 * @returns NULL if it doesnt exist
 */
device_t * dr_device_get(uint8_t node_id);

/** @brief Gets device at the specifed index
 *
 * @param[in] index The id of the node to get
 * @returns Pointer the the device
 */
device_t * dr_device_at_index_get(uint8_t index);

/** @brief Checks if a device already exists in the device table
 *
 * @param[in] node_id The id of the node to check
 * @returns True if device exists
 */
bool dr_device_exists(uint8_t node_id);

/** @brief Checks if a device already exists in the device table at the specifed index
 *
 * @param[in] index The index at which to check
 * @returns True if device exists, false otherwise
 */
bool dr_device_at_index_exists(uint8_t index);

/** @brief Gets the device index with specified node id
 *
 * @param[in] node_id The id of the node to get

 * @returns The index of the device with node_id.
 * @returns 0 if it doesnt exist
 */
uint8_t dr_index_of_node_get(uint8_t node_id);



#ifdef __cplusplus
}
#endif

#endif

/**
 *@}
 **/
