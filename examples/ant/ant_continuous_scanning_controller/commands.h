/*
This software is subject to the license described in the License.txt file
included with this software distribution. You may not use this file except in compliance
with this license.

Copyright (c) Dynastream Innovations Inc. 2015
All rights reserved.
*/

#ifndef __COMMANDS_H__
#define __COMMANDS_H__

#ifdef __cplusplus
extern "C" {
#endif

#define COMMAND_LIGHT_OFF               ((uint8_t) 0)       /**< Command: light off. */
#define COMMAND_LIGHT_ON                ((uint8_t) 1)       /**< Command: light on. */

#define COMMAND_STATE_OFF               ((uint8_t) 1)       /**< Command: off */
#define COMMAND_STATE_ON                ((uint8_t) 2)       /**< Command: on. */
#define COMMAND_STATE_ALL_OFF           ((uint8_t) 3)       /**< Command: all off. */
#define COMMAND_STATE_ALL_ON            ((uint8_t) 4)       /**< Command: all on. */

#define MOBILE_COMMAND_PAGE             ((uint8_t) 0x10)    /**< Mobile command. */
#define DEVICE_STATUS_PAGE              ((uint8_t) 0x20)    /**< Device status. */

#define ADDRESS_ALL_NODES               ((uint8_t) 0)       /**< Address all nodes. */


#ifdef __cplusplus
}
#endif

#endif
