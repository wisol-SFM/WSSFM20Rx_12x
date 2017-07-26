/*
This software is subject to the license described in the License.txt file
included with this software distribution. You may not use this file except in compliance
with this license.
Copyright (c) Dynastream Innovations Inc. 2015
All rights reserved.
*/

#ifndef __MESSAGE_CACHE_H__
#define __MESSAGE_CACHE_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint8_t dst;
    uint8_t page;
    uint8_t data;
    uint8_t pay0;
    uint8_t pay1;
    uint8_t seq;
    uint8_t ticks;
} sf_message_t;

typedef struct
{
    uint8_t          front;
    uint8_t          back;
    uint8_t          count;
    uint8_t          size;
    sf_message_t * buffer;
} message_cache_t;


/**@brief Empty message cache.
 *
 * @param p_mc          Pointer to message cache struct
 */
void mc_clear(message_cache_t * p_mc);

/**@brief Add message to cache. Old messages are deleted to make room if full.
 *
 * @param p_mc          Pointer to message cache struct
 * @param p_message     Pointer to message to add
 */
void mc_add(message_cache_t * p_mc, uint8_t * p_message);

/**@brief Check if message is in cache.
 *
 * @param p_mc          Pointer to message cache struct
 * @param p_message     Pointer to message to check
 * @return True if message is already in cache
 */
bool mc_is_in_cache(message_cache_t * p_mc, uint8_t * p_message);

/**@brief Purge messages older than given value.
 *
 * @param p_mc          Pointer to message cache struct
 * @param max_ticks     Maximum number of ticks for messages to stay in cache
 */
void mc_cleanup(message_cache_t * p_mc, uint8_t max_ticks);


#ifdef __cplusplus
}
#endif

#endif
