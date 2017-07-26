/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
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

/**
 * @file
 * @brief An example of use of Adafruit tag reader combined with Type 2 Tag parser.
 *
 * @sa nfc_adafruit_tag_reader_example
 */

/**
 * @defgroup nfc_adafruit_tag_reader_example This example presents combined use of the Adafruit tag reader
 *      (@ref adafruit_pn532) library with Type 2 Tag parser (@ref nfc_type_2_tag_parser).

 */

#include <stdbool.h>
#include <stdint.h>

#include "nrf_delay.h"
#include "app_error.h"
#include "bsp.h"
#include "hardfault.h"

#include "adafruit_pn532.h"
#include "nfc_t2t_parser.h"
#include "nfc_ndef_msg_parser.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define TAG_TYPE_2_UID_LENGTH               7                       /// Length of the Tag's UID.
#define TAG_DATA_BUFFER_SIZE                1024                    /// Buffer size for data from a Tag.

#define TAG_DETECT_TIMEOUT                  5000                    /// Timeout for function which searches for a tag.
#define TAG_AFTER_READ_DELAY                2000                    /// Delay after Tag read.

#define TAG_TYPE_2_DATA_AREA_SIZE_OFFSET    T2T_CC_BLOCK_OFFSET + 2 /// Offset of the byte with Tag's Data size.
#define TAG_TYPE_2_DATA_AREA_MULTIPLICATOR  8                       /// Multiplicator for a value stored in the Tag's Data size byte.


/**
 * @brief Function for initializations not directly related to Adafruit.
 */
void utils_setup(void)
{
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
}


/**
 * @brief Function for reading data from a Tag.
 *
 * This function waits for a Tag to appear in the field. When a Tag is detected, all of the pages
 * within a Tag are read.
 */
ret_code_t tag_data_read(uint8_t * buffer, uint32_t buffer_size)
{
    ret_code_t err_code;
    uint8_t i;

    // Buffer for UID.
    uint8_t uid[TAG_TYPE_2_UID_LENGTH] = { 0 };
    uint8_t uid_length                  = TAG_TYPE_2_UID_LENGTH;

    // Not enough size in the buffer to read a tag header.
    if (buffer_size < T2T_FIRST_DATA_BLOCK_OFFSET)
    {
        return NRF_ERROR_NO_MEM;
    }

    // Detect a ISO14443A Tag in the field and initiate a communication. This function activates
    // the NFC RF field. If a Tag is present, its UID is stored in the uid buffer. The
    // UID read from the Tag can not be longer than uidLength value passed to the function.
    // As a result, the uidLength variable returns length of the Tags UID read.
    err_code = adafruit_pn532_read_passive_target_id(PN532_MIFARE_ISO14443A_BAUD,
                                                   uid,
                                                   &uid_length,
                                                   TAG_DETECT_TIMEOUT);
    if (err_code != NRF_SUCCESS)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (uid_length != TAG_TYPE_2_UID_LENGTH)
    {
        return NRF_ERROR_NOT_SUPPORTED;
    }

    // Read pages 0 - 3 to get the header information.
    for (i = 0; i < 4; i++)
    {
        err_code = adafruit_pn532_ntag2xx_read_page(i, buffer + 4 * i);
        if (err_code)
        {
            NRF_LOG_INFO("Failed to read page %d\r\n", i);
            return NRF_ERROR_INTERNAL;
        }
    }

    uint16_t data_bytes_in_tag = TAG_TYPE_2_DATA_AREA_MULTIPLICATOR *
                                     buffer[TAG_TYPE_2_DATA_AREA_SIZE_OFFSET];

    if (data_bytes_in_tag + T2T_FIRST_DATA_BLOCK_OFFSET > buffer_size)
    {
        return NRF_ERROR_NO_MEM;
    }

    uint8_t pages_to_read = data_bytes_in_tag / T2T_BLOCK_SIZE;
    for (i = 0; i < pages_to_read; i++)
    {
        uint16_t offset_for_page = T2T_FIRST_DATA_BLOCK_OFFSET + 4 * i;
        err_code = adafruit_pn532_ntag2xx_read_page(i + 4, buffer + offset_for_page);
        if (err_code)
        {
            NRF_LOG_INFO("Failed to read page %d\r\n", i + 4);
            return NRF_ERROR_INTERNAL;
        }
    }

    return NRF_SUCCESS;
}

/**
 * @brief Function for analyzing NDEF data from a TLV block.
 *
 * This function checks if a TLV block is in NDEF format. If an NDEF block is detected,
 * the NDEF data is parsed and printed.
 */
void ndef_data_analyze(tlv_block_t * p_tlv_block)
{
    uint8_t desc_buf[NFC_NDEF_PARSER_REQIRED_MEMO_SIZE_CALC(10)];
    uint32_t nfc_data_len;
    uint32_t desc_buf_len = sizeof(desc_buf);

    ret_code_t ret_code;

    if (p_tlv_block->tag == TLV_NDEF_MESSAGE)
    {
        nfc_data_len = p_tlv_block->length;

        ret_code = ndef_msg_parser(desc_buf,
                            &desc_buf_len,
                            p_tlv_block->p_value,
                            &nfc_data_len);

        if (ret_code != NRF_SUCCESS)
        {
            NRF_LOG_INFO("Error during parsing a NDEF message.\r\n");
        }

        ndef_msg_printout((nfc_ndef_msg_desc_t *) desc_buf);
    }
}

/**
 * @brief Function for analyzing data from a Tag.
 *
 * This function parses content of a Tag and prints it out.
 */
void tag_data_analyze(uint8_t * buffer)
{
    ret_code_t err_code;

    // Static declaration of Type 2 Tag structure. Maximum of 10 TLV blocks can be read.
    NFC_TYPE_2_TAG_DESC_DEF(test_1, 10);
    type_2_tag_t * test_type_2_tag = &NFC_TYPE_2_TAG_DESC(test_1);

    err_code = type_2_tag_parse(test_type_2_tag, buffer);
    if (err_code == NRF_ERROR_NO_MEM)
    {
        NRF_LOG_INFO("Not enough memory to read whole tag. Printing what've been read.\r\n\r\n");
    }
    else if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_INFO("Error during parsing a tag. Printing what could've been read.\r\n\r\n");
    }

    type_2_tag_printout(test_type_2_tag);

    tlv_block_t * p_tlv_block = test_type_2_tag->p_tlv_block_array;
    uint32_t i;

    for (i = 0; i < test_type_2_tag->tlv_count; i++)
    {
        ndef_data_analyze(p_tlv_block);
        p_tlv_block++;
    }
}

/**
 * @brief Function for waiting specified time after a Tag read operation.
 */
void after_read_delay(void)
{
    ret_code_t err_code;

    // Turn off the RF field.
    err_code = adafruit_pn532_field_off();
    APP_ERROR_CHECK(err_code);
    nrf_delay_ms(TAG_AFTER_READ_DELAY);
}


int main(void)
{
    ret_code_t err_code;

    // Buffer for tag data.
    static uint8_t tag_data[TAG_DATA_BUFFER_SIZE];

    utils_setup();

    err_code = adafruit_pn532_init(false);
    APP_ERROR_CHECK(err_code);

    for (;;)
    {
        err_code = tag_data_read(tag_data, TAG_DATA_BUFFER_SIZE);
        switch (err_code)
        {
            case NRF_SUCCESS:
                tag_data_analyze(tag_data);
                after_read_delay();
                break;
            case NRF_ERROR_NO_MEM:
                NRF_LOG_INFO("Declared buffer is to small to store tag data.\r\n");
                after_read_delay();
                break;
            case NRF_ERROR_NOT_FOUND:
                NRF_LOG_INFO("No Tag found.\r\n");
                // No delay here as we want to search for another tag immediately.
                break;
            case NRF_ERROR_NOT_SUPPORTED:
                NRF_LOG_INFO("Tag not supported.\r\n");
                after_read_delay();
                break;
            default:
                NRF_LOG_INFO("Error during tag read.\r\n");
                err_code = adafruit_pn532_field_off();
                break;
        }
        NRF_LOG_FLUSH();
    }
}

/** @} */ /* End of group nfc_adafruit_tag_reader_example */

