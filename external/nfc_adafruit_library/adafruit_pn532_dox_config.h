/**
 *
 * @defgroup adafruit_pn532_config Adafruit PN532 implementation for nRF5x configuration
 * @{
 * @ingroup adafruit_pn532
 */
/** @brief Enabling PN532 module
 *
 *  Set to 1 to activate.
 *
 * @note This is an NRF_CONFIG macro.
 */
#define ADAFRUIT_PN532_ENABLED

/** @brief Pin number
 *
 *  Minimum value: 0
 *  Maximum value: 31
 *
 * @note This is an NRF_CONFIG macro.
 */
#define PN532_IRQ


/** @brief Pin number
 *
 *  Minimum value: 0
 *  Maximum value: 31
 *
 * @note This is an NRF_CONFIG macro.
 */
#define PN532_RESET


/** @brief Pin number
 *
 *  Minimum value: 0
 *  Maximum value: 31
 *
 * @note This is an NRF_CONFIG macro.
 */
#define PN532_CONFIG_SCL


/** @brief Pin number
 *
 *  Minimum value: 0
 *  Maximum value: 31
 *
 * @note This is an NRF_CONFIG macro.
 */
#define PN532_CONFIG_SDA


/** @brief TWI instance to be used
 *
 *  Following options are available:
 * - 0
 * - 1
 * - 2
 *
 * @note This is an NRF_CONFIG macro.
 */
#define PN532_CONFIG_TWI_INSTANCE



/** @} */
