#ifndef ANT_DFU_CONSTRAINS_H__
#define ANT_DFU_CONSTRAINS_H__

#ifdef __cplusplus
extern "C" {
#endif

/**@file
 *
 * @defgroup ant_dfu_constrains Memory constraints for ANT DFU
 * @{
 *
 * @ingroup nrf_dfu
 *
 */

#ifdef NRF51 // nrf51 @ S210
    /** End of nRF51 flash */
    #define NRF5x_FLASH_END                 0x00040000UL

    /** This field should correspond to the start address of the bootloader, found in the
        UICR.BOOTLOADERADDR, 0x10001014, register. This value is used for a sanity check,
        so the bootloader will fail immediately if this value differs from the runtime value.
        The value is used to determine the maximum DFU region size. */
    #define BOOTLOADER_REGION_START         0x0003B800

    /** Page location of the bootloader settings address. */
    #define BOOTLOADER_SETTINGS_ADDRESS     0x0003FC00

    /** Size of a flash codepage. Used for size of the reserved flash space in the bootloader
        region. Will be runtime checked against NRF_UICR->CODEPAGESIZE to ensure the region is
        correct. */
    #define CODE_PAGE_SIZE                  1024

#elif defined(NRF52) // nrf52 @ S212 and S332
    /** End of nRF52 flash */
    #define NRF5x_FLASH_END                 0x00080000UL

    /** This field should correspond to the start address of the bootloader, found in the
        UICR.BOOTLOADERADDR, 0x10001014, register. This value is used for a sanity check,
        so the bootloader will fail immediately if this value differs from the runtime value.
        The value is used to determine the maximum DFU region size. */
    #define BOOTLOADER_REGION_START         0x00079000

    /** Page location of the bootloader settings address. */
    #define BOOTLOADER_SETTINGS_ADDRESS     (NRF5x_FLASH_END - CODE_PAGE_SIZE)

    /** The sd_mbr_command call may require parameters to be retained in a separate flash page provided by the application.
        The uicr register UICR.NRFFW[1] must be set to an address corresponding to this page in the application flash space. */
    #define BOOTLOADER_MBR_RETAINING_PAGE_ADDRESS     (BOOTLOADER_SETTINGS_ADDRESS - CODE_PAGE_SIZE)

    /** Size of a flash codepage. Used for size of the reserved flash space in the bootloader
        region. Will be runtime checked against NRF_UICR->CODEPAGESIZE to ensure the region is
        correct. */
    #define CODE_PAGE_SIZE                  4096

#else
    #error Unknown platform for ANT DFU
#endif

/**@} */


#ifdef __cplusplus
}
#endif

#endif //ANT_DFU_CONSTRAINS_H__
