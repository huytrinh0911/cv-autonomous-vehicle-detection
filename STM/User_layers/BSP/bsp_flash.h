/**
 * @file       bsp_flash.h
 * @copyright
 * @license    This project is released under the Fiot License.
 * @version    0.0.0
 * @date
 * @author
 * @author
 *
 * @brief
 *
 * @note
 * @example
 *
 * @example
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __BSP_FLASH_H
#define __BSP_FLASH_H

/* Includes ----------------------------------------------------------- */
#include "common.h"
#include "protocol.pb.h"

/* Public defines ----------------------------------------------------- */

/* Public enumerate/structure ----------------------------------------- */

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */
bool bsp_flash_fuzzy_read(protobuf_fuzzy_coef_t *fuzzy_config);
bool bsp_flash_fuzzy_write(protobuf_fuzzy_coef_t *fuzzy_config);

#endif // __BSP_FLASH_H

/* End of file -------------------------------------------------------- */
