/**
 * @file       bsp_utils.h
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
#ifndef __BSP_UTILS_H
#define __BSP_UTILS_H

/* Includes ----------------------------------------------------------- */
#include "common.h"

/* Public defines ----------------------------------------------------- */
#define NUM_DELAY_ENTRIES	4

/* Public enumerate/structure ----------------------------------------- */

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */
void bsp_utils_init(void);
void delay_init(void);
int8_t delay_us(uint32_t us, void *new_callback);
int8_t delay_ms(uint32_t ms, void *new_callback);
void delay_cancel(void *callback);

#endif // __BSP_UTILS_H

/* End of file -------------------------------------------------------- */
