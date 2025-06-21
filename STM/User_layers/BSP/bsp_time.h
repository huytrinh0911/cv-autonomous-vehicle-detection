/**
 * @file       bsp_time.h
 * @copyright
 * @license    
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
#ifndef __BSP_TIME_H
#define __BSP_TIME_H

/* Includes ----------------------------------------------------------- */
#include "common.h"

/* Public defines ----------------------------------------------------- */

/* Public enumerate/structure ----------------------------------------- */

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */
extern uint64_t bsp_timestamp_tick_ms;

/* Public function prototypes ----------------------------------------- */
void     bsp_timestamp_set(uint64_t timestamp);
uint64_t bsp_timestamp_get(void);

#endif // __BSP_TIME_H

/* End of file -------------------------------------------------------- */
