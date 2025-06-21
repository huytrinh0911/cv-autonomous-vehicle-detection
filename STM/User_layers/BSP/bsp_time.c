/**
 * @file       bsp_time.c
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

/* Includes ----------------------------------------------------------- */
#include "bsp_time.h"

/* Private defines ---------------------------------------------------- */

/* Private enumerate/structure ---------------------------------------- */

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */
uint64_t bsp_timestamp_tick_ms = 0;

/* Private variables -------------------------------------------------- */

/* Private function prototypes ---------------------------------------- */

/* Function definitions ----------------------------------------------- */
void bsp_timestamp_set(uint64_t timestamp)
{
    bsp_timestamp_tick_ms = timestamp;
}

uint64_t bsp_timestamp_get(void)
{
    return bsp_timestamp_tick_ms;
}

/* Private definitions ----------------------------------------------- */

/* End of file -------------------------------------------------------- */
