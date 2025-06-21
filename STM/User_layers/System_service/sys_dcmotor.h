/**
 * @file       sys_dcmotor.h
 * @copyright
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
#ifndef __SYS_DCMOTOR_H
#define __SYS_DCMOTOR_H

/* Includes ----------------------------------------------------------- */
#include "bsp_dcmotor_encoder.h"
#include "bsp_dcmotor_speed_control.h"
#include "common.h"

/* Public defines ----------------------------------------------------- */

/* Public enumerate/structure ----------------------------------------- */

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */
void sys_dcmotor_init(void);
void sys_dcmotor_loop(void);
void sys_dcmotor_stop(void);
void sys_dcmotor_speed_get(volatile float *speed);
void sys_dcmotor_speed_set(float speed);

#endif // __SYS_DCMOTOR_H

/* End of file -------------------------------------------------------- */
