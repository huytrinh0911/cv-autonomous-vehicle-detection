/**
 * @file       bsp_servo.h
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
#ifndef __BSP_SERVO_H
#define __BSP_SERVO_H

/* Includes ----------------------------------------------------------- */
#include "common.h"

/* Public defines ----------------------------------------------------- */

/* Public enumerate/structure ----------------------------------------- */

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */
void bsp_servo_init(void);
void bsp_servo_set(float degree);
void bsp_servo_camera_set(float degree);

#endif // __BSP_SERVO_H

/* End of file -------------------------------------------------------- */
