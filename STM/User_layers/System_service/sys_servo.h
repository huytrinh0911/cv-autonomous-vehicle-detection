/**
 * @file       sys_servo.h
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
#ifndef __SYS_SERVO_H
#define __SYS_SERVO_H

/* Includes ----------------------------------------------------------- */
#include "bsp_servo.h"

/* Public defines ----------------------------------------------------- */
#define SYS_SERVO_SUCCESS (0x00000000)
#define SYS_SERVO_ERROR   (0xFFFFFFFF)

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  SERVO_DRIVE,
  SERVO_CAMERA,
  SERVO_ALL
} servo_type_t;

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */
void sys_servo_init(void);
void sys_servo_set(float degree, servo_type_t servo_type);
// void sys_servo_set(float degree);
void sys_servo_loop(void);

#endif // __SYS_SERVO_H

/* End of file -------------------------------------------------------- */
