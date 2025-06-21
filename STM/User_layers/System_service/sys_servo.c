/**
 * @file       sys_servo.c
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

/* Includes ----------------------------------------------------------- */
#include "sys_servo.h"

/* Private defines ---------------------------------------------------- */
#define SYS_SERVO_CBUFFER_SIZE (20) /* Size of circular buffer */

/* Private enumerate/structure ---------------------------------------- */

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */

/* Private function prototypes ---------------------------------------- */

/* Function definitions ----------------------------------------------- */
void sys_servo_init(void)
{
  bsp_servo_init();
}

void sys_servo_set(float degree, servo_type_t servo_type)
{
  if (degree > 70)
  {
    degree = 70;
  }
  if (degree < -70)
  {
    degree = -70;
  }

  switch (servo_type)
  {
  case SERVO_DRIVE:
  {
    bsp_servo_set(degree);
    break;
  }
  case SERVO_CAMERA:
  {
    bsp_servo_camera_set(degree);
    break;
  }
  case SERVO_ALL:
  {
    bsp_servo_set(degree);
    bsp_servo_camera_set(degree);
    break;
  }
  default:
  {
    break;
  }
  }

  return;
}

/* Private definitions ----------------------------------------------- */

/* End of file -------------------------------------------------------- */
