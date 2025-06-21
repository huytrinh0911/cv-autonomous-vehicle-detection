/**
 * @file       bsp_dcmotor_speed_control.h
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
#ifndef __BSP_DCMOTOR_SPEED_CONTROL_H
#define __BSP_DCMOTOR_SPEED_CONTROL_H

/* Includes ----------------------------------------------------------- */
#include "bsp_dcmotor_encoder.h"
#include "common.h"

/* Public defines ----------------------------------------------------- */

/* Public enumerate/structure ----------------------------------------- */
typedef struct
{
  float set_point;
  float measure;
  float e0;
  float e1;
  float e2;
  float u0;
  float u1;
  float umax;
  float umin;
  float kp;
  float ki;
  float kd;
} dcmotor_pid_t;

typedef enum
{
  BSP_DCMOTOR_FORWARD = 0,
  BSP_DCMOTOR_BACKWARD,
  BSP_DCMOTOR_STOP
} bsp_dcmotor_direction_t;

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */
void bsp_dcmotor_speed_control_init(void);
void bsp_dcmotor_speed_control_PID(void);
void bsp_dcmotor_speed_control_set_robot_velocity(float robot_linear_velocity); // cm/s
void bsp_dcmotor_clear(void);
void bsp_dcmotor_stop(void);

#endif // __BSP_DCMOTOR_SPEED_CONTROL_H

/* End of file -------------------------------------------------------- */
