/**
 * @file       bsp_dcmotor_encoder.h
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
#ifndef __BSP_DCMOTOR_ENCODER_H
#define __BSP_DCMOTOR_ENCODER_H

/* Includes ----------------------------------------------------------- */
#include "common.h"

/* Public defines ----------------------------------------------------- */
#define PI                                        3.1415926535897f
#define BSP_DCMOTOR_ENCODER_INITIAL_COUNTER_VALUE (0x8000)

#define BSP_DCMOTOR_SAMPLE_TIME_S (0.02) // second
#define BSP_DCMOTOR_SAMPLE_TIME_US \
  (BSP_DCMOTOR_SAMPLE_TIME_S * 1000000)                 // second
#define BSP_DCMOTOR_ENCODER_PULSE_MODE           (4.0f) // rising
#define BSP_DCMOTOR_ENCODER_PULSE_PER_ROUND      (11.0f)
#define BSP_DCMOTOR_GEAR_MOTOR_RATIO             (171.0f)
#define BSP_DCMOTOR_NUMBER_OF_DRIVING_GEAR_TEETH (54.0f)
#define BSP_DCMOTOR_NUMBER_OF_DRIVEN_GEAR_TEETH  (30.0f)
#define BSP_DCMOTOR_GEAR_WHEEL_RATIO \
  (float)(BSP_DCMOTOR_NUMBER_OF_DRIVING_GEAR_TEETH / BSP_DCMOTOR_NUMBER_OF_DRIVEN_GEAR_TEETH)
#define BSP_DCMOTOR_WHEEL_DIAMETER (6.2f)

/* Public enumerate/structure ----------------------------------------- */
typedef struct
{
  float rps;
  float rpm;
  float omega; // rad/s
} dcmotor_speed_t;

typedef struct
{
  float wheel_diameter;       // cm
  float linear_velocity_sp;   // cm/s
  float linear_velocity_meas; // cm/s
} dcmotor_robot_t;

typedef struct
{
  uint16_t        enc_cnt;
  int32_t         delta_enc_cnt;
  uint32_t        rpwm;
  uint32_t        lpwm;
  dcmotor_speed_t motor_speed;
  dcmotor_speed_t wheel_speed;
  dcmotor_robot_t robot;
} dcmotor_t;

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */
void bsp_dcmotor_encoder_init(void);
void bsp_dcmotor_encoder_take_sample(void);

float bsp_dcmotor_encoder_get_motor_rps(void);
float bsp_dcmotor_encoder_get_motor_rpm(void);
float bsp_dcmotor_encoder_get_motor_omega(void);
float bsp_dcmotor_encoder_get_wheel_rps(void);
float bsp_dcmotor_encoder_get_wheel_rpm(void);
float bsp_dcmotor_encoder_get_wheel_omega(void);
float bsp_dcmotor_encoder_get_robot_linear_velocity(void);
void bsp_dcmotor_encoder_set_robot_linear_velocity(float robot_linear_velocity);

#endif // __BSP_DCMOTOR_ENCODER_H

/* End of file -------------------------------------------------------- */
