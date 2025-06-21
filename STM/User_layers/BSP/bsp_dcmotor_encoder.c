/**
 * @file       bsp_dcmotor_encoder.c
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

/* Includes ----------------------------------------------------------- */
#include "bsp_dcmotor_encoder.h"
#include "tim.h"

/* Private defines ---------------------------------------------------- */

/* Private enumerate/structure ---------------------------------------- */

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */
static TIM_HandleTypeDef *htim_dcmotor_encoder = &htim8; // APB2  - 168 Mhz
static dcmotor_t          bdcmotor;

/* Private variables -------------------------------------------------- */

/* Private function prototypes ---------------------------------------- */

/* Function definitions ----------------------------------------------- */
void bsp_dcmotor_encoder_init(void)
{
  memset(&bdcmotor, 0, sizeof(dcmotor_t));
  bdcmotor.robot.wheel_diameter = BSP_DCMOTOR_WHEEL_DIAMETER;

  __HAL_TIM_SET_COUNTER(htim_dcmotor_encoder, BSP_DCMOTOR_ENCODER_INITIAL_COUNTER_VALUE);
  HAL_TIM_Encoder_Start(htim_dcmotor_encoder, TIM_CHANNEL_ALL);
}

void bsp_dcmotor_encoder_take_sample(void)
{
  bdcmotor.enc_cnt = __HAL_TIM_GET_COUNTER(htim_dcmotor_encoder);
  __HAL_TIM_SET_COUNTER(htim_dcmotor_encoder, BSP_DCMOTOR_ENCODER_INITIAL_COUNTER_VALUE);

  bdcmotor.delta_enc_cnt =
  (int32_t)bdcmotor.enc_cnt - (int32_t)BSP_DCMOTOR_ENCODER_INITIAL_COUNTER_VALUE;

  bdcmotor.motor_speed.rps =
  ((float)bdcmotor.delta_enc_cnt /
   (BSP_DCMOTOR_ENCODER_PULSE_MODE * BSP_DCMOTOR_ENCODER_PULSE_PER_ROUND *
    BSP_DCMOTOR_GEAR_MOTOR_RATIO)) /
  (float)BSP_DCMOTOR_SAMPLE_TIME_S;
  bdcmotor.motor_speed.rpm   = bdcmotor.motor_speed.rps * 60;
  bdcmotor.motor_speed.omega = bdcmotor.motor_speed.rps * (2 * PI);

  bdcmotor.wheel_speed.rps = bdcmotor.motor_speed.rps * BSP_DCMOTOR_GEAR_WHEEL_RATIO;
  bdcmotor.wheel_speed.rpm = bdcmotor.motor_speed.rpm * BSP_DCMOTOR_GEAR_WHEEL_RATIO;
  bdcmotor.wheel_speed.omega = bdcmotor.motor_speed.omega * BSP_DCMOTOR_GEAR_WHEEL_RATIO;

  bdcmotor.robot.linear_velocity_meas =
  bdcmotor.wheel_speed.rps * (bdcmotor.robot.wheel_diameter * PI);
}

float bsp_dcmotor_encoder_get_motor_rps(void)
{
  return bdcmotor.motor_speed.rps;
}

float bsp_dcmotor_encoder_get_motor_rpm(void)
{
  return bdcmotor.motor_speed.rpm;
}

float bsp_dcmotor_encoder_get_motor_omega(void)
{
  return bdcmotor.motor_speed.omega;
}

float bsp_dcmotor_encoder_get_wheel_rps(void)
{
  return bdcmotor.wheel_speed.rps;
}

float bsp_dcmotor_encoder_get_wheel_rpm(void)
{
  return bdcmotor.wheel_speed.rpm;
}

float bsp_dcmotor_encoder_get_wheel_omega(void)
{
  return bdcmotor.wheel_speed.omega;
}

float bsp_dcmotor_encoder_get_robot_linear_velocity(void)
{
  return bdcmotor.robot.linear_velocity_meas;
}

void bsp_dcmotor_encoder_set_robot_linear_velocity(float robot_linear_velocity)
{
  bdcmotor.robot.linear_velocity_sp = robot_linear_velocity;
}

/* Private definitions ----------------------------------------------- */

/* End of file -------------------------------------------------------- */
