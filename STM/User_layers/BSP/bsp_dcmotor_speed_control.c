/**
 * @file       bsp_dcmotor_speed_control.c
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
#include "bsp_dcmotor_speed_control.h"
#include "tim.h"
#include <math.h>

/* Private defines ---------------------------------------------------- */
#define BSP_DCMOTOR_KP   (1500.0f)
#define BSP_DCMOTOR_KI   (3000.0f)
#define BSP_DCMOTOR_KD   (300.0f)
#define BSP_DCMOTOR_UMAX (16799.0f) // 16800 - 1
#define BSP_DCMOTOR_UMIN (0.0f)

/* Private enumerate/structure ---------------------------------------- */

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */
static TIM_HandleTypeDef *htim_dcmotor_pwm = &htim9; // APB2  - 168 MHz
static dcmotor_pid_t      bdcmotor_pid_ctrl;
static bool               dcmotor_is_run = false;

/* Private function prototypes ---------------------------------------- */
static void bsp_dcmotor_set_pwm(uint32_t cnt, bsp_dcmotor_direction_t dir);

/* Function definitions ----------------------------------------------- */
void bsp_dcmotor_speed_control_init(void)
{
  dcmotor_is_run = false;
  memset(&bdcmotor_pid_ctrl, 0, sizeof(dcmotor_pid_t));

  bdcmotor_pid_ctrl.umax = BSP_DCMOTOR_UMAX;
  bdcmotor_pid_ctrl.umin = BSP_DCMOTOR_UMIN;
  bdcmotor_pid_ctrl.kp   = BSP_DCMOTOR_KP;
  bdcmotor_pid_ctrl.ki   = BSP_DCMOTOR_KI;
  bdcmotor_pid_ctrl.kd   = BSP_DCMOTOR_KD;

  __HAL_TIM_SET_PRESCALER(htim_dcmotor_pwm, 1 - 1);
  __HAL_TIM_SET_AUTORELOAD(htim_dcmotor_pwm, 16800 - 1); // f_tim = 10 kHz
  __HAL_TIM_SET_COUNTER(htim_dcmotor_pwm, ZERO);
  __HAL_TIM_SET_COMPARE(htim_dcmotor_pwm, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(htim_dcmotor_pwm, TIM_CHANNEL_2, 0);

  HAL_TIM_PWM_Start(htim_dcmotor_pwm, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(htim_dcmotor_pwm, TIM_CHANNEL_2);
}

void bsp_dcmotor_speed_control_PID(void)
{
  if (!dcmotor_is_run)
  {
    return;
  }
  
  bdcmotor_pid_ctrl.measure = bsp_dcmotor_encoder_get_motor_rps();
  bdcmotor_pid_ctrl.e2      = bdcmotor_pid_ctrl.e1;
  bdcmotor_pid_ctrl.e1      = bdcmotor_pid_ctrl.e0;
  bdcmotor_pid_ctrl.e0 = bdcmotor_pid_ctrl.set_point - bdcmotor_pid_ctrl.measure;

  bdcmotor_pid_ctrl.u1 = bdcmotor_pid_ctrl.u0;

  bdcmotor_pid_ctrl.u0 = (bdcmotor_pid_ctrl.u1) +
  (bdcmotor_pid_ctrl.kp * (bdcmotor_pid_ctrl.e0 - bdcmotor_pid_ctrl.e1)) +
  (bdcmotor_pid_ctrl.ki * (bdcmotor_pid_ctrl.e0 + bdcmotor_pid_ctrl.e1)) +
  (bdcmotor_pid_ctrl.kd *
   (bdcmotor_pid_ctrl.e0 - 2 * bdcmotor_pid_ctrl.e1 + bdcmotor_pid_ctrl.e2));

  if (isnan(bdcmotor_pid_ctrl.u0))
  {
    bdcmotor_pid_ctrl.u0 = bdcmotor_pid_ctrl.u1;
  }

  if (bdcmotor_pid_ctrl.u0 > bdcmotor_pid_ctrl.umax)
  {
    bdcmotor_pid_ctrl.u0 = bdcmotor_pid_ctrl.umax;
  }

  if (bdcmotor_pid_ctrl.u0 < bdcmotor_pid_ctrl.umin)
  {
    bdcmotor_pid_ctrl.u0 = bdcmotor_pid_ctrl.umin;
  }

  if (bdcmotor_pid_ctrl.set_point > 0)
  {
    bsp_dcmotor_set_pwm((uint32_t)bdcmotor_pid_ctrl.u0, BSP_DCMOTOR_FORWARD);
  }
  else if (bdcmotor_pid_ctrl.set_point == 0)
  {
    bsp_dcmotor_set_pwm(0, BSP_DCMOTOR_STOP);
    bdcmotor_pid_ctrl.u0 = 0;
  }
  else // no use
  {
    //    bsp_dcmotor_set_pwm((uint32_t)bdcmotor_pid_ctrl.u0, BSP_DCMOTOR_BACKWARD);
  }
}

void bsp_dcmotor_speed_control_set_robot_velocity(float robot_linear_velocity)
{
  if (!dcmotor_is_run)
  {
    dcmotor_is_run = true;
  }
  bdcmotor_pid_ctrl.set_point =
  (robot_linear_velocity / (((PI * BSP_DCMOTOR_WHEEL_DIAMETER)) * BSP_DCMOTOR_GEAR_WHEEL_RATIO));
  bsp_dcmotor_encoder_set_robot_linear_velocity(robot_linear_velocity);
}

void bsp_dcmotor_clear(void)
{
  bdcmotor_pid_ctrl.e0        = 0;
  bdcmotor_pid_ctrl.e1        = 0;
  bdcmotor_pid_ctrl.e2        = 0;
  bdcmotor_pid_ctrl.u0        = 0;
  bdcmotor_pid_ctrl.u1        = 0;
  bdcmotor_pid_ctrl.measure   = 0;
  bdcmotor_pid_ctrl.set_point = 0;
}

void bsp_dcmotor_stop(void)
{
  dcmotor_is_run = false;
  bsp_dcmotor_set_pwm(0, BSP_DCMOTOR_STOP);
}

/* Private definitions ----------------------------------------------- */
static void bsp_dcmotor_set_pwm(uint32_t cnt, bsp_dcmotor_direction_t dir)
{
  switch (dir)
  {
  case BSP_DCMOTOR_FORWARD:
  {
    __HAL_TIM_SET_COMPARE(htim_dcmotor_pwm, TIM_CHANNEL_1, cnt);
    __HAL_TIM_SET_COMPARE(htim_dcmotor_pwm, TIM_CHANNEL_2, 0);

    break;
  }
  case BSP_DCMOTOR_BACKWARD:
  {
    __HAL_TIM_SET_COMPARE(htim_dcmotor_pwm, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(htim_dcmotor_pwm, TIM_CHANNEL_2, cnt);
    break;
  }
  default:
  {
    __HAL_TIM_SET_COMPARE(htim_dcmotor_pwm, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(htim_dcmotor_pwm, TIM_CHANNEL_2, 0);
    break;
  }
  }
}

/* End of file -------------------------------------------------------- */
