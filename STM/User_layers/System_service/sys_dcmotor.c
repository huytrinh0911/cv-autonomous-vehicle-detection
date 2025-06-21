/**
 * @file       sys_dcmotor.c
 * @copyright
 * @license    This project is  under the Fiot License.
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
#include "sys_dcmotor.h"
#include "sys_network.h"
#include "sys_uart.h"
#include "tim.h"

/* Private defines ---------------------------------------------------- */
#define SPEED_UPDATE_INTERVAL  (250)    // ms

/* Private enumerate/structure ---------------------------------------- */

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */
volatile bool sdcmotor_take_sample_flag = false;

/* Private variables -------------------------------------------------- */
static TIM_HandleTypeDef *htim_dcmotor_sample = &htim2; // APB1 - 84 MHz

/* Private function prototypes ---------------------------------------- */
static void sys_dcmotor_callback(TIM_HandleTypeDef *htim);
static void sys_dcmotor_take_sample(void);

/* Function definitions ----------------------------------------------- */
void sys_dcmotor_init(void)
{
  bsp_dcmotor_encoder_init();
  bsp_dcmotor_speed_control_init();

  HAL_TIM_RegisterCallback(htim_dcmotor_sample, HAL_TIM_PERIOD_ELAPSED_CB_ID,
                           sys_dcmotor_callback);
  __HAL_TIM_SET_PRESCALER(htim_dcmotor_sample, 84 - 1);
  __HAL_TIM_SET_AUTORELOAD(htim_dcmotor_sample, BSP_DCMOTOR_SAMPLE_TIME_US - 1); // f_tim = 50 Hz
  __HAL_TIM_SET_COUNTER(htim_dcmotor_sample, ZERO);
  __HAL_TIM_ENABLE(htim_dcmotor_sample);
  __HAL_TIM_ENABLE_IT(htim_dcmotor_sample, TIM_IT_UPDATE);
  sys_dcmotor_stop();
}

void sys_dcmotor_loop(void)
{
  if (sdcmotor_take_sample_flag)
  {
    sys_dcmotor_take_sample();
  }
}

void sys_dcmotor_stop(void)
{
  bsp_dcmotor_stop();
  bsp_dcmotor_clear();
}

void sys_dcmotor_speed_get(volatile float *speed)
{
  *speed = bsp_dcmotor_encoder_get_robot_linear_velocity();
}

void sys_dcmotor_speed_set(float speed)
{
  bsp_dcmotor_speed_control_set_robot_velocity(speed);
}

/* Private definitions ----------------------------------------------- */
static void sys_dcmotor_callback(TIM_HandleTypeDef *htim)
{
  sdcmotor_take_sample_flag = true;
}

static void sys_dcmotor_take_sample(void)
{
  sdcmotor_take_sample_flag = false;
  bsp_dcmotor_encoder_take_sample();
  bsp_dcmotor_speed_control_PID();
}

/* End of file -------------------------------------------------------- */
