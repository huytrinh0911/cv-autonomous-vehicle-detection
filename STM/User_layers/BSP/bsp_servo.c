/**
 * @file       bsp_servo.c
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
#include "bsp_servo.h"
#include "bsp_utils.h"
#include "tim.h"

/* Private defines ---------------------------------------------------- */
// #define BSP_SERVO_INITIAL_ANGLE_VALUE        (4499)
#define BSP_SERVO_INITIAL_ANGLE_VALUE        (4332)
#define BSP_SERVO_CAMERA_INITIAL_ANGLE_VALUE (4299)
#define BSP_SERVO_OFFSET_DEGREE              (5.0f)
#define BSP_SERVO_CAMERA_OFFSET_DEGREE       (6.0f)
#define BSP_SERVO_0_DEGREE_ANGLE_VALUE       (1499)
#define BSP_SERVO_180_DEGREE_ANGLE_VALUE     (7499)
#define BSP_SERVO_STEP_DEGREE                (5)
#define BSP_SERVO_TIME_DELAY_CAMERA          (60)

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  float angle_expect;
  float angle_current;
  int16_t step_num;
  int16_t step_value;
} servo_t;

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */
static TIM_HandleTypeDef *htim_servo = &htim1;  // APB2 - 168 MHz

static servo_t servo_camera;

/* Private function prototypes ---------------------------------------- */
void bsp_servo_camera_callback_01(void);
void bsp_servo_camera_callback_02(void);

/* Function definitions ----------------------------------------------- */
void bsp_servo_init(void)
{
  __HAL_TIM_SET_PRESCALER(htim_servo, 56 - 1);
  __HAL_TIM_SET_AUTORELOAD(htim_servo, 60000 - 1);
  
  // __HAL_TIM_DISABLE_OCxPRELOAD(htim_servo, TIM_CHANNEL_1);
  __HAL_TIM_DISABLE_OCxPRELOAD(htim_servo, TIM_CHANNEL_4);
  
  __HAL_TIM_SET_COMPARE(htim_servo, TIM_CHANNEL_1, BSP_SERVO_INITIAL_ANGLE_VALUE);
  __HAL_TIM_SET_COMPARE(htim_servo, TIM_CHANNEL_4, BSP_SERVO_CAMERA_INITIAL_ANGLE_VALUE);
  
  HAL_TIM_PWM_Start(htim_servo, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(htim_servo, TIM_CHANNEL_4);

  servo_camera.angle_current = 0;
  servo_camera.angle_expect  = 0;
  servo_camera.step_num      = 0;
}

void bsp_servo_set(float degree)
{
  HAL_TIM_PWM_Stop(htim_servo, TIM_CHANNEL_1);

  uint16_t degree_val =
  (((((-(degree + BSP_SERVO_OFFSET_DEGREE) + 90.0f) * (1.0f / 90.0f)) + 0.5f) / 20.0f) * 60000.0f) -
  1.0f;

  __HAL_TIM_SET_COMPARE(htim_servo, TIM_CHANNEL_1, degree_val);

  HAL_TIM_PWM_Start(htim_servo, TIM_CHANNEL_1);
}

void bsp_servo_camera_set(float degree)
{
  servo_camera.angle_expect = degree;
  servo_camera.step_num =
  (servo_camera.angle_expect - servo_camera.angle_current) / BSP_SERVO_STEP_DEGREE;

  if (servo_camera.step_num == 0)
  {
    return;
  }
  else if (servo_camera.step_num > 0)
  {
    servo_camera.step_value = 5;
  }
  else if (servo_camera.step_num < 0)
  {
    servo_camera.step_value = -5;
  }

  servo_camera.angle_current += servo_camera.step_value;
  servo_camera.step_num -= (servo_camera.step_value / BSP_SERVO_STEP_DEGREE);

  HAL_TIM_PWM_Stop(htim_servo, TIM_CHANNEL_4);

  uint16_t degree_val =
  (((((-(servo_camera.angle_current + BSP_SERVO_CAMERA_OFFSET_DEGREE) + 90.0f) * (1.0f / 90.0f)) + 0.5f) / 20.0f) * 60000.0f) - 1.0f;

  __HAL_TIM_SET_COMPARE(htim_servo, TIM_CHANNEL_4, degree_val);

  HAL_TIM_PWM_Start(htim_servo, TIM_CHANNEL_4);

  if (servo_camera.step_num == 0)
    return;

  delay_ms(BSP_SERVO_TIME_DELAY_CAMERA, bsp_servo_camera_callback_01);
}

/* Private definitions ----------------------------------------------- */
void bsp_servo_camera_callback_01(void)
{
  servo_camera.angle_current += servo_camera.step_value;
  servo_camera.step_num -= (servo_camera.step_value / BSP_SERVO_STEP_DEGREE);

  HAL_TIM_PWM_Stop(htim_servo, TIM_CHANNEL_4);

  uint16_t degree_val =
  (((((-(servo_camera.angle_current + BSP_SERVO_CAMERA_OFFSET_DEGREE) + 90.0f) * (1.0f / 90.0f)) + 0.5f) / 20.0f) * 60000.0f) - 1.0f;

  __HAL_TIM_SET_COMPARE(htim_servo, TIM_CHANNEL_4, degree_val);

  HAL_TIM_PWM_Start(htim_servo, TIM_CHANNEL_4);

  if (servo_camera.step_num == 0)
    return;

  delay_ms(BSP_SERVO_TIME_DELAY_CAMERA, bsp_servo_camera_callback_02);
}

void bsp_servo_camera_callback_02(void)
{
  servo_camera.angle_current += servo_camera.step_value;
  servo_camera.step_num -= (servo_camera.step_value / BSP_SERVO_STEP_DEGREE);

  HAL_TIM_PWM_Stop(htim_servo, TIM_CHANNEL_4);

  uint16_t degree_val =
  (((((-(servo_camera.angle_current + BSP_SERVO_CAMERA_OFFSET_DEGREE) + 90.0f) * (1.0f / 90.0f)) + 0.5f) / 20.0f) * 60000.0f) - 1.0f;

  __HAL_TIM_SET_COMPARE(htim_servo, TIM_CHANNEL_4, degree_val);

  HAL_TIM_PWM_Start(htim_servo, TIM_CHANNEL_4);

  if (servo_camera.step_num == 0)
    return;

  delay_ms(BSP_SERVO_TIME_DELAY_CAMERA, bsp_servo_camera_callback_01);
}

/* End of file -------------------------------------------------------- */
