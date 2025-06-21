/**
 * @file        fuzzy_controller.h
 * @copyright
 * @license
 * @version     0.0.0
 * @date
 * @author      Khoi Nguyen Thanh
 * @brief       none
 *
 * @note        none
 *
 * @example     none
 *
 */
/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __FZ_CTRLR_H
#define __FZ_CTRLR_H

/* Includes ----------------------------------------------------------- */
#include "common.h"
#include "fuzzy.h"
#include "platform.h"
#include "robot_control.h"

// -------------
#include "protocol.pb.h"

/* Public defines ----------------------------------------------------- */
#define FZ_CTRLR_SAMPLE_TIME_S                        ((1.0f) / (30.0f))
#define FZ_CTRLR_COORDINATE_CENTER_SETPOINT           (645.0f) 
#define FZ_CTRLR_COORDINATE_TURN_LEFT_LEFT_SETPOINT   (180.0f)  // x_left_sp
#define FZ_CTRLR_COORDINATE_TURN_RIGHT_RIGHT_SETPOINT (1120.0f) // x_right_sp


/** Controller coefficient */
#define FZ_CTRLR_ERROR_NOR_COEF      (284.0f) // k1
#define FZ_CTRLR_ERROR_DOT_NOR_COEF  (55.0f)  // k2
#define FZ_CTRLR_PHI_NOR_COEF        (45.0f)  
#define FZ_CTRLR_PHI_DOT_NOR_COEF    (90.0f)  
#define FZ_CTRLR_THETA_DENOR_COEF    (55.0f)
#define FZ_CTRLR_VELOCITY_NOR_COEF   (20.0f)
#define FZ_CTRLR_VELOCITY_DENOR_COEF (20.0f)

#define E_C1 (0.25f)
#define E_C2 (0.50f)
#define E_C3 (0.75f)

#define EDOT_C1 (0.25f)
#define EDOT_C2 (0.50f)
#define EDOT_C3 (0.75f)

#define THETA_C1 (0.26f)
#define THETA_C2 (0.50f)
#define THETA_C3 (0.75f)

#define VELO_C1 (0.25f)
#define VELO_C2 (0.50f)
#define VELO_C3 (0.75f)

#define Y_THETA_C1 (0.25f)
#define Y_THETA_C2 (0.5f)

#define Y_THETA_4_C1 (0.2f)
#define Y_THETA_4_C2 (0.4f)
#define Y_THETA_4_C3 (0.6f)
#define Y_THETA_4_C4 (0.8f)

#define Y_VELO_C1 (0.6f)
#define Y_VELO_C2 (0.8f)

/* Public enumerate/structure ----------------------------------------- */
typedef struct
{
  float beta[LANGUAGE_5_NUM][LANGUAGE_5_NUM];
  protobuf_fuzzy_7_rule_t y;
} fuzzy_controller_theta_t;

typedef struct
{
  float beta[LANGUAGE_3_NUM][LANGUAGE_3_NUM][LANGUAGE_5_NUM][LANGUAGE_5_NUM];
  protobuf_fuzzy_11_rule_t y;
} fuzzy_controller_theta_4_input_t;

typedef struct
{
  float beta[LANGUAGE_3_NUM][LANGUAGE_5_NUM][LANGUAGE_5_NUM];
  protobuf_fuzzy_9_rule_t y;
} fuzzy_controller_theta_3_input_t;

typedef struct
{
  float beta[LANGUAGE_5_NUM][LANGUAGE_5_NUM];
  protobuf_fuzzy_7_rule_t y;
} fuzzy_controller_theta_2_input_t;

typedef struct 
{
  float beta[LANGUAGE_5_NUM][LANGUAGE_5_NUM];
  protobuf_fuzzy_3_positive_rule_t y;
} fuzzy_controllerlinear_velocity_t;

typedef struct
{
  protobuf_robot_var_info_t           var;
  protobuf_fuzzy_normalization_coef_t coef;
} fuzzy_controller_t;

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */
void fuzzy_controller_init(void);

void fuzzy_controller_process_2_input(protobuf_robot_var_info_t *var, float theta_offset);
void fuzzy_controller_process_3_input(protobuf_robot_var_info_t *var, float theta_offset);

void  fuzzy_controller_clear(void);
bool  fuzzy_controller_coef_get(protobuf_fuzzy_coef_t *fz_coef);
void  fuzzy_controller_coef_set(protobuf_fuzzy_coef_t *fz_coef);
void  fuzzy_controller_max_speed_set(float max_speed);
float fuzzy_controller_max_speed_get(void);
void  fuzzy_controller_max_rotation_angle_set(float max_rotation_angle);
void  fuzzy_controller_set_following_coef(void);
void  fuzzy_controller_set_turning_coef(void);

#endif // __FZ_CTRLR_H

/* End of file -------------------------------------------------------- */
