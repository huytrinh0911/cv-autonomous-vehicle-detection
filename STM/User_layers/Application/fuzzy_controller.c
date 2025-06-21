/**
 * @file        fuzzy_controller.c
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

/* Includes ----------------------------------------------------------- */
#include "fuzzy_controller.h"
#include "sys_dcmotor.h"
#include "sys_servo.h"
#include "robot_control.h"
#include "sys_network_cmd.h"
#include "bsp_flash.h"
#include "platform.h"
#include <math.h>

// -------------
#include "protocol.pb.h"

/* Private defines ---------------------------------------------------- */
#define CHECK_NAN_INF

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  float                        beta[LANGUAGE_5_NUM];
  protobuf_fuzzy_5_rule_coef_t coef;
} fuzzy_controller_5_rule_t;

typedef struct
{
  float                        beta[LANGUAGE_3_NUM];
  protobuf_fuzzy_3_rule_coef_t coef;
} fuzzy_controller_3_rule_t;

typedef struct
{
  float                                 beta[PO_LANGUAGE_NUM];
  protobuf_fuzzy_3_positive_coef_rule_t coef;
} fuzzy_controller_3_positive_rule_t;

typedef enum
{
  FZ_VAR_E            = 0,
  FZ_VAR_E_NOR        = 1,
  FZ_VAR_E_DOT        = 2,
  FZ_VAR_E_EDOT_NOR   = 3,
  FZ_VAR_THETA_NOR    = 4,
  FZ_VAR_THETA        = 5,
  FZ_VAR_VELO_NOR     = 6,
  FZ_VAR_VELO_SP_NOR  = 7,
  FZ_VAR_VELO_SP      = 8,
  FZ_VAR_E_PHI_NOR    = 9,
} fuzzy_var_type_t;

/* Private macros ----------------------------------------------------- */
#define GET_DERIVATIVE(_VAL_, _PRE_VAL_, _SAMPLE_TIME_MS) \
  (float)((float)((_VAL_ - _PRE_VAL_) / (float)((float)_SAMPLE_TIME_MS / 1000)))

#define COEF_TRAPEZIUM_RULE_INFO(_VAR_, _VAL_L_, _VAL_CL_, _VAL_CR_, _VAL_R_) \
  _VAR_.left         = _VAL_L_;                                     \
  _VAR_.center_left  = _VAL_CL_;                                    \
  _VAR_.center_right = _VAL_CR_;                                    \
  _VAR_.right        = _VAL_R_;

#define Y_5_RULE_INFO(_VAR_, _VAR_NB_, _VAR_NS_, _VAR_ZE_, _VAR_PS_, _VAR_PB_) \
  _VAR_.NB = _VAR_NB_;                                                         \
  _VAR_.NS = _VAR_NS_;                                                         \
  _VAR_.ZE = _VAR_ZE_;                                                         \
  _VAR_.PS = _VAR_PS_;                                                         \
  _VAR_.PB = _VAR_PB_;

#define Y_3_POS_RULE_INFO(_VAR_, _VAR_PS_, _VAR_PM_, _VAR_PB_) \
  _VAR_.PS = _VAR_PS_;                                         \
  _VAR_.PM = _VAR_PM_;                                         \
  _VAR_.PB = _VAR_PB_;

#define Y_7_RULE_INFO(_VAR_, _VAR_NB_, _VAR_NM_, _VAR_NS_, _VAR_ZE_, _VAR_PS_, \
                      _VAR_PM_, _VAR_PB_)                                      \
  _VAR_.NB = _VAR_NB_;                                                         \
  _VAR_.NM = _VAR_NM_;                                                         \
  _VAR_.NS = _VAR_NS_;                                                         \
  _VAR_.ZE = _VAR_ZE_;                                                         \
  _VAR_.PS = _VAR_PS_;                                                         \
  _VAR_.PM = _VAR_PM_;                                                         \
  _VAR_.PB = _VAR_PB_;

#define Y_9_RULE_INFO(_VAR_, _VAR_NB_, _VAR_NM_, _VAR_NS_, _VAR_NVS_,    \
                      _VAR_ZE_, _VAR_PVS_, _VAR_PS_, _VAR_PM_, _VAR_PB_) \
  _VAR_.NB  = _VAR_NB_;                                                  \
  _VAR_.NM  = _VAR_NM_;                                                  \
  _VAR_.NS  = _VAR_NS_;                                                  \
  _VAR_.NVS = _VAR_NVS_;                                                 \
  _VAR_.ZE  = _VAR_ZE_;                                                  \
  _VAR_.PVS = _VAR_PVS_;                                                 \
  _VAR_.PS  = _VAR_PS_;                                                  \
  _VAR_.PM  = _VAR_PM_;                                                  \
  _VAR_.PB  = _VAR_PB_;

#define Y_11_RULE_INFO(_VAR_, _VAR_NVB_, _VAR_NB_, _VAR_NM_, _VAR_NS_, _VAR_NVS_,    \
                       _VAR_ZE_, _VAR_PVS_, _VAR_PS_, _VAR_PM_, _VAR_PB_, _VAR_PVB_) \
  _VAR_.NVB = _VAR_NVB_;                                                             \
  _VAR_.NB  = _VAR_NB_;                                                              \
  _VAR_.NM  = _VAR_NM_;                                                              \
  _VAR_.NS  = _VAR_NS_;                                                              \
  _VAR_.NVS = _VAR_NVS_;                                                             \
  _VAR_.ZE  = _VAR_ZE_;                                                              \
  _VAR_.PVS = _VAR_PVS_;                                                             \
  _VAR_.PS  = _VAR_PS_;                                                              \
  _VAR_.PM  = _VAR_PM_;                                                              \
  _VAR_.PB  = _VAR_PB_;                                                              \
  _VAR_.PVB = _VAR_PVB_;

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */
static volatile fuzzy_controller_t fz_ctrlr;

static fuzzy_controller_5_rule_t fz_e_rule;
static fuzzy_controller_5_rule_t fz_e_dot_rule;
static fuzzy_controller_3_rule_t fz_e_phi_rule;

static fuzzy_controller_5_rule_t          fz_theta_rule;
static fuzzy_controller_3_positive_rule_t fz_velo_rule;

static fuzzy_controller_theta_2_input_t  fz_theta_2_input; 
static fuzzy_controller_theta_3_input_t  fz_theta_3_input; 
static fuzzy_controllerlinear_velocity_t fz_velo;

static uint32_t start_timestamp = 0;
static uint32_t duration = 0;

/* Private function prototypes ---------------------------------------- */
static float fuzzy_controller_theta_process_2_input(float e_nor, float e_dot_nor);
static float fuzzy_controller_theta_process_3_input(float e_nor, float e_dot_nor, float e_phi_nor);
//static float fuzzy_controller_theta_process_4_input(float e_nor, float e_dot_nor, float phi_nor, float phi_dot_nor);
static float fuzzy_controller_linear_velocity_process(float theta_nor, float current_velocity_nor);
static void  fuzzy_check_valid(float value, fuzzy_var_type_t var_type);

/* Function definitions ----------------------------------------------- */
void fuzzy_controller_init(void)
{
  fz_e_rule.coef.has_NB = true;
  fz_e_rule.coef.has_NS = true;
  fz_e_rule.coef.has_PB = true;
  fz_e_rule.coef.has_PS = true;
  fz_e_rule.coef.has_ZE = true;

  fz_e_dot_rule.coef.has_NB = true;
  fz_e_dot_rule.coef.has_NS = true;
  fz_e_dot_rule.coef.has_PB = true;
  fz_e_dot_rule.coef.has_PS = true;
  fz_e_dot_rule.coef.has_ZE = true;

  fz_e_phi_rule.coef.has_NE = true;
  fz_e_phi_rule.coef.has_PO = true;
  fz_e_phi_rule.coef.has_ZE = true;

  fz_e_phi_rule.coef.has_NE = true;
  fz_e_phi_rule.coef.has_PO = true;
  fz_e_phi_rule.coef.has_ZE = true;

  fz_theta_rule.coef.has_NB = true;
  fz_theta_rule.coef.has_NS = true;
  fz_theta_rule.coef.has_PB = true;
  fz_theta_rule.coef.has_PS = true;
  fz_theta_rule.coef.has_ZE = true;

  fz_velo_rule.coef.has_PB = true;
  fz_velo_rule.coef.has_PM = true;
  fz_velo_rule.coef.has_PS = true;

  protobuf_fuzzy_coef_t fuzzy_config_temp;

if (fuzzy_controller_coef_get(&fuzzy_config_temp))
{
  fz_ctrlr.coef.e_nor     = fuzzy_config_temp.nor_coef.e_nor;
  fz_ctrlr.coef.e_dot_nor = fuzzy_config_temp.nor_coef.e_dot_nor;
  fz_ctrlr.coef.e_nor_following_state = fuzzy_config_temp.nor_coef.e_nor_following_state;
  fz_ctrlr.coef.e_dot_nor_following_state = fuzzy_config_temp.nor_coef.e_dot_nor_following_state;
  fz_ctrlr.coef.e_nor_turning_state = fuzzy_config_temp.nor_coef.e_nor_turning_state;
  fz_ctrlr.coef.e_dot_nor_turning_state = fuzzy_config_temp.nor_coef.e_dot_nor_turning_state;
  fz_ctrlr.coef.e_phi_nor     = fuzzy_config_temp.nor_coef.e_phi_nor;
  fz_ctrlr.coef.theta_denor   = FZ_CTRLR_THETA_DENOR_COEF;
  fz_ctrlr.coef.velo_nor      = FZ_CTRLR_VELOCITY_NOR_COEF;
  fz_ctrlr.coef.velo_sp_denor = FZ_CTRLR_VELOCITY_DENOR_COEF;

  // memcpy((uint8_t *)&fz_ctrlr.coef,       (uint8_t *)&fz_coef->nor_coef, sizeof(protobuf_fuzzy_normalization_coef_t));
  memcpy((uint8_t *)&fz_e_rule.coef,        (uint8_t *)&fuzzy_config_temp.e_in_rule,                sizeof(protobuf_fuzzy_5_rule_coef_t));
  memcpy((uint8_t *)&fz_e_dot_rule.coef,    (uint8_t *)&fuzzy_config_temp.e_dot_in_rule,            sizeof(protobuf_fuzzy_5_rule_coef_t));
  memcpy((uint8_t *)&fz_theta_rule.coef,    (uint8_t *)&fuzzy_config_temp.theta_in_rule,            sizeof(protobuf_fuzzy_5_rule_coef_t));
  memcpy((uint8_t *)&fz_velo_rule.coef,     (uint8_t *)&fuzzy_config_temp.velo_in_rule,             sizeof(protobuf_fuzzy_3_positive_coef_rule_t));
  memcpy((uint8_t *)&fz_theta_2_input.y,    (uint8_t *)&fuzzy_config_temp.theta_out_rule,           sizeof(protobuf_fuzzy_7_rule_t));
  memcpy((uint8_t *)&fz_theta_3_input.y,    (uint8_t *)&fuzzy_config_temp.theta_out_3_input_rule,   sizeof(protobuf_fuzzy_9_rule_t));
  memcpy((uint8_t *)&fz_velo.y,             (uint8_t *)&fuzzy_config_temp.velo_out_rule,            sizeof(protobuf_fuzzy_3_positive_rule_t));
  memcpy((uint8_t *)&fz_e_phi_rule.coef,    (uint8_t *)&fuzzy_config_temp.phi_in_rule,              sizeof(protobuf_fuzzy_3_rule_coef_t));
}
else
{
  fz_ctrlr.coef.e_nor                     = FZ_CTRLR_ERROR_NOR_COEF;
  fz_ctrlr.coef.e_dot_nor                 = FZ_CTRLR_ERROR_DOT_NOR_COEF;
  fz_ctrlr.coef.theta_denor               = FZ_CTRLR_THETA_DENOR_COEF;
  fz_ctrlr.coef.velo_nor                  = FZ_CTRLR_VELOCITY_NOR_COEF;
  fz_ctrlr.coef.velo_sp_denor             = FZ_CTRLR_VELOCITY_DENOR_COEF;
  fz_ctrlr.coef.e_nor_following_state     = FZ_CTRLR_ERROR_NOR_COEF;
  fz_ctrlr.coef.e_dot_nor_following_state = FZ_CTRLR_ERROR_DOT_NOR_COEF;
  fz_ctrlr.coef.e_nor_turning_state       = FZ_CTRLR_ERROR_NOR_COEF;
  fz_ctrlr.coef.e_dot_nor_turning_state   = FZ_CTRLR_ERROR_DOT_NOR_COEF;
  fz_ctrlr.coef.e_phi_nor                 = FZ_CTRLR_PHI_NOR_COEF;

  //+=============================+=======================================+====================================+===============================+
  //                       | RAPEZIUM_RULE_INFO      | LEFT          | CENTER LEFT  | CENTER RIGHT      | RIGHT             |
  //                       |                         |               |              |                   |                   |
  //------------------------------+---------------------------------------+------------------------------------+-------------------------------+
  /** e nor */
  COEF_TRAPEZIUM_RULE_INFO(fz_e_rule.coef.NB,           -1.0f,            -1.0f,           -E_C3,               -E_C2);
  COEF_TRAPEZIUM_RULE_INFO(fz_e_rule.coef.NS,           -E_C3,            -E_C2,           -E_C2,               -E_C1);
  COEF_TRAPEZIUM_RULE_INFO(fz_e_rule.coef.ZE,           -E_C2,            -E_C1,            E_C1,                E_C2);
  COEF_TRAPEZIUM_RULE_INFO(fz_e_rule.coef.PS,            E_C1,             E_C2,            E_C2,                E_C3);
  COEF_TRAPEZIUM_RULE_INFO(fz_e_rule.coef.PB,            E_C2,             E_C3,            1.0f,                1.0f);
  /**  e dot nor */
  COEF_TRAPEZIUM_RULE_INFO(fz_e_dot_rule.coef.NB,       -1.0,             -1.0f,       -EDOT_C3,             -EDOT_C2);
  COEF_TRAPEZIUM_RULE_INFO(fz_e_dot_rule.coef.NS,       -EDOT_C3,       -EDOT_C2,       -EDOT_C2,            -EDOT_C1);
  COEF_TRAPEZIUM_RULE_INFO(fz_e_dot_rule.coef.ZE,       -EDOT_C2,       -EDOT_C1,        EDOT_C1,             EDOT_C2);
  COEF_TRAPEZIUM_RULE_INFO(fz_e_dot_rule.coef.PS,        EDOT_C1,        EDOT_C2,        EDOT_C2,             EDOT_C3);
  COEF_TRAPEZIUM_RULE_INFO(fz_e_dot_rule.coef.PB,        EDOT_C2,        EDOT_C3,           1.0f,                1.0f);
  /** phi nor */
  COEF_TRAPEZIUM_RULE_INFO(fz_e_phi_rule.coef.NE,       -1.0,             -1.0f,           -1.0,                0.0);
  COEF_TRAPEZIUM_RULE_INFO(fz_e_phi_rule.coef.ZE,       -1.0,               0.0,            0.0,                1.0);
  COEF_TRAPEZIUM_RULE_INFO(fz_e_phi_rule.coef.PO,        0.0,              1.0f,            1.0,                1.0);
  /**  theta nor */
  COEF_TRAPEZIUM_RULE_INFO(fz_theta_rule.coef.NB,       -1.0f,             -1.0f,       -THETA_C3,          -THETA_C2);
  COEF_TRAPEZIUM_RULE_INFO(fz_theta_rule.coef.NS,       -THETA_C3,     -THETA_C2,       -THETA_C2,          -THETA_C1);
  COEF_TRAPEZIUM_RULE_INFO(fz_theta_rule.coef.ZE,       -THETA_C2,     -THETA_C1,        THETA_C1,           THETA_C2);
  COEF_TRAPEZIUM_RULE_INFO(fz_theta_rule.coef.PS,        THETA_C1,      THETA_C2,        THETA_C2,           THETA_C3);
  COEF_TRAPEZIUM_RULE_INFO(fz_theta_rule.coef.PB,        THETA_C2,      THETA_C3,            1.0f,               1.0f);
  /** velo nor */
  COEF_TRAPEZIUM_RULE_INFO(fz_velo_rule.coef.PS,          0.0f,          0.0f,            VELO_C1,            VELO_C2);
  COEF_TRAPEZIUM_RULE_INFO(fz_velo_rule.coef.PM,          VELO_C1,       VELO_C2,         VELO_C2,            VELO_C3);
  COEF_TRAPEZIUM_RULE_INFO(fz_velo_rule.coef.PB,          VELO_C2,       VELO_C3,            1.0f,               1.0f);

  //+=============================+=======================================+====================================+===============================+
  //                       | Y_3_POS_RULE_INFO       | PS           | PM            | PB            |
  //                       |                         |              |               |               |
  //------------------------------+---------------------------------------+------------------------------------+-------------------------------+
  Y_3_POS_RULE_INFO(        fz_velo.y,                  Y_VELO_C1,    Y_VELO_C2,      1.0f);

  //+=============================+=======================================+====================================+===============================================+
  //                       | Y_7_RULE_INFO             | NB         | NM            | NS            | ZE        | PS            | PM         | PB        |
  //                       |                           |            |               |               |           |               |            |           |
  //------------------------------+---------------------------------------+------------------------------------+-----------------------------------------------+
  Y_7_RULE_INFO(            fz_theta_2_input.y,         -1.0f,      -Y_THETA_C2,    -Y_THETA_C1,    0.0f,       Y_THETA_C1,      Y_THETA_C2,   1.0f);
  
  //+=============================+=======================================+====================================+===============================================+
  //                       | Y_9_RULE_INFO             | NB            | NM            | NS           | NVS            | ZE     | PVS          | PS            | PM            | PB          |       
  //                       |                           |               |               |              |                |        |              |               |               |             |      
  //------------------------------+---------------------------------------+------------------------------------+---------------------------------------------------++-----------------------------------------------+
  Y_9_RULE_INFO(            fz_theta_3_input.y,         -1.0f,         -Y_THETA_4_C3,  -Y_THETA_4_C2,  -Y_THETA_4_C1,   0.0f,   Y_THETA_4_C1,   Y_THETA_4_C2,   Y_THETA_4_C3,   1.0f);
}
}


void fuzzy_controller_process_2_input(protobuf_robot_var_info_t *var, float theta_offset)
{
  duration                  = HAL_GetTick() - start_timestamp;
  start_timestamp           = HAL_GetTick();
  var->time_process = duration;

  var->e_nor = fuzzy_normalize(var->e, fz_ctrlr.coef.e_nor);
  fuzzy_check_valid(var->e_nor, FZ_VAR_E_NOR);

  var->edot = GET_DERIVATIVE(var->e, var->pre_e, duration);
  fuzzy_check_valid(var->edot, FZ_VAR_E_DOT);

  var->edot_nor = fuzzy_normalize(var->edot, fz_ctrlr.coef.e_dot_nor);
  fuzzy_check_valid(var->edot_nor, FZ_VAR_E_EDOT_NOR);

  var->theta_nor = fuzzy_controller_theta_process_2_input(var->e_nor, var->edot_nor);
  fuzzy_check_valid(var->theta_nor, FZ_VAR_THETA_NOR);

  var->theta = fuzzy_denormalize(var->theta_nor, fz_ctrlr.coef.theta_denor);
  fuzzy_check_valid(var->theta, FZ_VAR_THETA);

  var->theta += theta_offset;
  fuzzy_check_valid(var->theta, FZ_VAR_THETA);

  var->theta_nor = fuzzy_normalize(var->theta, fz_ctrlr.coef.theta_denor);
  fuzzy_check_valid(var->theta_nor, FZ_VAR_THETA_NOR);

  var->theta = fuzzy_denormalize(var->theta_nor, fz_ctrlr.coef.theta_denor);
  fuzzy_check_valid(var->theta, FZ_VAR_THETA);

  sys_dcmotor_speed_get(&var->current_speed);
  var->current_speed_nor =
  fuzzy_normalize(var->current_speed, fz_ctrlr.coef.velo_nor);
  fuzzy_check_valid(var->current_speed_nor, FZ_VAR_VELO_NOR);

  var->current_setpoint_speed_nor =
  fuzzy_controller_linear_velocity_process(var->theta_nor, var->current_speed_nor);
  fuzzy_check_valid(var->current_setpoint_speed_nor, FZ_VAR_VELO_SP_NOR);

  var->current_setpoint_speed =
  fuzzy_denormalize(var->current_setpoint_speed_nor, fz_ctrlr.coef.velo_sp_denor);
  fuzzy_check_valid(var->current_setpoint_speed, FZ_VAR_VELO_SP);


  var->pre_e_tmp = var->pre_e;
  var->pre_e     = var->e;
}


void fuzzy_controller_process_3_input(protobuf_robot_var_info_t *var, float theta_offset)
{
  duration                  = HAL_GetTick() - start_timestamp;
  start_timestamp           = HAL_GetTick();
  var->time_process = duration;

  var->e_nor = fuzzy_normalize(var->e, fz_ctrlr.coef.e_nor);
  fuzzy_check_valid(var->e_nor, FZ_VAR_E_NOR);

  var->edot = GET_DERIVATIVE(var->e, var->pre_e, duration);
  fuzzy_check_valid(var->edot, FZ_VAR_E_DOT);

  var->edot_nor = fuzzy_normalize(var->edot, fz_ctrlr.coef.e_dot_nor);
  fuzzy_check_valid(var->edot_nor, FZ_VAR_E_EDOT_NOR);

  var->e_phi_nor = fuzzy_normalize(var->e_phi, fz_ctrlr.coef.e_phi_nor);
  fuzzy_check_valid(var->e_phi_nor, FZ_VAR_E_PHI_NOR);

  var->theta_nor =
  fuzzy_controller_theta_process_3_input(var->e_nor, var->edot_nor,
                                         var->e_phi_nor);
  fuzzy_check_valid(var->theta_nor, FZ_VAR_THETA_NOR);

  var->theta = fuzzy_denormalize(var->theta_nor, fz_ctrlr.coef.theta_denor);
  fuzzy_check_valid(var->theta, FZ_VAR_THETA);

  var->theta += theta_offset;
  fuzzy_check_valid(var->theta, FZ_VAR_THETA);

  var->theta_nor = fuzzy_normalize(var->theta, fz_ctrlr.coef.theta_denor);
  fuzzy_check_valid(var->theta_nor, FZ_VAR_THETA_NOR);

  var->theta = fuzzy_denormalize(var->theta_nor, fz_ctrlr.coef.theta_denor);
  fuzzy_check_valid(var->theta, FZ_VAR_THETA);

  sys_dcmotor_speed_get(&var->current_speed);
  var->current_speed_nor =
  fuzzy_normalize(var->current_speed, fz_ctrlr.coef.velo_nor);
  fuzzy_check_valid(var->current_speed_nor, FZ_VAR_VELO_NOR);

  var->current_setpoint_speed_nor =
  fuzzy_controller_linear_velocity_process(var->theta_nor, var->current_speed_nor);
  fuzzy_check_valid(var->current_setpoint_speed_nor, FZ_VAR_VELO_SP_NOR);

  var->current_setpoint_speed =
  fuzzy_denormalize(var->current_setpoint_speed_nor, fz_ctrlr.coef.velo_sp_denor);
  fuzzy_check_valid(var->current_setpoint_speed, FZ_VAR_VELO_SP);


  var->pre_e_tmp = var->pre_e;
  var->pre_e     = var->e;
}

bool fuzzy_controller_coef_get(protobuf_fuzzy_coef_t *fz_coef)
{
  if (bsp_flash_fuzzy_read(fz_coef))
  {
    sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_PROGRAMMER,
                                   protobuf_message_type_t_MESSAGE_INFO, "Read fuzzy config from flash successfully.");
    return true;
  }
  {
    sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_PROGRAMMER,
                                   protobuf_message_type_t_MESSAGE_ERR,
                                   "Read fuzzy config from flash failed.");
    return false;
  }
  return true;
}

void fuzzy_controller_coef_set(protobuf_fuzzy_coef_t *fz_coef)
{
  fz_ctrlr.coef.e_nor                 = fz_coef->nor_coef.e_nor;
  fz_ctrlr.coef.e_dot_nor             = fz_coef->nor_coef.e_dot_nor;
  fz_ctrlr.coef.e_nor_following_state = fz_coef->nor_coef.e_nor_following_state;
  fz_ctrlr.coef.e_dot_nor_following_state = fz_coef->nor_coef.e_dot_nor_following_state;
  fz_ctrlr.coef.e_nor_turning_state = fz_coef->nor_coef.e_nor_turning_state;
  fz_ctrlr.coef.e_dot_nor_turning_state = fz_coef->nor_coef.e_dot_nor_turning_state;
  fz_ctrlr.coef.e_phi_nor     = fz_coef->nor_coef.e_phi_nor;

  memcpy((uint8_t *)&fz_e_rule.coef,        (uint8_t *)&fz_coef->e_in_rule,               sizeof(protobuf_fuzzy_5_rule_coef_t));
  memcpy((uint8_t *)&fz_e_dot_rule.coef,    (uint8_t *)&fz_coef->e_dot_in_rule,           sizeof(protobuf_fuzzy_5_rule_coef_t));
  memcpy((uint8_t *)&fz_theta_rule.coef,    (uint8_t *)&fz_coef->theta_in_rule,           sizeof(protobuf_fuzzy_5_rule_coef_t));
  memcpy((uint8_t *)&fz_velo_rule.coef,     (uint8_t *)&fz_coef->velo_in_rule,            sizeof(protobuf_fuzzy_3_positive_coef_rule_t));
  memcpy((uint8_t *)&fz_theta_2_input.y,    (uint8_t *)&fz_coef->theta_out_rule,          sizeof(protobuf_fuzzy_7_rule_t));
  memcpy((uint8_t *)&fz_theta_3_input.y,    (uint8_t *)&fz_coef->theta_out_3_input_rule,  sizeof(protobuf_fuzzy_9_rule_t));
  memcpy((uint8_t *)&fz_velo.y,             (uint8_t *)&fz_coef->velo_out_rule,           sizeof(protobuf_fuzzy_3_positive_rule_t));
  memcpy((uint8_t *)&fz_e_phi_rule.coef,    (uint8_t *)&fz_coef->phi_in_rule,             sizeof(protobuf_fuzzy_3_rule_coef_t));

  if (bsp_flash_fuzzy_write(fz_coef))
  {
    sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_PROGRAMMER,
                                   protobuf_message_type_t_MESSAGE_INFO,
                                   "Write fuzzy config to flash successfully.");
  }
  else
  {
    sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_PROGRAMMER,
                                   protobuf_message_type_t_MESSAGE_ERR,
                                   "Write fuzzy config to flash failed.");
  }
}

void fuzzy_controller_max_speed_set(float max_speed)
{
  fz_ctrlr.coef.velo_nor = max_speed;
  fz_ctrlr.coef.velo_sp_denor = max_speed;
}

float fuzzy_controller_max_speed_get(void)
{
  return fz_ctrlr.coef.velo_nor;
}

void fuzzy_controller_max_rotation_angle_set(float max_rotation_angle)
{
  fz_ctrlr.coef.theta_denor = max_rotation_angle;
}

void fuzzy_controller_clear(void)
{
  memset((uint8_t *)&fz_ctrlr.var, 0, sizeof(fz_ctrlr.var));
  sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                 protobuf_message_type_t_MESSAGE_INFO,
                                 "Fuzzy controller variables are cleared.");
}

void fuzzy_controller_set_following_coef(void)
{
  fz_ctrlr.coef.e_nor     = fz_ctrlr.coef.e_nor_following_state;
  fz_ctrlr.coef.e_dot_nor = fz_ctrlr.coef.e_dot_nor_following_state;
}

void fuzzy_controller_set_turning_coef(void)
{
  fz_ctrlr.coef.e_nor     = fz_ctrlr.coef.e_nor_turning_state;
  fz_ctrlr.coef.e_dot_nor = fz_ctrlr.coef.e_dot_nor_turning_state;
}

/* Private definitions ----------------------------------------------- */
static float fuzzy_controller_theta_process_2_input(float e_nor, float e_dot_nor)
{
  /** e dot nor */
  fz_e_dot_rule.beta[e_dot_NB_i] =
  fuzzy_trapezium(e_dot_nor, fz_e_dot_rule.coef.NB.left,
                  fz_e_dot_rule.coef.NB.center_left,
                  fz_e_dot_rule.coef.NB.center_right,
                  fz_e_dot_rule.coef.NB.right);

  fz_e_dot_rule.beta[e_dot_NS_i] =
  fuzzy_trapezium(e_dot_nor, fz_e_dot_rule.coef.NS.left,
                  fz_e_dot_rule.coef.NS.center_left,
                  fz_e_dot_rule.coef.NS.center_right,
                  fz_e_dot_rule.coef.NS.right);

  fz_e_dot_rule.beta[e_dot_ZE_i] =
  fuzzy_trapezium(e_dot_nor, fz_e_dot_rule.coef.ZE.left,
                  fz_e_dot_rule.coef.ZE.center_left,
                  fz_e_dot_rule.coef.ZE.center_right,
                  fz_e_dot_rule.coef.ZE.right);

  fz_e_dot_rule.beta[e_dot_PS_i] =
  fuzzy_trapezium(e_dot_nor, fz_e_dot_rule.coef.PS.left,
                  fz_e_dot_rule.coef.PS.center_left,
                  fz_e_dot_rule.coef.PS.center_right,
                  fz_e_dot_rule.coef.PS.right);

  fz_e_dot_rule.beta[e_dot_PB_i] =
  fuzzy_trapezium(e_dot_nor, fz_e_dot_rule.coef.PB.left,
                  fz_e_dot_rule.coef.PB.center_left,
                  fz_e_dot_rule.coef.PB.center_right,
                  fz_e_dot_rule.coef.PB.right);

  /** e nor */
  fz_e_rule.beta[e_NB_i] = fuzzy_trapezium(e_nor, fz_e_rule.coef.NB.left,
                                           fz_e_rule.coef.NB.center_left,
                                           fz_e_rule.coef.NB.center_right,
                                           fz_e_rule.coef.NB.right);

  fz_e_rule.beta[e_NS_i] = fuzzy_trapezium(e_nor, fz_e_rule.coef.NS.left,
                                           fz_e_rule.coef.NS.center_left,
                                           fz_e_rule.coef.NS.center_right,
                                           fz_e_rule.coef.NS.right);

  fz_e_rule.beta[e_ZE_i] = fuzzy_trapezium(e_nor, fz_e_rule.coef.ZE.left,
                                           fz_e_rule.coef.ZE.center_left,
                                           fz_e_rule.coef.ZE.center_right,
                                           fz_e_rule.coef.ZE.right);

  fz_e_rule.beta[e_PS_i] = fuzzy_trapezium(e_nor, fz_e_rule.coef.PS.left,
                                           fz_e_rule.coef.PS.center_left,
                                           fz_e_rule.coef.PS.center_right,
                                           fz_e_rule.coef.PS.right);

  fz_e_rule.beta[e_PB_i] = fuzzy_trapezium(e_nor, fz_e_rule.coef.PB.left,
                                           fz_e_rule.coef.PB.center_left,
                                           fz_e_rule.coef.PB.center_right,
                                           fz_e_rule.coef.PB.right);


  for (int e_dot_i = 0; e_dot_i < LANGUAGE_5_NUM; e_dot_i++)
  {
    for (int e_i = 0; e_i < LANGUAGE_5_NUM; e_i++)
    {
      fz_theta_2_input.beta[e_dot_i][e_i] =
      fuzzy_min(fz_e_dot_rule.beta[e_dot_i], fz_e_rule.beta[e_i]);
    }
  }

  float NB[] = { fz_theta_2_input.beta[e_dot_NB_i][e_NB_i], fz_theta_2_input.beta[e_dot_NB_i][e_NS_i],
                 fz_theta_2_input.beta[e_dot_NS_i][e_NB_i] };

  float NM[] = { fz_theta_2_input.beta[e_dot_NB_i][e_ZE_i], fz_theta_2_input.beta[e_dot_NS_i][e_NS_i],
                 fz_theta_2_input.beta[e_dot_ZE_i][e_NB_i] };

  float NS[] = { fz_theta_2_input.beta[e_dot_NB_i][e_PS_i], fz_theta_2_input.beta[e_dot_NS_i][e_ZE_i],
                 fz_theta_2_input.beta[e_dot_ZE_i][e_NS_i], fz_theta_2_input.beta[e_dot_PS_i][e_NB_i] };

  float ZE[] = { fz_theta_2_input.beta[e_dot_NB_i][e_PB_i], fz_theta_2_input.beta[e_dot_NS_i][e_PS_i],
                 fz_theta_2_input.beta[e_dot_ZE_i][e_ZE_i], fz_theta_2_input.beta[e_dot_PS_i][e_NS_i],
                 fz_theta_2_input.beta[e_dot_PB_i][e_NB_i] };

  float PS[] = { fz_theta_2_input.beta[e_dot_NS_i][e_PB_i], fz_theta_2_input.beta[e_dot_ZE_i][e_PS_i],
                 fz_theta_2_input.beta[e_dot_PS_i][e_ZE_i], fz_theta_2_input.beta[e_dot_PB_i][e_NS_i] };

  float PM[] = { fz_theta_2_input.beta[e_dot_ZE_i][e_PB_i], fz_theta_2_input.beta[e_dot_PS_i][e_PS_i],
                 fz_theta_2_input.beta[e_dot_PB_i][e_ZE_i] };

  float PB[] = { fz_theta_2_input.beta[e_dot_PS_i][e_PB_i], fz_theta_2_input.beta[e_dot_PB_i][e_PS_i],
                 fz_theta_2_input.beta[e_dot_PB_i][e_PB_i] };

  float theta_nor = 0;

  float NB_sum  = fuzzy_weight_sum(NB, 1, sizeof(NB) / sizeof(float));
  float PB_sum  = fuzzy_weight_sum(PB, 1, sizeof(PB) / sizeof(float));
  float NM_sum  = fuzzy_weight_sum(NM, 1, sizeof(NM) / sizeof(float));
  float PM_sum  = fuzzy_weight_sum(PM, 1, sizeof(PM) / sizeof(float));
  float NS_sum  = fuzzy_weight_sum(NS, 1, sizeof(NS) / sizeof(float));
  float PS_sum  = fuzzy_weight_sum(PS, 1, sizeof(PS) / sizeof(float));
  float ZE_sum  = fuzzy_weight_sum(ZE, 1, sizeof(ZE) / sizeof(float));
  float all_sum =
  NB_sum + PB_sum + NM_sum + PM_sum + NS_sum + PS_sum + ZE_sum;

  theta_nor = ((NB_sum * fz_theta_2_input.y.NB +
                PB_sum * fz_theta_2_input.y.PB + 
                NM_sum * fz_theta_2_input.y.NM +
                PM_sum * fz_theta_2_input.y.PM + 
                NS_sum * fz_theta_2_input.y.NS +
                PS_sum * fz_theta_2_input.y.PS + 
                ZE_sum * fz_theta_2_input.y.ZE) / all_sum);

  return (theta_nor);
}

static float fuzzy_controller_theta_process_3_input(float e_nor, float e_dot_nor, float e_phi_nor)
{
  /** e dot nor */
  fz_e_dot_rule.beta[e_dot_NB_i] =
  fuzzy_trapezium(e_dot_nor, fz_e_dot_rule.coef.NB.left,
                  fz_e_dot_rule.coef.NB.center_left,
                  fz_e_dot_rule.coef.NB.center_right,
                  fz_e_dot_rule.coef.NB.right);

  fz_e_dot_rule.beta[e_dot_NS_i] =
  fuzzy_trapezium(e_dot_nor, fz_e_dot_rule.coef.NS.left,
                  fz_e_dot_rule.coef.NS.center_left,
                  fz_e_dot_rule.coef.NS.center_right,
                  fz_e_dot_rule.coef.NS.right);

  fz_e_dot_rule.beta[e_dot_ZE_i] =
  fuzzy_trapezium(e_dot_nor, fz_e_dot_rule.coef.ZE.left,
                  fz_e_dot_rule.coef.ZE.center_left,
                  fz_e_dot_rule.coef.ZE.center_right,
                  fz_e_dot_rule.coef.ZE.right);

  fz_e_dot_rule.beta[e_dot_PS_i] =
  fuzzy_trapezium(e_dot_nor, fz_e_dot_rule.coef.PS.left,
                  fz_e_dot_rule.coef.PS.center_left,
                  fz_e_dot_rule.coef.PS.center_right,
                  fz_e_dot_rule.coef.PS.right);

  fz_e_dot_rule.beta[e_dot_PB_i] =
  fuzzy_trapezium(e_dot_nor, fz_e_dot_rule.coef.PB.left,
                  fz_e_dot_rule.coef.PB.center_left,
                  fz_e_dot_rule.coef.PB.center_right,
                  fz_e_dot_rule.coef.PB.right);

  /** e nor */
  fz_e_rule.beta[e_NB_i] = fuzzy_trapezium(e_nor, fz_e_rule.coef.NB.left,
                                           fz_e_rule.coef.NB.center_left,
                                           fz_e_rule.coef.NB.center_right,
                                           fz_e_rule.coef.NB.right);

  fz_e_rule.beta[e_NS_i] = fuzzy_trapezium(e_nor, fz_e_rule.coef.NS.left,
                                           fz_e_rule.coef.NS.center_left,
                                           fz_e_rule.coef.NS.center_right,
                                           fz_e_rule.coef.NS.right);

  fz_e_rule.beta[e_ZE_i] = fuzzy_trapezium(e_nor, fz_e_rule.coef.ZE.left,
                                           fz_e_rule.coef.ZE.center_left,
                                           fz_e_rule.coef.ZE.center_right,
                                           fz_e_rule.coef.ZE.right);

  fz_e_rule.beta[e_PS_i] = fuzzy_trapezium(e_nor, fz_e_rule.coef.PS.left,
                                           fz_e_rule.coef.PS.center_left,
                                           fz_e_rule.coef.PS.center_right,
                                           fz_e_rule.coef.PS.right);

  fz_e_rule.beta[e_PB_i] = fuzzy_trapezium(e_nor, fz_e_rule.coef.PB.left,
                                           fz_e_rule.coef.PB.center_left,
                                           fz_e_rule.coef.PB.center_right,
                                           fz_e_rule.coef.PB.right);

  /** phi nor */
  fz_e_phi_rule.beta[phi_NE_i] =  fuzzy_trapezium(e_phi_nor, fz_e_phi_rule.coef.NE.left, 
                                                  fz_e_phi_rule.coef.NE.center_left,
                                                  fz_e_phi_rule.coef.NE.center_right, 
                                                  fz_e_phi_rule.coef.NE.right);

  fz_e_phi_rule.beta[phi_ZE_i] =  fuzzy_trapezium(e_phi_nor, fz_e_phi_rule.coef.ZE.left, 
                                                  fz_e_phi_rule.coef.ZE.center_left,
                                                  fz_e_phi_rule.coef.ZE.center_right, 
                                                  fz_e_phi_rule.coef.ZE.right);

  fz_e_phi_rule.beta[phi_PO_i] =  fuzzy_trapezium(e_phi_nor, fz_e_phi_rule.coef.PO.left, 
                                                  fz_e_phi_rule.coef.PO.center_left,
                                                  fz_e_phi_rule.coef.PO.center_right, 
                                                  fz_e_phi_rule.coef.PO.right);


  for (int e_phi_i = 0; e_phi_i < LANGUAGE_3_NUM; e_phi_i++)
  {
    for (int e_dot_i = 0; e_dot_i < LANGUAGE_5_NUM; e_dot_i++)
    {
      for (int e_i = 0; e_i < LANGUAGE_5_NUM; e_i++)
      {
        fz_theta_3_input.beta[e_phi_i][e_dot_i][e_i] =
        fuzzy_min_3_input(fz_e_phi_rule.beta[e_phi_i],
                          fz_e_dot_rule.beta[e_dot_i], fz_e_rule.beta[e_i]);
      }
    }
  }

  float NB[] =  { fz_theta_3_input.beta[phi_ZE_i][e_dot_NB_i][e_NB_i],
    
                  fz_theta_3_input.beta[phi_PO_i][e_dot_NB_i][e_NB_i],
                  fz_theta_3_input.beta[phi_PO_i][e_dot_NB_i][e_NS_i],
                  fz_theta_3_input.beta[phi_PO_i][e_dot_NS_i][e_NB_i],
                };

  float PB[] =  { fz_theta_3_input.beta[phi_NE_i][e_dot_PB_i][e_PB_i],
                  fz_theta_3_input.beta[phi_NE_i][e_dot_PB_i][e_PS_i],
                  fz_theta_3_input.beta[phi_NE_i][e_dot_PS_i][e_PB_i],

                  fz_theta_3_input.beta[phi_ZE_i][e_dot_PB_i][e_PB_i],
                };

  float NM[] =  { fz_theta_3_input.beta[phi_NE_i][e_dot_NB_i][e_NB_i],  

                  fz_theta_3_input.beta[phi_ZE_i][e_dot_NB_i][e_NS_i],
                  fz_theta_3_input.beta[phi_ZE_i][e_dot_NS_i][e_NB_i],

                  fz_theta_3_input.beta[phi_PO_i][e_dot_NB_i][e_ZE_i],
                  fz_theta_3_input.beta[phi_PO_i][e_dot_NS_i][e_NS_i],
                  fz_theta_3_input.beta[phi_PO_i][e_dot_ZE_i][e_NB_i],
                };

  float PM[] =  { fz_theta_3_input.beta[phi_PO_i][e_dot_PB_i][e_PB_i],  

                  fz_theta_3_input.beta[phi_ZE_i][e_dot_PB_i][e_PS_i],
                  fz_theta_3_input.beta[phi_ZE_i][e_dot_PS_i][e_PB_i],

                  fz_theta_3_input.beta[phi_PO_i][e_dot_PB_i][e_ZE_i],
                  fz_theta_3_input.beta[phi_PO_i][e_dot_PS_i][e_PS_i],
                  fz_theta_3_input.beta[phi_PO_i][e_dot_ZE_i][e_PB_i],
                };

  float NS[] =  { fz_theta_3_input.beta[phi_NE_i][e_dot_NB_i][e_NS_i],  
                  fz_theta_3_input.beta[phi_NE_i][e_dot_NS_i][e_NB_i],

                  fz_theta_3_input.beta[phi_ZE_i][e_dot_NB_i][e_ZE_i],
                  fz_theta_3_input.beta[phi_ZE_i][e_dot_NS_i][e_NS_i],
                  fz_theta_3_input.beta[phi_ZE_i][e_dot_ZE_i][e_NB_i],

                  fz_theta_3_input.beta[phi_PO_i][e_dot_PS_i][e_NB_i],
                  fz_theta_3_input.beta[phi_PO_i][e_dot_ZE_i][e_NS_i],
                  fz_theta_3_input.beta[phi_PO_i][e_dot_NS_i][e_ZE_i],
                  fz_theta_3_input.beta[phi_PO_i][e_dot_NB_i][e_PS_i],
                };

  float PS[] =  { fz_theta_3_input.beta[phi_PO_i][e_dot_PB_i][e_PS_i],  
                  fz_theta_3_input.beta[phi_PO_i][e_dot_PS_i][e_PB_i],
                
                  fz_theta_3_input.beta[phi_ZE_i][e_dot_PB_i][e_ZE_i],
                  fz_theta_3_input.beta[phi_ZE_i][e_dot_PS_i][e_PS_i],
                  fz_theta_3_input.beta[phi_ZE_i][e_dot_ZE_i][e_PB_i],
                
                  fz_theta_3_input.beta[phi_NE_i][e_dot_NS_i][e_PB_i],
                  fz_theta_3_input.beta[phi_NE_i][e_dot_ZE_i][e_PS_i],
                  fz_theta_3_input.beta[phi_NE_i][e_dot_PS_i][e_ZE_i],
                  fz_theta_3_input.beta[phi_NE_i][e_dot_PB_i][e_NS_i],
                };

  float NVS[] = { fz_theta_3_input.beta[phi_NE_i][e_dot_NB_i][e_ZE_i],
                  fz_theta_3_input.beta[phi_NE_i][e_dot_NS_i][e_NS_i],
                  fz_theta_3_input.beta[phi_NE_i][e_dot_ZE_i][e_NB_i],

                  fz_theta_3_input.beta[phi_ZE_i][e_dot_PS_i][e_NB_i],
                  fz_theta_3_input.beta[phi_ZE_i][e_dot_ZE_i][e_NS_i],
                  fz_theta_3_input.beta[phi_ZE_i][e_dot_NS_i][e_ZE_i],
                  fz_theta_3_input.beta[phi_ZE_i][e_dot_NB_i][e_PS_i],
                  
                  fz_theta_3_input.beta[phi_PO_i][e_dot_PB_i][e_NB_i],
                  fz_theta_3_input.beta[phi_PO_i][e_dot_PS_i][e_NS_i],
                  fz_theta_3_input.beta[phi_PO_i][e_dot_ZE_i][e_ZE_i],
                  fz_theta_3_input.beta[phi_PO_i][e_dot_NS_i][e_PS_i],
                  fz_theta_3_input.beta[phi_PO_i][e_dot_NB_i][e_PB_i],
                };

  float PVS[] = { fz_theta_3_input.beta[phi_PO_i][e_dot_PB_i][e_ZE_i],
                  fz_theta_3_input.beta[phi_PO_i][e_dot_PS_i][e_PS_i],
                  fz_theta_3_input.beta[phi_PO_i][e_dot_ZE_i][e_PB_i],

                  fz_theta_3_input.beta[phi_ZE_i][e_dot_NS_i][e_PB_i],
                  fz_theta_3_input.beta[phi_ZE_i][e_dot_ZE_i][e_PS_i],
                  fz_theta_3_input.beta[phi_ZE_i][e_dot_PS_i][e_ZE_i],
                  fz_theta_3_input.beta[phi_ZE_i][e_dot_PB_i][e_NS_i],

                  fz_theta_3_input.beta[phi_NE_i][e_dot_NB_i][e_PB_i],
                  fz_theta_3_input.beta[phi_NE_i][e_dot_NS_i][e_PS_i],
                  fz_theta_3_input.beta[phi_NE_i][e_dot_ZE_i][e_ZE_i],
                  fz_theta_3_input.beta[phi_NE_i][e_dot_PS_i][e_NS_i],
                  fz_theta_3_input.beta[phi_NE_i][e_dot_PB_i][e_NB_i],
                };

  float ZE[] =  { fz_theta_3_input.beta[phi_NE_i][e_dot_PS_i][e_NB_i],
                  fz_theta_3_input.beta[phi_NE_i][e_dot_ZE_i][e_NS_i], 
                  fz_theta_3_input.beta[phi_NE_i][e_dot_NS_i][e_ZE_i], 
                  fz_theta_3_input.beta[phi_NE_i][e_dot_NB_i][e_PS_i], 

                  fz_theta_3_input.beta[phi_ZE_i][e_dot_NB_i][e_PB_i], 
                  fz_theta_3_input.beta[phi_ZE_i][e_dot_NS_i][e_PS_i], 
                  fz_theta_3_input.beta[phi_ZE_i][e_dot_ZE_i][e_ZE_i], 
                  fz_theta_3_input.beta[phi_ZE_i][e_dot_PS_i][e_NS_i], 
                  fz_theta_3_input.beta[phi_ZE_i][e_dot_PB_i][e_NB_i], 
                  
                  fz_theta_3_input.beta[phi_PO_i][e_dot_PB_i][e_NS_i], 
                  fz_theta_3_input.beta[phi_PO_i][e_dot_PS_i][e_ZE_i], 
                  fz_theta_3_input.beta[phi_PO_i][e_dot_ZE_i][e_PS_i], 
                  fz_theta_3_input.beta[phi_PO_i][e_dot_NS_i][e_PB_i], 
                };

  float theta_nor = 0;

  float NB_sum  = fuzzy_weight_sum(NB, 1, sizeof(NB) / sizeof(float));
  float PB_sum  = fuzzy_weight_sum(PB, 1, sizeof(PB) / sizeof(float));
  float NM_sum  = fuzzy_weight_sum(NM, 1, sizeof(NM) / sizeof(float));
  float PM_sum  = fuzzy_weight_sum(PM, 1, sizeof(PM) / sizeof(float));
  float NS_sum  = fuzzy_weight_sum(NS, 1, sizeof(NS) / sizeof(float));
  float PS_sum  = fuzzy_weight_sum(PS, 1, sizeof(PS) / sizeof(float));
  float NVS_sum = fuzzy_weight_sum(NVS, 1, sizeof(NVS) / sizeof(float));
  float PVS_sum = fuzzy_weight_sum(PVS, 1, sizeof(PVS) / sizeof(float));
  float ZE_sum  = fuzzy_weight_sum(ZE, 1, sizeof(ZE) / sizeof(float));
  float all_sum =
  NB_sum + PB_sum + NM_sum + PM_sum + NS_sum + PS_sum + NVS_sum + PVS_sum + ZE_sum;

  theta_nor = ((NB_sum * fz_theta_3_input.y.NB +
                PB_sum * fz_theta_3_input.y.PB + 
                NM_sum * fz_theta_3_input.y.NM +
                PM_sum * fz_theta_3_input.y.PM + 
                NS_sum * fz_theta_3_input.y.NS +
                PS_sum * fz_theta_3_input.y.PS + 
                NVS_sum * fz_theta_3_input.y.NVS +
                PVS_sum * fz_theta_3_input.y.PVS + 
                ZE_sum * fz_theta_3_input.y.ZE) / all_sum);

  return (theta_nor);
}

#if 0
static float fuzzy_controller_theta_process_4_input(float e_nor, float e_dot_nor, float phi_nor, float phi_dot_nor)
{
  /** e dot nor */
  fz_e_dot_rule.beta[e_dot_NB_i] =
  fuzzy_trapezium(e_dot_nor, fz_e_dot_rule.coef.NB.left,
                  fz_e_dot_rule.coef.NB.center_left,
                  fz_e_dot_rule.coef.NB.center_right,
                  fz_e_dot_rule.coef.NB.right);

  fz_e_dot_rule.beta[e_dot_NS_i] =
  fuzzy_trapezium(e_dot_nor, fz_e_dot_rule.coef.NS.left,
                  fz_e_dot_rule.coef.NS.center_left,
                  fz_e_dot_rule.coef.NS.center_right,
                  fz_e_dot_rule.coef.NS.right);

  fz_e_dot_rule.beta[e_dot_ZE_i] =
  fuzzy_trapezium(e_dot_nor, fz_e_dot_rule.coef.ZE.left,
                  fz_e_dot_rule.coef.ZE.center_left,
                  fz_e_dot_rule.coef.ZE.center_right,
                  fz_e_dot_rule.coef.ZE.right);

  fz_e_dot_rule.beta[e_dot_PS_i] =
  fuzzy_trapezium(e_dot_nor, fz_e_dot_rule.coef.PS.left,
                  fz_e_dot_rule.coef.PS.center_left,
                  fz_e_dot_rule.coef.PS.center_right,
                  fz_e_dot_rule.coef.PS.right);

  fz_e_dot_rule.beta[e_dot_PB_i] =
  fuzzy_trapezium(e_dot_nor, fz_e_dot_rule.coef.PB.left,
                  fz_e_dot_rule.coef.PB.center_left,
                  fz_e_dot_rule.coef.PB.center_right,
                  fz_e_dot_rule.coef.PB.right);

  /** e nor */
  fz_e_rule.beta[e_NB_i] = fuzzy_trapezium(e_nor, fz_e_rule.coef.NB.left,
                                           fz_e_rule.coef.NB.center_left,
                                           fz_e_rule.coef.NB.center_right,
                                           fz_e_rule.coef.NB.right);

  fz_e_rule.beta[e_NS_i] = fuzzy_trapezium(e_nor, fz_e_rule.coef.NS.left,
                                           fz_e_rule.coef.NS.center_left,
                                           fz_e_rule.coef.NS.center_right,
                                           fz_e_rule.coef.NS.right);

  fz_e_rule.beta[e_ZE_i] = fuzzy_trapezium(e_nor, fz_e_rule.coef.ZE.left,
                                           fz_e_rule.coef.ZE.center_left,
                                           fz_e_rule.coef.ZE.center_right,
                                           fz_e_rule.coef.ZE.right);

  fz_e_rule.beta[e_PS_i] = fuzzy_trapezium(e_nor, fz_e_rule.coef.PS.left,
                                           fz_e_rule.coef.PS.center_left,
                                           fz_e_rule.coef.PS.center_right,
                                           fz_e_rule.coef.PS.right);

  fz_e_rule.beta[e_PB_i] = fuzzy_trapezium(e_nor, fz_e_rule.coef.PB.left,
                                           fz_e_rule.coef.PB.center_left,
                                           fz_e_rule.coef.PB.center_right,
                                           fz_e_rule.coef.PB.right);

  /** phi nor */
  fz_phi_rule.beta[phi_NE_i] =  fuzzy_trapezium(phi_nor, fz_phi_rule.coef.NE.left, 
                                                fz_phi_rule.coef.NE.center_left,
                                                fz_phi_rule.coef.NE.center_right, 
                                                fz_phi_rule.coef.NE.right);

  fz_phi_rule.beta[phi_ZE_i] =  fuzzy_trapezium(phi_nor, fz_phi_rule.coef.ZE.left, 
                                                fz_phi_rule.coef.ZE.center_left,
                                                fz_phi_rule.coef.ZE.center_right, 
                                                fz_phi_rule.coef.ZE.right);

  fz_phi_rule.beta[phi_PO_i] =  fuzzy_trapezium(phi_nor, fz_phi_rule.coef.PO.left, 
                                                fz_phi_rule.coef.PO.center_left,
                                                fz_phi_rule.coef.PO.center_right, 
                                                fz_phi_rule.coef.PO.right);

  /** phi dot nor */
  fz_phi_dot_rule.beta[phi_dot_NE_i] =  fuzzy_trapezium(phi_dot_nor, fz_phi_dot_rule.coef.NE.left, 
                                                    fz_phi_dot_rule.coef.NE.center_left,
                                                    fz_phi_dot_rule.coef.NE.center_right, 
                                                    fz_phi_dot_rule.coef.NE.right);

  fz_phi_dot_rule.beta[phi_dot_ZE_i] =  fuzzy_trapezium(phi_dot_nor, fz_phi_dot_rule.coef.ZE.left, 
                                                    fz_phi_dot_rule.coef.ZE.center_left,
                                                    fz_phi_dot_rule.coef.ZE.center_right, 
                                                    fz_phi_dot_rule.coef.ZE.right);

  fz_phi_dot_rule.beta[phi_dot_PO_i] =  fuzzy_trapezium(phi_dot_nor, fz_phi_dot_rule.coef.PO.left, 
                                                    fz_phi_dot_rule.coef.PO.center_left,
                                                    fz_phi_dot_rule.coef.PO.center_right, 
                                                    fz_phi_dot_rule.coef.PO.right);

  for (int phi_dot_i = 0; phi_dot_i < LANGUAGE_3_NUM; phi_dot_i++)
  {
    for (int phi_i = 0; phi_i < LANGUAGE_3_NUM; phi_i++)
    {
      for (int e_dot_i = 0; e_dot_i < LANGUAGE_5_NUM; e_dot_i++)
      {
        for (int e_i = 0; e_i < LANGUAGE_5_NUM; e_i++)
        {
          fz_theta_4_input.beta[phi_dot_i][phi_i][e_dot_i][e_i] =
          fuzzy_min_4_input(fz_phi_rule.beta[phi_dot_i], fz_phi_dot_rule.beta[phi_i],
                            fz_e_dot_rule.beta[e_dot_i], fz_e_rule.beta[e_i]);
        }
      }
    }
  }

  float NVB[] = { fz_theta_4_input.beta[phi_dot_NE_i][phi_NE_i][e_dot_NB_i][e_NB_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_NE_i][e_dot_NB_i][e_NS_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_NE_i][e_dot_NS_i][e_NB_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_ZE_i][e_dot_NB_i][e_NB_i],
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_NE_i][e_dot_NB_i][e_NB_i],
                };

  float PVB[] = { fz_theta_4_input.beta[phi_dot_PO_i][phi_PO_i][e_dot_PB_i][e_PB_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_PO_i][e_dot_PB_i][e_PS_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_PO_i][e_dot_PS_i][e_PB_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_ZE_i][e_dot_PB_i][e_PB_i],
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_PO_i][e_dot_PB_i][e_PB_i],
                };

  float NB[] =  { fz_theta_4_input.beta[phi_dot_NE_i][phi_NE_i][e_dot_NB_i][e_ZE_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_NE_i][e_dot_NS_i][e_NS_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_NE_i][e_dot_ZE_i][e_NB_i],

                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_NE_i][e_dot_NB_i][e_NS_i],
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_NE_i][e_dot_NS_i][e_NB_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_ZE_i][e_dot_NB_i][e_NS_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_ZE_i][e_dot_NS_i][e_NB_i],
                  
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_PO_i][e_dot_NB_i][e_NB_i],
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_ZE_i][e_dot_NB_i][e_NB_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_NE_i][e_dot_NB_i][e_NB_i],
                };

  float PB[] =  { fz_theta_4_input.beta[phi_dot_PO_i][phi_PO_i][e_dot_PB_i][e_ZE_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_PO_i][e_dot_PS_i][e_PS_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_PO_i][e_dot_ZE_i][e_PB_i],

                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_PO_i][e_dot_PB_i][e_PS_i],
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_PO_i][e_dot_PS_i][e_PB_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_ZE_i][e_dot_PB_i][e_PS_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_ZE_i][e_dot_PS_i][e_PB_i],
                  
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_NE_i][e_dot_PB_i][e_PB_i],
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_ZE_i][e_dot_PB_i][e_PB_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_PO_i][e_dot_PB_i][e_PB_i],
                };

  float NM[] =  { fz_theta_4_input.beta[phi_dot_NE_i][phi_NE_i][e_dot_PS_i][e_NB_i],  // 1
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_NE_i][e_dot_ZE_i][e_NS_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_NE_i][e_dot_NS_i][e_ZE_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_NE_i][e_dot_NB_i][e_PS_i],

                  fz_theta_4_input.beta[phi_dot_NE_i][phi_ZE_i][e_dot_ZE_i][e_NB_i],  // 2
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_ZE_i][e_dot_NS_i][e_NS_i],  
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_ZE_i][e_dot_NB_i][e_ZE_i],  

                  fz_theta_4_input.beta[phi_dot_NE_i][phi_PO_i][e_dot_NS_i][e_NB_i],  // 3
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_PO_i][e_dot_NB_i][e_NS_i],  

                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_NE_i][e_dot_ZE_i][e_NB_i],  // 4
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_NE_i][e_dot_NS_i][e_NS_i],  
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_NE_i][e_dot_NB_i][e_ZE_i],

                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_ZE_i][e_dot_NS_i][e_NB_i],  // 5
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_ZE_i][e_dot_NB_i][e_NS_i], 

                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_PO_i][e_dot_NB_i][e_NB_i],  // 6

                  fz_theta_4_input.beta[phi_dot_PO_i][phi_NE_i][e_dot_NS_i][e_NB_i],  // 7
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_NE_i][e_dot_NB_i][e_NS_i], 

                  fz_theta_4_input.beta[phi_dot_PO_i][phi_ZE_i][e_dot_NB_i][e_NB_i],  // 8
                };

  float PM[] =  { fz_theta_4_input.beta[phi_dot_PO_i][phi_PO_i][e_dot_PB_i][e_NS_i],  // 9
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_PO_i][e_dot_PS_i][e_ZE_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_PO_i][e_dot_ZE_i][e_PS_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_PO_i][e_dot_NS_i][e_PB_i],

                  fz_theta_4_input.beta[phi_dot_PO_i][phi_ZE_i][e_dot_PB_i][e_ZE_i],  // 8
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_ZE_i][e_dot_PS_i][e_PS_i],  
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_ZE_i][e_dot_ZE_i][e_PB_i],  

                  fz_theta_4_input.beta[phi_dot_PO_i][phi_NE_i][e_dot_PB_i][e_PS_i],  // 7
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_NE_i][e_dot_PS_i][e_PB_i],

                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_PO_i][e_dot_PB_i][e_ZE_i],  // 6
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_PO_i][e_dot_PS_i][e_PS_i],  
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_PO_i][e_dot_ZE_i][e_PB_i], 

                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_ZE_i][e_dot_PB_i][e_PS_i],  // 5
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_ZE_i][e_dot_PS_i][e_PB_i],

                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_NE_i][e_dot_PB_i][e_PB_i],  // 4

                  fz_theta_4_input.beta[phi_dot_NE_i][phi_PO_i][e_dot_PB_i][e_PS_i],  // 3
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_PO_i][e_dot_PS_i][e_PB_i],

                  fz_theta_4_input.beta[phi_dot_NE_i][phi_ZE_i][e_dot_PB_i][e_PB_i],  // 2
                };

  float NS[] =  { fz_theta_4_input.beta[phi_dot_NE_i][phi_NE_i][e_dot_PB_i][e_NB_i],  // 1
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_NE_i][e_dot_PS_i][e_NS_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_NE_i][e_dot_ZE_i][e_ZE_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_NE_i][e_dot_NS_i][e_PS_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_NE_i][e_dot_NB_i][e_PB_i],

                  fz_theta_4_input.beta[phi_dot_NE_i][phi_ZE_i][e_dot_PS_i][e_NB_i],  // 2
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_ZE_i][e_dot_ZE_i][e_NS_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_ZE_i][e_dot_NS_i][e_ZE_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_ZE_i][e_dot_NB_i][e_PS_i],

                  fz_theta_4_input.beta[phi_dot_NE_i][phi_PO_i][e_dot_ZE_i][e_NB_i],  // 3
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_PO_i][e_dot_NS_i][e_NS_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_PO_i][e_dot_NB_i][e_ZE_i],

                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_NE_i][e_dot_PS_i][e_NB_i],  // 4
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_NE_i][e_dot_ZE_i][e_NS_i],
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_NE_i][e_dot_NS_i][e_ZE_i],
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_NE_i][e_dot_NB_i][e_PS_i],  

                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_ZE_i][e_dot_ZE_i][e_NB_i],  // 5
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_ZE_i][e_dot_NS_i][e_NS_i],  
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_ZE_i][e_dot_NB_i][e_ZE_i],  

                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_PO_i][e_dot_NS_i][e_NB_i],  // 6
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_PO_i][e_dot_NB_i][e_NS_i],  

                  fz_theta_4_input.beta[phi_dot_PO_i][phi_NE_i][e_dot_ZE_i][e_NB_i],  // 7
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_NE_i][e_dot_NS_i][e_NS_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_NE_i][e_dot_NB_i][e_ZE_i],

                  fz_theta_4_input.beta[phi_dot_PO_i][phi_ZE_i][e_dot_NS_i][e_NB_i],  // 8
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_ZE_i][e_dot_NB_i][e_NS_i], 

                  fz_theta_4_input.beta[phi_dot_PO_i][phi_PO_i][e_dot_NB_i][e_NB_i],  // 9
                };

  float PS[] =  { fz_theta_4_input.beta[phi_dot_PO_i][phi_PO_i][e_dot_PB_i][e_NB_i],  // 9
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_PO_i][e_dot_PS_i][e_NS_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_PO_i][e_dot_ZE_i][e_ZE_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_PO_i][e_dot_NS_i][e_PS_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_PO_i][e_dot_NB_i][e_PB_i],

                  fz_theta_4_input.beta[phi_dot_PO_i][phi_ZE_i][e_dot_PB_i][e_NS_i],  // 8
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_ZE_i][e_dot_PS_i][e_ZE_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_ZE_i][e_dot_ZE_i][e_PS_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_ZE_i][e_dot_NS_i][e_PB_i],

                  fz_theta_4_input.beta[phi_dot_PO_i][phi_NE_i][e_dot_PB_i][e_ZE_i],  // 7
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_NE_i][e_dot_PS_i][e_PS_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_NE_i][e_dot_ZE_i][e_PB_i],

                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_PO_i][e_dot_PB_i][e_NS_i],  // 6
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_PO_i][e_dot_PS_i][e_ZE_i],
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_PO_i][e_dot_ZE_i][e_PS_i],
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_PO_i][e_dot_NS_i][e_PB_i],

                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_ZE_i][e_dot_PB_i][e_ZE_i],  // 5
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_ZE_i][e_dot_PS_i][e_PS_i],  
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_ZE_i][e_dot_ZE_i][e_PB_i],  

                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_NE_i][e_dot_PB_i][e_PS_i],  // 4
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_NE_i][e_dot_PS_i][e_PB_i],  

                  fz_theta_4_input.beta[phi_dot_NE_i][phi_PO_i][e_dot_PB_i][e_ZE_i],  // 3
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_PO_i][e_dot_PS_i][e_PS_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_PO_i][e_dot_ZE_i][e_PB_i],

                  fz_theta_4_input.beta[phi_dot_NE_i][phi_ZE_i][e_dot_PB_i][e_PS_i],  // 2
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_ZE_i][e_dot_PS_i][e_PB_i],  

                  fz_theta_4_input.beta[phi_dot_NE_i][phi_NE_i][e_dot_PB_i][e_PB_i],  // 1
                };

  float NVS[] = { fz_theta_4_input.beta[phi_dot_NE_i][phi_NE_i][e_dot_PB_i][e_NS_i],  // 1
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_NE_i][e_dot_PS_i][e_ZE_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_NE_i][e_dot_ZE_i][e_PS_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_NE_i][e_dot_NS_i][e_PB_i],

                  fz_theta_4_input.beta[phi_dot_NE_i][phi_ZE_i][e_dot_PB_i][e_NB_i],  // 2
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_ZE_i][e_dot_PS_i][e_NS_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_ZE_i][e_dot_ZE_i][e_ZE_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_ZE_i][e_dot_NS_i][e_PS_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_ZE_i][e_dot_NB_i][e_PB_i],

                  fz_theta_4_input.beta[phi_dot_NE_i][phi_PO_i][e_dot_PS_i][e_NB_i],  // 3
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_PO_i][e_dot_ZE_i][e_NS_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_PO_i][e_dot_NS_i][e_ZE_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_PO_i][e_dot_NB_i][e_PS_i],

                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_NE_i][e_dot_PB_i][e_NB_i],  // 4
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_NE_i][e_dot_PS_i][e_NS_i],
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_NE_i][e_dot_ZE_i][e_ZE_i],
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_NE_i][e_dot_NS_i][e_PS_i],
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_NE_i][e_dot_NB_i][e_PB_i],

                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_ZE_i][e_dot_PS_i][e_NB_i],  // 5
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_ZE_i][e_dot_ZE_i][e_NS_i],
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_ZE_i][e_dot_NS_i][e_ZE_i],
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_ZE_i][e_dot_NB_i][e_PS_i],

                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_PO_i][e_dot_ZE_i][e_NB_i],  // 6
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_PO_i][e_dot_NS_i][e_NS_i],
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_PO_i][e_dot_NB_i][e_ZE_i],

                  fz_theta_4_input.beta[phi_dot_PO_i][phi_NE_i][e_dot_PS_i][e_NB_i],  // 7
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_NE_i][e_dot_ZE_i][e_NS_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_NE_i][e_dot_NS_i][e_ZE_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_NE_i][e_dot_NB_i][e_PS_i],

                  fz_theta_4_input.beta[phi_dot_PO_i][phi_ZE_i][e_dot_ZE_i][e_NB_i],  // 8
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_ZE_i][e_dot_NS_i][e_NS_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_ZE_i][e_dot_NB_i][e_ZE_i],

                  fz_theta_4_input.beta[phi_dot_PO_i][phi_PO_i][e_dot_NS_i][e_NB_i],  // 9
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_PO_i][e_dot_NB_i][e_NS_i],
                };

  float PVS[] = { fz_theta_4_input.beta[phi_dot_PO_i][phi_PO_i][e_dot_PS_i][e_NB_i],  // 9
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_PO_i][e_dot_ZE_i][e_NS_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_PO_i][e_dot_NS_i][e_ZE_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_PO_i][e_dot_NB_i][e_PS_i],

                  fz_theta_4_input.beta[phi_dot_PO_i][phi_ZE_i][e_dot_PB_i][e_NB_i],  // 8
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_ZE_i][e_dot_PS_i][e_NS_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_ZE_i][e_dot_ZE_i][e_ZE_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_ZE_i][e_dot_NS_i][e_PS_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_ZE_i][e_dot_NB_i][e_PB_i],

                  fz_theta_4_input.beta[phi_dot_PO_i][phi_NE_i][e_dot_PB_i][e_NS_i],  // 7
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_NE_i][e_dot_PS_i][e_ZE_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_NE_i][e_dot_ZE_i][e_PS_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_NE_i][e_dot_NS_i][e_PB_i],

                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_PO_i][e_dot_PB_i][e_NB_i],  // 6
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_PO_i][e_dot_PS_i][e_NS_i],
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_PO_i][e_dot_ZE_i][e_ZE_i],
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_PO_i][e_dot_NS_i][e_PS_i],
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_PO_i][e_dot_NB_i][e_PB_i],

                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_ZE_i][e_dot_PB_i][e_NS_i],  // 5
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_ZE_i][e_dot_PS_i][e_ZE_i],
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_ZE_i][e_dot_ZE_i][e_PS_i],
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_ZE_i][e_dot_NS_i][e_PB_i],

                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_NE_i][e_dot_PB_i][e_ZE_i],  // 4
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_NE_i][e_dot_PS_i][e_PS_i],
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_NE_i][e_dot_ZE_i][e_PB_i],

                  fz_theta_4_input.beta[phi_dot_NE_i][phi_PO_i][e_dot_PB_i][e_NS_i],  // 3
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_PO_i][e_dot_PS_i][e_ZE_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_PO_i][e_dot_ZE_i][e_PS_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_PO_i][e_dot_NS_i][e_PB_i],

                  fz_theta_4_input.beta[phi_dot_NE_i][phi_ZE_i][e_dot_PB_i][e_ZE_i],  // 2
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_ZE_i][e_dot_PS_i][e_PS_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_ZE_i][e_dot_ZE_i][e_PB_i],

                  fz_theta_4_input.beta[phi_dot_NE_i][phi_NE_i][e_dot_PB_i][e_PS_i],  // 1
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_NE_i][e_dot_PS_i][e_PB_i],
                };

  float ZE[] =  { fz_theta_4_input.beta[phi_dot_NE_i][phi_NE_i][e_dot_PB_i][e_ZE_i],  // 1
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_NE_i][e_dot_PS_i][e_PS_i], 
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_NE_i][e_dot_ZE_i][e_PB_i], 

                  fz_theta_4_input.beta[phi_dot_NE_i][phi_ZE_i][e_dot_PB_i][e_NS_i],  // 2
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_ZE_i][e_dot_PS_i][e_ZE_i], 
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_ZE_i][e_dot_ZE_i][e_PS_i], 
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_ZE_i][e_dot_NS_i][e_PB_i], 

                  fz_theta_4_input.beta[phi_dot_NE_i][phi_PO_i][e_dot_PB_i][e_NB_i],  // 3
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_PO_i][e_dot_PS_i][e_NS_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_PO_i][e_dot_ZE_i][e_ZE_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_PO_i][e_dot_NS_i][e_PS_i],
                  fz_theta_4_input.beta[phi_dot_NE_i][phi_PO_i][e_dot_NB_i][e_PB_i],

                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_NE_i][e_dot_PB_i][e_NS_i],  // 4
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_NE_i][e_dot_PS_i][e_ZE_i], 
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_NE_i][e_dot_ZE_i][e_PS_i], 
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_NE_i][e_dot_NS_i][e_PB_i], 

                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_ZE_i][e_dot_PB_i][e_NB_i],  // 5
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_ZE_i][e_dot_PS_i][e_NS_i],
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_ZE_i][e_dot_ZE_i][e_ZE_i],
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_ZE_i][e_dot_NS_i][e_PS_i],
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_ZE_i][e_dot_NB_i][e_PB_i],

                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_PO_i][e_dot_PS_i][e_NB_i],  // 6
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_PO_i][e_dot_ZE_i][e_NS_i], 
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_PO_i][e_dot_NS_i][e_ZE_i], 
                  fz_theta_4_input.beta[phi_dot_ZE_i][phi_PO_i][e_dot_NB_i][e_PS_i],
                  
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_NE_i][e_dot_PB_i][e_NB_i],  // 7
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_NE_i][e_dot_PS_i][e_NS_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_NE_i][e_dot_ZE_i][e_ZE_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_NE_i][e_dot_NS_i][e_PS_i],
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_NE_i][e_dot_NB_i][e_PB_i],

                  fz_theta_4_input.beta[phi_dot_PO_i][phi_ZE_i][e_dot_PS_i][e_NB_i],  // 8
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_ZE_i][e_dot_ZE_i][e_NS_i], 
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_ZE_i][e_dot_NS_i][e_ZE_i], 
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_ZE_i][e_dot_NB_i][e_PS_i],

                  fz_theta_4_input.beta[phi_dot_PO_i][phi_PO_i][e_dot_ZE_i][e_NB_i],  // 9
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_PO_i][e_dot_NS_i][e_NS_i], 
                  fz_theta_4_input.beta[phi_dot_PO_i][phi_PO_i][e_dot_NB_i][e_ZE_i],
                };

  float theta_nor = 0;

  // float NVB_sum = fuzzy_weight_sum(NVB, fz_theta_4_input.y.NVB, sizeof(NVB) / sizeof(float));
  float NVB_sum = fuzzy_weight_sum(NVB, 1, sizeof(NVB) / sizeof(float));
  float PVB_sum = fuzzy_weight_sum(PVB, 1, sizeof(PVB) / sizeof(float));
  float NB_sum  = fuzzy_weight_sum(NB, 1, sizeof(NB) / sizeof(float));
  float PB_sum  = fuzzy_weight_sum(PB, 1, sizeof(PB) / sizeof(float));
  float NM_sum  = fuzzy_weight_sum(NM, 1, sizeof(NM) / sizeof(float));
  float PM_sum  = fuzzy_weight_sum(PM, 1, sizeof(PM) / sizeof(float));
  float NS_sum  = fuzzy_weight_sum(NS, 1, sizeof(NS) / sizeof(float));
  float PS_sum  = fuzzy_weight_sum(PS, 1, sizeof(PS) / sizeof(float));
  float NVS_sum = fuzzy_weight_sum(NVS, 1, sizeof(NVS) / sizeof(float));
  float PVS_sum = fuzzy_weight_sum(PVS, 1, sizeof(PVS) / sizeof(float));
  float ZE_sum  = fuzzy_weight_sum(ZE, 1, sizeof(ZE) / sizeof(float));
  float all_sum = NVB_sum + PVB_sum + NB_sum + PB_sum + NM_sum + PM_sum +
  NS_sum + PS_sum + NVS_sum + PVS_sum + ZE_sum;

  theta_nor = ((NVB_sum * fz_theta_4_input.y.NVB +
                PVB_sum * fz_theta_4_input.y.PVB + 
                NB_sum * fz_theta_4_input.y.NB +
                PB_sum * fz_theta_4_input.y.PB + 
                NM_sum * fz_theta_4_input.y.NM +
                PM_sum * fz_theta_4_input.y.PM + 
                NS_sum * fz_theta_4_input.y.NS +
                PS_sum * fz_theta_4_input.y.PS + 
                NVS_sum * fz_theta_4_input.y.NVS +
                PVS_sum * fz_theta_4_input.y.PVS + 
                ZE_sum * fz_theta_4_input.y.ZE) / all_sum);


#if 0
  theta_nor = (fuzzy_weight_sum(NB, fz_theta.y.NB, sizeof(NB) / sizeof(float)) +
               fuzzy_weight_sum(NM, fz_theta.y.NM, sizeof(NM) / sizeof(float)) +
               fuzzy_weight_sum(NS, fz_theta.y.NS, sizeof(NS) / sizeof(float)) +
               fuzzy_weight_sum(ZE, fz_theta.y.ZE, sizeof(ZE) / sizeof(float)) +
               fuzzy_weight_sum(PS, fz_theta.y.PS, sizeof(PS) / sizeof(float)) +
               fuzzy_weight_sum(PM, fz_theta.y.PM, sizeof(PM) / sizeof(float)) +
               fuzzy_weight_sum(PB, fz_theta.y.PB, sizeof(PB) / sizeof(float))) /
  fuzzy_sum_2d(fz_theta.beta[0], LANGUAGE_5_NUM, LANGUAGE_5_NUM);
#endif

  return (theta_nor);
}
#endif


static float fuzzy_controller_linear_velocity_process(float theta_nor, float current_velocity_nor)
{
  /** velo nor */
  fz_velo_rule.beta[v_PS_po_i] =
  fuzzy_trapezium(current_velocity_nor, fz_velo_rule.coef.PS.left,
                  fz_velo_rule.coef.PS.center_left,
                  fz_velo_rule.coef.PS.center_right,
                  fz_velo_rule.coef.PS.right);

  fz_velo_rule.beta[v_PM_po_i] =
  fuzzy_trapezium(current_velocity_nor, fz_velo_rule.coef.PM.left,
                  fz_velo_rule.coef.PM.center_left,
                  fz_velo_rule.coef.PM.center_right,
                  fz_velo_rule.coef.PM.right);

  fz_velo_rule.beta[v_PB_po_i] =
  fuzzy_trapezium(current_velocity_nor, fz_velo_rule.coef.PB.left,
                  fz_velo_rule.coef.PB.center_left,
                  fz_velo_rule.coef.PB.center_right,
                  fz_velo_rule.coef.PB.right);

  /** theta nor */
  fz_theta_rule.beta[theta_NB_i] =
  fuzzy_trapezium(theta_nor, fz_theta_rule.coef.NB.left,
                  fz_theta_rule.coef.NB.center_left,
                  fz_theta_rule.coef.NB.center_right,
                  fz_theta_rule.coef.NB.right);

  fz_theta_rule.beta[theta_NS_i] =
  fuzzy_trapezium(theta_nor, fz_theta_rule.coef.NS.left,
                  fz_theta_rule.coef.NS.center_left,
                  fz_theta_rule.coef.NS.center_right,
                  fz_theta_rule.coef.NS.right);

  fz_theta_rule.beta[theta_ZE_i] =
  fuzzy_trapezium(theta_nor, fz_theta_rule.coef.ZE.left,
                  fz_theta_rule.coef.ZE.center_left,
                  fz_theta_rule.coef.ZE.center_right,
                  fz_theta_rule.coef.ZE.right);

  fz_theta_rule.beta[theta_PS_i] =
  fuzzy_trapezium(theta_nor, fz_theta_rule.coef.PS.left,
                  fz_theta_rule.coef.PS.center_left,
                  fz_theta_rule.coef.PS.center_right,
                  fz_theta_rule.coef.PS.right);

  fz_theta_rule.beta[theta_PB_i] =
  fuzzy_trapezium(theta_nor, fz_theta_rule.coef.PB.left,
                  fz_theta_rule.coef.PB.center_left,
                  fz_theta_rule.coef.PB.center_right,
                  fz_theta_rule.coef.PB.right);

  for (int v_i = 0; v_i < PO_LANGUAGE_NUM; v_i++)
  {
    for (int theta_i = 0; theta_i < LANGUAGE_5_NUM; theta_i++)
    {
      fz_velo.beta[v_i][theta_i] =
      fuzzy_min(fz_velo_rule.beta[v_i], fz_theta_rule.beta[theta_i]);
    }
  }

  float PS[] = {
    fz_velo.beta[v_PS_po_i][theta_NB_i], fz_velo.beta[v_PS_po_i][theta_NS_i],
    fz_velo.beta[v_PS_po_i][theta_PS_i], fz_velo.beta[v_PS_po_i][theta_PB_i],
    fz_velo.beta[v_PM_po_i][theta_NB_i], fz_velo.beta[v_PM_po_i][theta_PB_i],
    fz_velo.beta[v_PB_po_i][theta_NB_i], fz_velo.beta[v_PB_po_i][theta_PB_i]
  };

  float PM[] = { fz_velo.beta[v_PS_po_i][theta_ZE_i], fz_velo.beta[v_PM_po_i][theta_NS_i],
                 fz_velo.beta[v_PM_po_i][theta_PS_i], fz_velo.beta[v_PB_po_i][theta_NS_i],
                 fz_velo.beta[v_PB_po_i][theta_PS_i] };

  float PB[] = { fz_velo.beta[v_PM_po_i][theta_ZE_i], fz_velo.beta[v_PB_po_i][theta_ZE_i] };

  float float_velo_nor = 0;
  float_velo_nor = (fuzzy_weight_sum(PS, fz_velo.y.PS, sizeof(PS) / sizeof(float)) +
                    fuzzy_weight_sum(PM, fz_velo.y.PM, sizeof(PM) / sizeof(float)) +
                    fuzzy_weight_sum(PB, fz_velo.y.PB, sizeof(PB) / sizeof(float))) /
  fuzzy_sum_2d(fz_velo.beta[0], PO_LANGUAGE_NUM, LANGUAGE_5_NUM);
  return float_velo_nor;
}

static void fuzzy_check_valid(float value, fuzzy_var_type_t var_type)
{
  if (!(isnan(value) || isinf(value)))
    return;

  switch (var_type)
  {
  case FZ_VAR_E:
  {
    sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                   protobuf_message_type_t_MESSAGE_ERR,
                                   "Invalid FZ_VAR_E. NaN or Inf.");
    break;
  }
  case FZ_VAR_E_NOR:
  {
    sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                   protobuf_message_type_t_MESSAGE_ERR,
                                   "Invalid FZ_VAR_E_NOR. NaN or Inf.");
    break;
  }
  case FZ_VAR_E_DOT:
  {
    sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                   protobuf_message_type_t_MESSAGE_ERR,
                                   "Invalid FZ_VAR_E_DOT. NaN or Inf.");
    break;
  }
  case FZ_VAR_E_EDOT_NOR:
  {
    sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                   protobuf_message_type_t_MESSAGE_ERR,
                                   "Invalid FZ_VAR_E_EDOT_NOR. NaN or Inf.");
    break;
  }
  case FZ_VAR_THETA_NOR:
  {
    sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                   protobuf_message_type_t_MESSAGE_ERR,
                                   "Invalid FZ_VAR_THETA_NOR. NaN or Inf.");
    break;
  }
  case FZ_VAR_THETA:
  {
    sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                   protobuf_message_type_t_MESSAGE_ERR,
                                   "Invalid FZ_VAR_THETA. NaN or Inf.");
    break;
  }
  case FZ_VAR_VELO_NOR:
  {
    sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                   protobuf_message_type_t_MESSAGE_ERR,
                                   "Invalid FZ_VAR_VELO_NOR. NaN or Inf.");
    break;
  }
  case FZ_VAR_VELO_SP_NOR:
  {
    sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                   protobuf_message_type_t_MESSAGE_ERR,
                                   "Invalid FZ_VAR_VELO_SP_NOR. NaN or Inf.");
    break;
  }
  case FZ_VAR_VELO_SP:
  {
    sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                   protobuf_message_type_t_MESSAGE_ERR,
                                   "Invalid FZ_VAR_VELO_SP. NaN or Inf.");
    break;
  }
  case FZ_VAR_E_PHI_NOR:
  {
    sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                   protobuf_message_type_t_MESSAGE_ERR,
                                   "Invalid FZ_VAR_E_PHI_NOR. NaN or Inf.");
    break;
  }
  default:
  {
    break;
  }
  }
  return;
}

/* End of file -------------------------------------------------------- */
