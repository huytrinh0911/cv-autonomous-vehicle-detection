/**
 * @file        fuzzy.h
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
#ifndef __FUZZY_H
#define __FUZZY_H

/* Includes ----------------------------------------------------------- */
#include "common.h"

/* Public defines ----------------------------------------------------- */
#define phi_dot_NE_i (0)
#define phi_dot_ZE_i (1)
#define phi_dot_PO_i (2)

#define phi_NE_i (0)
#define phi_ZE_i (1)
#define phi_PO_i (2)

#define e_dot_NB_i (0)
#define e_dot_NS_i (1)
#define e_dot_ZE_i (2)
#define e_dot_PS_i (3)
#define e_dot_PB_i (4)

#define e_NB_i (0)
#define e_NS_i (1)
#define e_ZE_i (2)
#define e_PS_i (3)
#define e_PB_i (4)

#define LANGUAGE_5_NUM (5)
#define LANGUAGE_3_NUM (3)

#define theta_NB_i (0)
#define theta_NS_i (1)
#define theta_ZE_i (2)
#define theta_PS_i (3)
#define theta_PB_i (4)

#define v_PS_po_i (0)
#define v_PM_po_i (1)
#define v_PB_po_i (2)

#define PO_LANGUAGE_NUM (3)

/* Public enumerate/structure ----------------------------------------- */
typedef struct
{
  float NE;
  float ZE;
  float PO;
} fuzzy_3_rule_t;

typedef struct
{
  float PS;
  float PM;
  float PB;
} fuzzy_3_positive_rule_t;

typedef struct
{
  float NB;
  float NS;
  float ZE;
  float PS;
  float PB;
} fuzzy_5_rule_t;

typedef struct
{
  float NB;
  float NM;
  float NS;
  float ZE;
  float PS;
  float PM;
  float PB;
} fuzzy_7_rule_t;

typedef struct 
{
  float left;
  float center_left;
  float center_right;
  float right;
} fuzzy_trapezium_rule_t;

typedef struct
{
  float left;
  float cente;
  float right;
} fuzzy_triangle_rule_t;

typedef struct 
{
  fuzzy_trapezium_rule_t PS;
  fuzzy_trapezium_rule_t PM;
  fuzzy_trapezium_rule_t PB;
} fuzzy_3_positive_coef_rule_t;

typedef struct 
{
  fuzzy_trapezium_rule_t NB;
  fuzzy_trapezium_rule_t NS;
  fuzzy_trapezium_rule_t ZE;
  fuzzy_trapezium_rule_t PS;
  fuzzy_trapezium_rule_t PB;
} fuzzy_5_rule_coef_t;

typedef struct
{
  fuzzy_5_rule_coef_t NB;
  fuzzy_5_rule_coef_t NM;
  fuzzy_5_rule_coef_t NS;
  fuzzy_5_rule_coef_t ZE;
  fuzzy_5_rule_coef_t PS;
  fuzzy_5_rule_coef_t PM;
  fuzzy_5_rule_coef_t PB;
} fuzzy_7_rule_coef_t;

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */
float fuzzy_trapezium(float data, float left, float center_left, float center_right, float right);
float fuzzy_triangle(float data, float left, float center, float right);
float fuzzy_saturation(float data);
float fuzzy_min(float a, float b);
float fuzzy_min_3_input(float a, float b, float c);
float fuzzy_min_4_input(float a, float b, float c, float d);
float fuzzy_normalize(float data, float coefficient);
float fuzzy_denormalize(float data, float coefficient);
float fuzzy_sum_2d(float *data, uint16_t row_len, uint16_t column);
float fuzzy_weight_sum(float *data, float y, uint16_t len);

#endif // __FUZZY_H

/* End of file -------------------------------------------------------- */
