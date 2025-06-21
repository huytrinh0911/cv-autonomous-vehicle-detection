/**
 * @file       robot_control.h
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

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __ROBOT_CONTROL_H
#define __ROBOT_CONTROL_H

/* Includes ----------------------------------------------------------- */
#include "network_msg.h"
#include "sys_dcmotor.h"
#include "common.h"

// ----------------
#include "protocol.pb.h"

/* Public defines ----------------------------------------------------- */
#define ROBOT_CONTROL_CBUFFER_SIZE (512)

#define ROBOT_COORDINATE_CENTER_SETPOINT           (668.0f)
#define ROBOT_COORDINATE_TURN_LEFT_LEFT_SETPOINT   (250.0f)  // x_left_sp
#define ROBOT_COORDINATE_TURN_RIGHT_RIGHT_SETPOINT (1279.0f - 250.0f) // x_right_sp

#define ROBOT_SPEED_MOVE_FORWARD_PERCENT (0.75f)

#define ROBOT_LANE_ANGLE_SETPOINT (4.6f)

#define ROBOT_SINGULARITY_ERROR_GAP  (100)
#define ROBOT_SINGULARITY_CENTER_GAP (100)
#define ROBOT_SINGULARITY_LEFT_GAP   (100)
#define ROBOT_SINGULARITY_RIGHT_GAP  (100)
#define ROBOT_SINGULARITY_PHI_GAP    (15)

#define ROBOT_ERROR_AVG_NUM          (10)
#define ROBOT_SINGULARITY_CNT        (5)

#define ROBOT_CAMERA_MODE_P_0_OFFSET  (0.0f)
#define ROBOT_CAMERA_MODE_P_30_OFFSET (25.0f)
#define ROBOT_CAMERA_MODE_P_45_OFFSET (35.0f)
#define ROBOT_CAMERA_MODE_N_30_OFFSET (-25.0f)
#define ROBOT_CAMERA_MODE_N_45_OFFSET (-35.0f)

#define ROBOT_CAMERA_0_TO_P_30_CONDITION    (-20.0f)
#define ROBOT_CAMERA_P_30_TO_P_45_CONDITION (-20.0f)
#define ROBOT_CAMERA_P_45_TO_P_30_CONDITION (15.0f)
#define ROBOT_CAMERA_P_30_TO_0_CONDITION    (10.0)

#define ROBOT_CAMERA_0_TO_N_30_CONDITION    (20.0f)
#define ROBOT_CAMERA_N_30_TO_N_45_CONDITION (20.0f)
#define ROBOT_CAMERA_N_45_TO_N_30_CONDITION (-15.0)
#define ROBOT_CAMERA_N_30_TO_0_CONDITION    (-10.0f)

// #define ROBOT_CAMERA_0_TO_P_30_CONDITION    (-17.5f)
// #define ROBOT_CAMERA_P_30_TO_P_45_CONDITION (-20.0f)
// #define ROBOT_CAMERA_P_45_TO_P_30_CONDITION (15.0f)
// #define ROBOT_CAMERA_P_30_TO_0_CONDITION    (12.0f)

// #define ROBOT_CAMERA_0_TO_N_30_CONDITION    (17.5f)
// #define ROBOT_CAMERA_N_30_TO_N_45_CONDITION (20.0f)
// #define ROBOT_CAMERA_N_45_TO_N_30_CONDITION (-15.0)
// #define ROBOT_CAMERA_N_30_TO_0_CONDITION    (-10.0f)

#define ROBOT_CAMERA_SERVO_ANGLE_0    (0.0f)
#define ROBOT_CAMERA_SERVO_ANGLE_P_30 (-30.0f)
#define ROBOT_CAMERA_SERVO_ANGLE_P_45 (-45.0f)
#define ROBOT_CAMERA_SERVO_ANGLE_N_30 (30.0f)
#define ROBOT_CAMERA_SERVO_ANGLE_N_45 (45.0f)

#define ROBOT_CAMERA_SKIP_COUNTER     (30)
#define ROBOT_TURN_SKIP_COUNTER       (10)
#define ROBOT_TURN_COUNTINOUS_COUNTER (10)

#define ROBOT_MAX_SPEED          (20.0f)
#define ROBOT_MAX_ROTATION_ANGLE (55.0f)

#define ROBOT_RETURN_TO_FOLLOWING_LEFT_POSITION_CONDITION (100.0f)
#define ROBOT_RETURN_TO_FOLLOWING_RIGHT_POSITION_CONDITION \
  (1279.0f - ROBOT_RETURN_TO_FOLLOWING_LEFT_POSITION_CONDITION)
#define ROBOT_RETURN_TO_FOLLOWING_PHI_ERROR_CONDITION (20.0f)

/* Public enumerate/structure ----------------------------------------- */
typedef struct
{
  bool     first_value;
  float    value[ROBOT_ERROR_AVG_NUM];
  float    outlier[ROBOT_SINGULARITY_CNT];
  uint16_t count_s;
} robot_process_input_data_var_t;

typedef struct
{
  robot_process_input_data_var_t center;
  robot_process_input_data_var_t left_down;
  robot_process_input_data_var_t right_down;
  robot_process_input_data_var_t phi_error;
  robot_process_input_data_var_t coor_error;

} robot_process_input_data_t;

typedef enum 
{
  ROBOT_CAMERA_STATE_0,
  ROBOT_CAMERA_STATE_P_30,
  ROBOT_CAMERA_STATE_P_45,
  ROBOT_CAMERA_STATE_N_30,
  ROBOT_CAMERA_STATE_N_45,
  ROBOT_CAMERA_STATE_MAX
} robot_camera_state_t;


/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */
void robot_control_init(void);
void robot_control_process(void);
void robot_control_signs_process(void);
void robot_control_add_coor(protobuf_vision_coordinate_detected_t *coor);
void robot_control_add_signs(protobuf_signs_info_t *signs_info);
void robot_control_start(void);
void robot_control_stop(void);
void robot_control_max_speed_set(float max_speed);
void robot_control_max_rotation_angle_set(float max_rotation_angle);
void robot_control_info_get(void);
void robot_add_direction_queue(protobuf_direction_cmd_t direction_cmd);
void robot_update_direction_queue(void);

#endif // __ROBOT_CONTROL_H

/* End of file -------------------------------------------------------- */
