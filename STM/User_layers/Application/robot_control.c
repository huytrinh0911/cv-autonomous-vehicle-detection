/**
 * @file       robot_control.c
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
#include "robot_control.h"
#include "fuzzy_controller.h"
#include "sys_network.h"
#include "sys_network_cmd.h"
#include "sys_servo.h"
#include "bsp_utils.h"

// ----------------
#include "protocol.pb.h"
#include "vl_cbuffer.h"

/* Private defines ---------------------------------------------------- */
#define ROBOT_VL_CBUFFER_SIZE (1024)
#define ROBOT_SKIP_COUNTER    (60)
#define HAVE_SKIP_COUNTER
#define DIRECTION_QUEUE_SIZE (10)

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  uint8_t queue_index;
  protobuf_direction_cmd_t next_direction_cmd;
  protobuf_direction_cmd_t direction_queue[DIRECTION_QUEUE_SIZE];
  protobuf_direction_cmd_t direction_cmd_after_stop_temp;
} direction_queue_info_t;

typedef struct
{
  bool                       is_first_time_enter_state;
  uint16_t                   robot_start_skip_cnt;
  uint16_t                   robot_rotate_camera_skip_cnt;
  float                      theta_offset;
  robot_process_input_data_t input_data;
  protobuf_robot_info_rsp_t  info;

  bool first_not_see_right_lane;
  bool first_see_right_lane_again;

  bool first_not_see_left_lane;
  bool first_see_left_lane_again;

  vl_cbuffer_t coor_cb;
  uint8_t      coor_buff[ROBOT_VL_CBUFFER_SIZE];

  vl_cbuffer_t signs_cb;
  uint8_t      signs_buff[(ROBOT_VL_CBUFFER_SIZE / 2)];

  protobuf_vision_coordinate_detected_t pre_coor_recv;

  protobuf_signs_t signs_detected;
  protobuf_signs_t signs_detect_obey;
  protobuf_signs_t pre_signs_detect_obey;

  direction_queue_info_t direction_queue_data;

  uint32_t stop_timestamp;
  uint32_t stop_dur;
  uint32_t stop_len_s;
  bool     is_stop_temp;

  float max_speed_temp;

  bool is_ready_to_return_following_state;
  uint8_t turn_skip_cnt;
  uint8_t turn_continous_cnt;

  bool stopping_redlight;
} robot_t;

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */
static robot_t robot;

/* Private function prototypes ---------------------------------------- */
static void robot_control_clear(void);

static void robot_process_input_lane_center(protobuf_vision_coordinate_detected_t *coor,
                                            float *center);
static void robot_process_input_coor_error(protobuf_vision_coordinate_detected_t *coor,
                                           float *coor_error,
                                           protobuf_robot_direct_state_t direct_state);
static void robot_process_input_lane_left(protobuf_vision_coordinate_detected_t *coor,
                                          float *left_down_coor);
static void robot_process_input_lane_right(protobuf_vision_coordinate_detected_t *coor,
                                           float *right_down_coor);
static void robot_process_input_lane_angle(protobuf_vision_coordinate_detected_t *coor,
                                           float *phi_error);

static float robot_cal_coor_error(protobuf_vision_coordinate_detected_t *coor,
                                  protobuf_robot_direct_state_t direct_state);

static float robot_cal_lane_angle_error(protobuf_vision_coordinate_detected_t *coor);
static float calculate_average(float *array, uint16_t length);

static void robot_camera_rotate_fsm(float angle_error);
static void robot_control_update(protobuf_robot_info_rsp_t *fuzzy_var);
static void robot_control_update_stop_temp(protobuf_robot_info_rsp_t *fuzzy_var);
static void robot_camera_state_set(robot_camera_state_t state);
static void robot_direct_fsm_stop(protobuf_vision_coordinate_detected_t *coor);
static void robot_direct_fsm_follow_lane(protobuf_vision_coordinate_detected_t *coor);
static void robot_direct_fsm_move_forward(protobuf_vision_coordinate_detected_t *coor);
static void robot_direct_fsm_turn_right(protobuf_vision_coordinate_detected_t *coor);
static void robot_direct_fsm_turn_left(protobuf_vision_coordinate_detected_t *coor);
static void robot_control_cmd_process_crossroads(bool is_crossroads);
static void robot_reset_direction_queue(void);
static void robot_get_next_direction_cmd(protobuf_direction_cmd_t *direct_cmd);
static void robot_control_set_direction(protobuf_direction_cmd_t direct_cmd);
static void robot_control_set_pre_speed(void);

/* Function definitions ----------------------------------------------- */
void robot_control_init(void)
{
  robot.info.has_state_info = true;
  robot.info.has_var_info   = true;

  robot.info.state_info.process_state = protobuf_robot_process_state_t_ROBOT_STATE_IDLE;
  robot.info.state_info.direct_state = protobuf_robot_direct_state_t_ROBOT_STATE_STOP;
  robot.info.state_info.pre_direct_state = protobuf_robot_direct_state_t_ROBOT_STATE_STOP;

  robot.info.var_info.theta                  = 0;
  robot.info.var_info.current_setpoint_speed = 0;
  robot.info.var_info.max_theta              = FZ_CTRLR_THETA_DENOR_COEF;
  robot.info.var_info.max_setpoint_speed     = FZ_CTRLR_VELOCITY_NOR_COEF;

  vl_cbuffer_init(&robot.coor_cb, robot.coor_buff, ROBOT_VL_CBUFFER_SIZE);
  vl_cbuffer_init(&robot.signs_cb, robot.signs_buff, (ROBOT_VL_CBUFFER_SIZE / 2));

  robot.info.state_info.camera_state = ROBOT_CAMERA_STATE_0;

  robot.robot_start_skip_cnt         = ROBOT_SKIP_COUNTER;
  robot.robot_rotate_camera_skip_cnt = ROBOT_CAMERA_SKIP_COUNTER;

  robot.input_data.center.first_value     = true;
  robot.input_data.coor_error.first_value = true;
  robot.input_data.left_down.first_value  = true;
  robot.input_data.right_down.first_value = true;
  robot.input_data.phi_error.first_value  = true;

  robot.theta_offset = 0;

  robot.is_stop_temp   = false;
  robot.stop_dur       = 0;
  robot.stop_len_s     = 0;
  robot.stop_timestamp = 0;

  robot_reset_direction_queue();
}

void robot_control_process(void)
{
#ifdef HAVE_SKIP_COUNTER
  if (robot.robot_start_skip_cnt)
  {
    return;
  }
#endif

  switch (robot.info.state_info.process_state)
  {
  case protobuf_robot_process_state_t_ROBOT_STATE_IDLE:
  {
    if (robot.info.state_info.direct_state == protobuf_robot_direct_state_t_ROBOT_STATE_STOP_TEMP)
    {
      if (robot.stopping_redlight)
      {
        return;
      }

      if (!robot.is_stop_temp)
      {
        return;
      }

      robot.stop_dur = HAL_GetTick() - robot.stop_timestamp;
      if(robot.stop_dur >= robot.stop_len_s*1000)
      {
        robot.is_stop_temp = false;
        robot.info.state_info.process_state = protobuf_robot_process_state_t_ROBOT_STATE_RUNNING;
        robot_control_set_direction(robot.direction_queue_data.direction_cmd_after_stop_temp);
      }
    }
    break;
  }
  case protobuf_robot_process_state_t_ROBOT_STATE_RUNNING:
  {
    protobuf_vision_coordinate_detected_t coor;
    uint32_t data_cnt = vl_cbuffer_count(&robot.coor_cb);
    if (data_cnt)
    {
      uint32_t data_read_cnt =
      vl_cbuffer_read(&robot.coor_cb, (uint8_t *)&coor,
                      sizeof(protobuf_vision_coordinate_detected_t));
      if (data_read_cnt != sizeof(protobuf_vision_coordinate_detected_t))
      {
        return;
      }
      robot.info.var_info.center          = coor.center_coor;
      robot.info.var_info.right_down_coor = coor.right_down_coor;
      robot.info.var_info.left_down_coor  = coor.left_down_coor;
      robot.info.var_info.right_up_coor   = coor.right_up_coor;
      robot.info.var_info.left_up_coor    = coor.left_up_coor;
      robot.info.var_info.phi             = coor.phi;
    }
    else
    {
      return;
    }

    switch (robot.info.state_info.direct_state)
    {
    case protobuf_robot_direct_state_t_ROBOT_STATE_STOP:
    {
      robot_direct_fsm_stop(&coor);
      break;
    }
    case protobuf_robot_direct_state_t_ROBOT_STATE_MOVE_FORWARD:
    {
      robot_direct_fsm_move_forward(&coor);
      break;
    }
    case protobuf_robot_direct_state_t_ROBOT_STATE_FOLLOW_LANE:
    {
      robot_direct_fsm_follow_lane(&coor);
      break;
    }
    case protobuf_robot_direct_state_t_ROBOT_STATE_TURN_LEFT:
    {
      robot_direct_fsm_turn_left(&coor);
      break;
    }
    case protobuf_robot_direct_state_t_ROBOT_STATE_TURN_RIGHT:
    {
      robot_direct_fsm_turn_right(&coor);
      break;
    }
    case protobuf_robot_direct_state_t_ROBOT_STATE_MOVE_BACKWARD:
    {
      break;
    }
    default:
    {
      break;
    }
    }

    memcpy((uint8_t *)&robot.pre_coor_recv, (uint8_t *)&coor, sizeof(protobuf_vision_coordinate_detected_t));
    break;
  }
  default:
  {
    break;
  }
  }
}

void robot_control_signs_process(void)
{
#ifdef HAVE_SKIP_COUNTER
  if (robot.robot_start_skip_cnt)
  {
    return;
  }
#endif

  protobuf_signs_info_t signs_info;
  uint32_t              data_cnt = vl_cbuffer_count(&robot.signs_cb);
  if (data_cnt)
  {
    uint32_t data_read_cnt = vl_cbuffer_read(&robot.signs_cb, (uint8_t *)&signs_info,
                                             sizeof(protobuf_signs_info_t));
    if (data_read_cnt != sizeof(protobuf_signs_info_t))
    {
      return;
    }
  }
  else
  {
    return;
  }

  memcpy((uint8_t *)&robot.signs_detect_obey, (uint8_t *)&signs_info.signs_obey,
         sizeof(protobuf_signs_t));
  memcpy((uint8_t *)&robot.signs_detected,
         (uint8_t *)&signs_info.signs_detected, sizeof(protobuf_signs_t));

  switch (robot.info.state_info.process_state)
  {
  case protobuf_robot_process_state_t_ROBOT_STATE_IDLE:
  {
    if (robot.stopping_redlight)
    {
      // if ((robot.pre_signs_detect_obey.green_light == false) &&
      //     (robot.signs_detect_obey.green_light == true))
      if ((robot.signs_detect_obey.red_light == false) &&
          (robot.signs_detect_obey.green_light == true))
      {
        robot.stopping_redlight = false;
        robot.info.state_info.process_state = protobuf_robot_process_state_t_ROBOT_STATE_RUNNING;
        robot_control_set_direction(robot.direction_queue_data.direction_cmd_after_stop_temp);
      }
    }
    break;
  }
  case protobuf_robot_process_state_t_ROBOT_STATE_RUNNING:
  {
    if ((robot.pre_signs_detect_obey.green_light == false) &&
        (robot.signs_detect_obey.green_light == true))
    {
      protobuf_direction_cmd_t direction_cmd;
      robot_get_next_direction_cmd(&direction_cmd);
      robot_control_set_direction(direction_cmd);

      sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                     protobuf_message_type_t_MESSAGE_INFO,
                                     "Obey green light.");
    }

    if ((robot.pre_signs_detect_obey.red_light == false) &&
        (robot.signs_detect_obey.red_light == true))
    {
      robot.info.state_info.process_state = protobuf_robot_process_state_t_ROBOT_STATE_IDLE;
      robot.info.state_info.direct_state = protobuf_robot_direct_state_t_ROBOT_STATE_STOP_TEMP;
      sys_dcmotor_stop();
      robot_get_next_direction_cmd(&robot.direction_queue_data.direction_cmd_after_stop_temp);
      vl_cbuffer_clear(&robot.coor_cb);
      // vl_cbuffer_clear(&robot.signs_cb);

      robot.stopping_redlight = true;

      robot_control_update_stop_temp(&robot.info);
      robot_control_update_stop_temp(&robot.info);
      robot_control_update_stop_temp(&robot.info);

      sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                     protobuf_message_type_t_MESSAGE_INFO, "Obey red light.");
    }

    if ((robot.pre_signs_detect_obey.bus_stop == false) &&
        (robot.signs_detect_obey.bus_stop == true))
    {
      robot.info.state_info.process_state = protobuf_robot_process_state_t_ROBOT_STATE_IDLE;
      robot.info.state_info.direct_state = protobuf_robot_direct_state_t_ROBOT_STATE_STOP_TEMP;
      sys_dcmotor_stop();
      robot.stop_len_s     = 5;
      robot.stop_timestamp = HAL_GetTick();
      robot.is_stop_temp   = true;
      robot.direction_queue_data.direction_cmd_after_stop_temp =
      protobuf_direction_cmd_t_DIRECTION_CMD_MOVE_FOLLOW_LANE;
      vl_cbuffer_clear(&robot.coor_cb);
      vl_cbuffer_clear(&robot.signs_cb);

      // robot_control_update(&robot.info);
      robot_control_update_stop_temp(&robot.info);
      robot_control_update_stop_temp(&robot.info);
      robot_control_update_stop_temp(&robot.info);
      
      sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                     protobuf_message_type_t_MESSAGE_INFO,
                                     "Obey bus stop. Stop for 5 seconds.");
    }

    if ((robot.pre_signs_detect_obey.left_turn_only == false) &&
        (robot.signs_detect_obey.left_turn_only == true))
    {
      robot_control_set_direction(protobuf_direction_cmd_t_DIRECTION_CMD_MOVE_TURN_LEFT);

      sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                     protobuf_message_type_t_MESSAGE_INFO, "Turning left.");
    }

    if ((robot.pre_signs_detect_obey.children_crossing == false) &&
        (robot.signs_detect_obey.children_crossing == true))
    {
      robot.max_speed_temp = robot.info.var_info.max_setpoint_speed;

      robot_control_max_speed_set(robot.info.var_info.max_setpoint_speed * 0.6);

      sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                     protobuf_message_type_t_MESSAGE_INFO,
                                     "The speed is reduced to 60 percent of "
                                     "the original for 10 seconds.");

      delay_ms(10000, robot_control_set_pre_speed);
    }

    if ((robot.pre_signs_detect_obey.speed_limit_40 == false) &&
        (robot.signs_detect_obey.speed_limit_40 == true))
    {
      robot.max_speed_temp = robot.info.var_info.max_setpoint_speed;

      robot_control_max_speed_set(robot.info.var_info.max_setpoint_speed * 0.8);

      sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                     protobuf_message_type_t_MESSAGE_INFO,
                                     "The speed is reduced to 80 percent of "
                                     "the original for 10 seconds.");

      delay_ms(10000, robot_control_set_pre_speed);
    }

    if ((robot.pre_signs_detect_obey.stop == false) && (robot.signs_detect_obey.stop == true))
    {
      robot_control_stop();

      sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                     protobuf_message_type_t_MESSAGE_INFO, "Obey stop sign.");
    }

    if ((robot.pre_signs_detect_obey.no_stopping == false) &&
        (robot.signs_detect_obey.no_stopping == true))
    {
      sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                     protobuf_message_type_t_MESSAGE_INFO, "No stopping.");
    }

    break;
  }
  default:
  {
    break;
  }
  }

  memcpy((uint8_t *)&robot.pre_signs_detect_obey,
         (uint8_t *)&robot.signs_detect_obey, sizeof(protobuf_signs_t));

}

void robot_control_add_coor(protobuf_vision_coordinate_detected_t *coor)
{
#ifdef HAVE_SKIP_COUNTER
  if (robot.robot_start_skip_cnt)
  {
    robot.robot_start_skip_cnt--;
    return;
  }
#endif

  if (robot.info.state_info.direct_state == protobuf_robot_direct_state_t_ROBOT_STATE_STOP ||
      robot.info.state_info.direct_state == protobuf_robot_direct_state_t_ROBOT_STATE_STOP_TEMP)
  {
    return;
  }

  vl_cbuffer_write(&robot.coor_cb, (uint8_t *)coor,
                   sizeof(protobuf_vision_coordinate_detected_t));
}

void robot_control_add_signs(protobuf_signs_info_t *signs_info)
{
#ifdef HAVE_SKIP_COUNTER
  if (robot.robot_start_skip_cnt)
  {
    return;
  }
#endif

  // if (robot.info.state_info.direct_state == protobuf_robot_direct_state_t_ROBOT_STATE_STOP ||
  //     robot.info.state_info.direct_state == protobuf_robot_direct_state_t_ROBOT_STATE_STOP_TEMP)
  if (robot.info.state_info.direct_state == protobuf_robot_direct_state_t_ROBOT_STATE_STOP)
  {
    return;
  }

  vl_cbuffer_write(&robot.signs_cb, (uint8_t *)signs_info, sizeof(protobuf_signs_info_t));
}


void robot_control_start(void)
{
  robot.info.state_info.camera_state = ROBOT_CAMERA_STATE_0;
  robot.theta_offset                 = ROBOT_CAMERA_MODE_P_0_OFFSET;

  robot.input_data.center.first_value     = true;
  robot.input_data.coor_error.first_value = true;
  robot.input_data.left_down.first_value  = true;
  robot.input_data.right_down.first_value = true;
  robot.input_data.phi_error.first_value  = true;

  robot.is_stop_temp   = false;
  robot.stop_dur       = 0;
  robot.stop_len_s     = 0;
  robot.stop_timestamp = 0;
  
  sys_servo_set(0, SERVO_ALL);
  robot.is_first_time_enter_state = true;
  robot.stopping_redlight         = false;

  robot.info.state_info.process_state = protobuf_robot_process_state_t_ROBOT_STATE_RUNNING;
  robot.info.state_info.direct_state = protobuf_robot_direct_state_t_ROBOT_STATE_FOLLOW_LANE;
  sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                 protobuf_message_type_t_MESSAGE_INFO, "Robot started.");
  robot.robot_start_skip_cnt = ROBOT_SKIP_COUNTER;

  robot_control_cmd_process_crossroads(false);
}

void robot_control_stop(void)
{
  robot_control_clear();
  fuzzy_controller_clear();
  sys_dcmotor_stop();
  sys_servo_set(0, SERVO_ALL);
  robot_reset_direction_queue();

  robot.robot_start_skip_cnt         = ROBOT_SKIP_COUNTER;
  robot.info.state_info.camera_state = ROBOT_CAMERA_STATE_0;
  robot.theta_offset                 = ROBOT_CAMERA_MODE_P_0_OFFSET;

  robot.input_data.center.first_value     = true;
  robot.input_data.coor_error.first_value = true;
  robot.input_data.left_down.first_value  = true;
  robot.input_data.right_down.first_value = true;
  robot.input_data.phi_error.first_value  = true;

  robot.stopping_redlight = false;
  robot.is_stop_temp      = false;
  robot.stop_dur          = 0;
  robot.stop_len_s        = 0;
  robot.stop_timestamp    = 0;

  robot_control_update(&robot.info);

  sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                 protobuf_message_type_t_MESSAGE_INFO, "Robot stopped.");

  robot_control_cmd_process_crossroads(false);
}

void robot_control_info_get(void)
{
  protobuf_network_packet_t p;
  p.which_params = protobuf_network_packet_t_robot_info_rsp_tag;
  memcpy((uint8_t *)&p.params.robot_info_rsp, (uint8_t *)&robot.info,
         sizeof(protobuf_robot_info_rsp_t));
  sys_network_send_packet(&p, protobuf_device_addr_t_DEVICE_ADDR_APP);
}

void robot_control_max_speed_set(float max_speed)
{
  robot.info.var_info.max_setpoint_speed = max_speed;
  fuzzy_controller_max_speed_set(max_speed);
}

void robot_control_max_rotation_angle_set(float max_rotation_angle)
{
  robot.info.var_info.max_theta = max_rotation_angle;
  fuzzy_controller_max_rotation_angle_set(max_rotation_angle);
  // sys_servo_set(max_rotation_angle, SERVO_ALL);
  // sys_servo_set(max_rotation_angle, SERVO_DRIVE);
}

void robot_add_direction_queue(protobuf_direction_cmd_t direct_cmd)
{
  if (robot.direction_queue_data.queue_index < DIRECTION_QUEUE_SIZE)
  {
    robot.direction_queue_data.direction_queue[robot.direction_queue_data.queue_index] =
    direct_cmd;
    robot.direction_queue_data.next_direction_cmd =
    robot.direction_queue_data.direction_queue[0];
    robot.direction_queue_data.queue_index++;
    return;
  }
  sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                 protobuf_message_type_t_MESSAGE_WARN,
                                 "Direction commands queue is out of slot.");
  return;
}


void robot_update_direction_queue(void)
{
  protobuf_network_packet_t p;
  p.which_params = protobuf_network_packet_t_direction_list_tag;
  p.params.direction_list.first   = robot.direction_queue_data.direction_queue[0];
  p.params.direction_list.second  = robot.direction_queue_data.direction_queue[1];
  p.params.direction_list.third   = robot.direction_queue_data.direction_queue[2];
  p.params.direction_list.fourth  = robot.direction_queue_data.direction_queue[3];
  p.params.direction_list.fifth   = robot.direction_queue_data.direction_queue[4];
  p.params.direction_list.sixth   = robot.direction_queue_data.direction_queue[5];
  p.params.direction_list.seventh = robot.direction_queue_data.direction_queue[6];
  p.params.direction_list.eighth  = robot.direction_queue_data.direction_queue[7];
  p.params.direction_list.ninth   = robot.direction_queue_data.direction_queue[8];
  p.params.direction_list.tenth   = robot.direction_queue_data.direction_queue[9];

  sys_network_send_packet(&p, protobuf_device_addr_t_DEVICE_ADDR_APP);
}


/* Private definitions ----------------------------------------------- */
static void robot_control_clear(void)
{
  robot.info.state_info.process_state = protobuf_robot_process_state_t_ROBOT_STATE_IDLE;
  robot.info.state_info.direct_state = protobuf_robot_direct_state_t_ROBOT_STATE_STOP;
  robot.info.state_info.pre_direct_state = protobuf_robot_direct_state_t_ROBOT_STATE_STOP;
  robot.info.var_info.theta                  = 0;
  robot.info.var_info.current_setpoint_speed = 0;
  robot.info.var_info.current_speed          = 0;
  vl_cbuffer_clear(&robot.coor_cb);
  vl_cbuffer_clear(&robot.signs_cb);
}

static void robot_control_update(protobuf_robot_info_rsp_t *fuzzy_var)
{
  protobuf_network_packet_t p;
  p.which_params = protobuf_network_packet_t_robot_info_rsp_tag;
  memcpy((uint8_t *)&p.params.robot_info_rsp, (uint8_t *)fuzzy_var,
         sizeof(protobuf_robot_info_rsp_t));
  sys_network_send_packet(&p, protobuf_device_addr_t_DEVICE_ADDR_APP);
}

static void robot_control_update_stop_temp(protobuf_robot_info_rsp_t *fuzzy_var)
{
  protobuf_network_packet_t p;
  p.which_params = protobuf_network_packet_t_robot_info_rsp_tag;
  memcpy((uint8_t *)&p.params.robot_info_rsp, (uint8_t *)fuzzy_var,
         sizeof(protobuf_robot_info_rsp_t));

  p.params.robot_info_rsp.var_info.current_speed          = 0;
  p.params.robot_info_rsp.var_info.current_setpoint_speed = 0;
  p.params.robot_info_rsp.var_info.time_process           = 1;

  sys_network_send_packet(&p, protobuf_device_addr_t_DEVICE_ADDR_APP);
}

static float robot_cal_coor_error(protobuf_vision_coordinate_detected_t *coor,
                                  protobuf_robot_direct_state_t direct_state)
{
  switch (direct_state)
  {
  case protobuf_robot_direct_state_t_ROBOT_STATE_MOVE_FORWARD:
  case protobuf_robot_direct_state_t_ROBOT_STATE_FOLLOW_LANE:
  {
    return (ROBOT_COORDINATE_CENTER_SETPOINT - coor->center_coor);
    break;
  }
  case protobuf_robot_direct_state_t_ROBOT_STATE_TURN_LEFT:
  {
    return (ROBOT_COORDINATE_TURN_LEFT_LEFT_SETPOINT - coor->left_down_coor);
    break;
  }
  case protobuf_robot_direct_state_t_ROBOT_STATE_TURN_RIGHT:
  {
    return (ROBOT_COORDINATE_TURN_RIGHT_RIGHT_SETPOINT - coor->right_down_coor);
    break;
  }
  default:
  {
    return 0;
    break;
  }
  }
  return 0;
}

static float robot_cal_lane_angle_error(protobuf_vision_coordinate_detected_t *coor)
{
  return (ROBOT_LANE_ANGLE_SETPOINT - coor->phi);
}

static void robot_process_input_lane_center(protobuf_vision_coordinate_detected_t *coor,
                                            float *center)
{
  float center_tmp = coor->center_coor;

  if (robot.input_data.center.first_value)
  {
    robot.input_data.center.first_value = false;
    robot.input_data.center.count_s     = 0;
    for (int i = 0; i < ROBOT_ERROR_AVG_NUM; i++)
    {
      robot.input_data.center.value[i] = center_tmp;
    }
  }
  else
  {
    for (int i = (ROBOT_ERROR_AVG_NUM - 1); i > 0; i--)
    {
      robot.input_data.center.value[i] = robot.input_data.center.value[i - 1];
    }
    robot.input_data.center.value[0] = center_tmp;
  }

  float average_tmp = calculate_average((robot.input_data.center.value + 1),
                                        (ROBOT_ERROR_AVG_NUM - 1));

  float delta_tmp = abs((robot.input_data.center.value[0] - average_tmp));

  // delta_tmp < ROBOT_SINGULARITY_CENTER_GAP
  if (delta_tmp < ROBOT_SINGULARITY_CENTER_GAP)
  {
    robot.input_data.center.count_s = 0;
    *center                    = robot.input_data.center.value[0];
    return;
  }

  // delta_tmp >= ROBOT_SINGULARITY_CENTER_GAP
  for (int i = (ROBOT_SINGULARITY_CNT - 1); i > 0; i--)
  {
    robot.input_data.center.outlier[i] = robot.input_data.center.outlier[i - 1];
  }
  robot.input_data.center.outlier[0] = robot.input_data.center.value[0];
  robot.input_data.center.count_s += 1;

  if (robot.input_data.center.count_s >= ROBOT_SINGULARITY_CNT)
  {
    float average_s_tmp = calculate_average((robot.input_data.center.outlier + 1),
                                            (ROBOT_SINGULARITY_CNT - 1));
    float delta_s_tmp = abs((robot.input_data.center.outlier[0] - average_s_tmp));
    if (delta_s_tmp < ROBOT_SINGULARITY_CENTER_GAP)
    {
      robot.input_data.center.count_s     = 0;
      robot.input_data.center.first_value = true;
      *center = robot.input_data.center.outlier[0];

      sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                     protobuf_message_type_t_MESSAGE_WARN,
                                     "Reset first value.");

      return;
    }
  }

  robot.input_data.center.value[0] = robot.input_data.center.value[1];
  *center                     = robot.input_data.center.value[0];


  sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                 protobuf_message_type_t_MESSAGE_WARN,
                                 "Outlier detected. center_tmp = %d, "
                                 "center[0] = %d, average_center_tmp = %d",
                                 (int16_t)center_tmp,
                                 (int16_t)robot.input_data.center.value[0],
                                 (int16_t)average_tmp);

  return;
}

static void robot_process_input_lane_angle(protobuf_vision_coordinate_detected_t *coor,
                                           float *phi_error)
{
  float phi_error_tmp = robot_cal_lane_angle_error(coor);

  if (robot.input_data.phi_error.first_value)
  {
    robot.input_data.phi_error.first_value = false;
    robot.input_data.phi_error.count_s     = 0;
    for (int i = 0; i < ROBOT_ERROR_AVG_NUM; i++)
    {
      robot.input_data.phi_error.value[i] = phi_error_tmp;
    }
  }
  else
  {
    for (int i = (ROBOT_ERROR_AVG_NUM - 1); i > 0; i--)
    {
      robot.input_data.phi_error.value[i] = robot.input_data.phi_error.value[i - 1];
    }
    robot.input_data.phi_error.value[0] = phi_error_tmp;
  }

  float average_tmp = calculate_average((robot.input_data.phi_error.value + 1),
                                        (ROBOT_ERROR_AVG_NUM - 1));

  float delta_tmp = abs((robot.input_data.phi_error.value[0] - average_tmp));

  // delta_tmp < ROBOT_SINGULARITY_PHI_GAP
  if (delta_tmp < ROBOT_SINGULARITY_PHI_GAP)
  {
    robot.input_data.phi_error.count_s = 0;
    *phi_error                    = robot.input_data.phi_error.value[0];
    return;
  }

  // delta_tmp >= ROBOT_SINGULARITY_PHI_GAP
  for (int i = (ROBOT_SINGULARITY_CNT - 1); i > 0; i--)
  {
    robot.input_data.phi_error.outlier[i] = robot.input_data.phi_error.outlier[i - 1];
  }
  robot.input_data.phi_error.outlier[0] = robot.input_data.phi_error.value[0];
  robot.input_data.phi_error.count_s += 1;

  if (robot.input_data.phi_error.count_s >= ROBOT_SINGULARITY_CNT)
  {
    float average_s_tmp = calculate_average((robot.input_data.phi_error.outlier + 1),
                                            (ROBOT_SINGULARITY_CNT - 1));
    float delta_s_tmp = abs((robot.input_data.phi_error.outlier[0] - average_s_tmp));
    if (delta_s_tmp < ROBOT_SINGULARITY_PHI_GAP)
    {
      robot.input_data.phi_error.count_s     = 0;
      robot.input_data.phi_error.first_value = true;
      *phi_error = robot.input_data.phi_error.outlier[0];

      sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                     protobuf_message_type_t_MESSAGE_WARN,
                                     "Reset first value.");

      return;
    }
  }

  robot.input_data.phi_error.value[0] = robot.input_data.phi_error.value[1];
  *phi_error                     = robot.input_data.phi_error.value[0];


  sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                 protobuf_message_type_t_MESSAGE_WARN,
                                 "Outlier detected. phi_error_tmp = %d, "
                                 "phi_error[0] = %d, average_error_tmp = %d",
                                 (int16_t)phi_error_tmp,
                                 (int16_t)robot.input_data.phi_error.value[0],
                                 (int16_t)average_tmp);

  return;
}

static void robot_process_input_lane_left(protobuf_vision_coordinate_detected_t *coor,
                                          float *left_down_coor)
{
  float left_down_tmp = coor->left_down_coor;

  if (robot.input_data.left_down.first_value)
  {
    robot.input_data.left_down.first_value = false;
    robot.input_data.left_down.count_s     = 0;
    for (int i = 0; i < ROBOT_ERROR_AVG_NUM; i++)
    {
      robot.input_data.left_down.value[i] = left_down_tmp;
    }
  }
  else
  {
    for (int i = (ROBOT_ERROR_AVG_NUM - 1); i > 0; i--)
    {
      robot.input_data.left_down.value[i] = robot.input_data.left_down.value[i - 1];
    }
    robot.input_data.left_down.value[0] = left_down_tmp;
  }

  float average_tmp = calculate_average((robot.input_data.left_down.value + 1),
                                        (ROBOT_ERROR_AVG_NUM - 1));

  float delta_tmp = abs((robot.input_data.left_down.value[0] - average_tmp));

  // delta_tmp < ROBOT_SINGULARITY_LEFT_GAP
  if (delta_tmp < ROBOT_SINGULARITY_LEFT_GAP)
  {
    robot.input_data.left_down.count_s = 0;
    *left_down_coor                    = robot.input_data.left_down.value[0];
    return;
  }

  // delta_tmp >= ROBOT_SINGULARITY_LEFT_GAP
  for (int i = (ROBOT_SINGULARITY_CNT - 1); i > 0; i--)
  {
    robot.input_data.left_down.outlier[i] = robot.input_data.left_down.outlier[i - 1];
  }
  robot.input_data.left_down.outlier[0] = robot.input_data.left_down.value[0];
  robot.input_data.left_down.count_s += 1;

  if (robot.input_data.left_down.count_s >= ROBOT_SINGULARITY_CNT)
  {
    float average_s_tmp = calculate_average((robot.input_data.left_down.outlier + 1),
                                            (ROBOT_SINGULARITY_CNT - 1));
    float delta_s_tmp = abs((robot.input_data.left_down.outlier[0] - average_s_tmp));
    if (delta_s_tmp < ROBOT_SINGULARITY_LEFT_GAP)
    {
      robot.input_data.left_down.count_s     = 0;
      robot.input_data.left_down.first_value = true;
      *left_down_coor = robot.input_data.left_down.outlier[0];

      sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                     protobuf_message_type_t_MESSAGE_WARN,
                                     "Reset first value.");

      return;
    }
  }

  robot.input_data.left_down.value[0] = robot.input_data.left_down.value[1];
  *left_down_coor                     = robot.input_data.left_down.value[0];


  sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                 protobuf_message_type_t_MESSAGE_WARN,
                                 "Outlier detected. left_down_tmp = %d, "
                                 "left_down[0] = %d, average_error_tmp = %d",
                                 (int16_t)left_down_tmp,
                                 (int16_t)robot.input_data.left_down.value[0],
                                 (int16_t)average_tmp);

  return;
}

static void robot_process_input_lane_right(protobuf_vision_coordinate_detected_t *coor,
                                           float *right_down_coor)
{
  float right_down_tmp = coor->right_down_coor;

  if (robot.input_data.right_down.first_value)
  {
    robot.input_data.right_down.first_value = false;
    robot.input_data.right_down.count_s     = 0;
    for (int i = 0; i < ROBOT_ERROR_AVG_NUM; i++)
    {
      robot.input_data.right_down.value[i] = right_down_tmp;
    }
  }
  else
  {
    for (int i = (ROBOT_ERROR_AVG_NUM - 1); i > 0; i--)
    {
      robot.input_data.right_down.value[i] = robot.input_data.right_down.value[i - 1];
    }
    robot.input_data.right_down.value[0] = right_down_tmp;
  }

  float average_tmp = calculate_average((robot.input_data.right_down.value + 1),
                                        (ROBOT_ERROR_AVG_NUM - 1));

  float delta_tmp = abs((robot.input_data.right_down.value[0] - average_tmp));

  // delta_tmp < ROBOT_SINGULARITY_RIGHT_GAP
  if (delta_tmp < ROBOT_SINGULARITY_RIGHT_GAP)
  {
    robot.input_data.right_down.count_s = 0;
    *right_down_coor                    = robot.input_data.right_down.value[0];
    return;
  }

  // delta_tmp >= ROBOT_SINGULARITY_RIGHT_GAP
  for (int i = (ROBOT_SINGULARITY_CNT - 1); i > 0; i--)
  {
    robot.input_data.right_down.outlier[i] = robot.input_data.right_down.outlier[i - 1];
  }
  robot.input_data.right_down.outlier[0] = robot.input_data.right_down.value[0];
  robot.input_data.right_down.count_s += 1;

  if (robot.input_data.right_down.count_s >= ROBOT_SINGULARITY_CNT)
  {
    float average_s_tmp = calculate_average((robot.input_data.right_down.outlier + 1),
                                            (ROBOT_SINGULARITY_CNT - 1));
    float delta_s_tmp = abs((robot.input_data.right_down.outlier[0] - average_s_tmp));
    if (delta_s_tmp < ROBOT_SINGULARITY_RIGHT_GAP)
    {
      robot.input_data.right_down.count_s     = 0;
      robot.input_data.right_down.first_value = true;
      *right_down_coor = robot.input_data.right_down.outlier[0];

      sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                     protobuf_message_type_t_MESSAGE_WARN,
                                     "Reset first value.");

      return;
    }
  }

  robot.input_data.right_down.value[0] = robot.input_data.right_down.value[1];
  *right_down_coor                     = robot.input_data.right_down.value[0];


  sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                 protobuf_message_type_t_MESSAGE_WARN,
                                 "Outlier detected. right_down_tmp = %d, "
                                 "right_down[0] = %d, average_error_tmp = %d",
                                 (int16_t)right_down_tmp,
                                 (int16_t)robot.input_data.right_down.value[0],
                                 (int16_t)average_tmp);

  return;
}

static void robot_process_input_coor_error(protobuf_vision_coordinate_detected_t *coor,
                                           float *coor_error,
                                           protobuf_robot_direct_state_t direct_state)
{
  float coor_error_tmp = robot_cal_coor_error(coor, direct_state);


  if (robot.input_data.coor_error.first_value)
  {
    robot.input_data.coor_error.first_value = false;
    robot.input_data.coor_error.count_s     = 0;
    for (int i = 0; i < ROBOT_ERROR_AVG_NUM; i++)
    {
      robot.input_data.coor_error.value[i]      = coor_error_tmp;
    }
  }
  else
  {
    for (int i = (ROBOT_ERROR_AVG_NUM - 1); i > 0; i--)
    {
      robot.input_data.coor_error.value[i] = robot.input_data.coor_error.value[i - 1];
    }
    robot.input_data.coor_error.value[0]      = coor_error_tmp;
  }

  float average_tmp =
  calculate_average((robot.input_data.coor_error.value + 1), (ROBOT_ERROR_AVG_NUM - 1));

  float delta_tmp = abs((robot.input_data.coor_error.value[0] - average_tmp));

  // delta_tmp < ROBOT_SINGULARITY_ERROR_GAP
  if (delta_tmp < ROBOT_SINGULARITY_ERROR_GAP)
  {
    robot.input_data.coor_error.count_s = 0;
    *coor_error                   = robot.input_data.coor_error.value[0];
    return;
  }

  // delta_tmp >= ROBOT_SINGULARITY_ERROR_GAP
  for (int i = (ROBOT_SINGULARITY_CNT - 1); i > 0; i--)
  {
    robot.input_data.coor_error.outlier[i] = robot.input_data.coor_error.outlier[i - 1];
  }
  robot.input_data.coor_error.outlier[0]      = robot.input_data.coor_error.value[0];
  robot.input_data.coor_error.count_s += 1;

  if (robot.input_data.coor_error.count_s >= ROBOT_SINGULARITY_CNT)
  {
    float average_s_tmp =
    calculate_average((robot.input_data.coor_error.outlier + 1), (ROBOT_SINGULARITY_CNT - 1));
    float delta_s_tmp = abs((robot.input_data.coor_error.outlier[0] - average_s_tmp));
    if (delta_s_tmp < ROBOT_SINGULARITY_ERROR_GAP)
    {
      robot.input_data.coor_error.count_s     = 0;
      robot.input_data.coor_error.first_value = true;
      *coor_error                       = robot.input_data.coor_error.outlier[0];

      sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                     protobuf_message_type_t_MESSAGE_WARN,
                                     "Reset first value.");

      return;
    }
  }

  robot.input_data.coor_error.value[0]      = robot.input_data.coor_error.value[1];
  *coor_error                         = robot.input_data.coor_error.value[0];


  sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                 protobuf_message_type_t_MESSAGE_WARN,
                                 "Outlier detected. coor_error_tmp = %d, "
                                 "coor_error[0] = %d, average_error_tmp = %d",
                                 (int16_t)coor_error_tmp,
                                 (int16_t)robot.input_data.coor_error.value[0],
                                 (int16_t)average_tmp);

  return;
}

static float calculate_average(float *array, uint16_t length)
{
  if (length == 0)
    return 0.0f;

  float sum = 0.0f;
  for (uint16_t i = 0; i < length; i++)
  {
    sum += array[i];
  }

  return sum / length;
}

static void robot_camera_state_set(robot_camera_state_t state)
{
  switch (state)
  {
    case ROBOT_CAMERA_STATE_0:
    {
      robot.robot_rotate_camera_skip_cnt = ROBOT_CAMERA_SKIP_COUNTER;
      robot.info.state_info.camera_state = ROBOT_CAMERA_STATE_0;
      robot.theta_offset                 = ROBOT_CAMERA_MODE_P_0_OFFSET;
      sys_servo_set(ROBOT_CAMERA_SERVO_ANGLE_0, SERVO_CAMERA);
      sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                     protobuf_message_type_t_MESSAGE_INFO,
                                     "Camera rotate to ZERO position.");
      break;
    }
    case ROBOT_CAMERA_STATE_P_30:
    {
      robot.robot_rotate_camera_skip_cnt = ROBOT_CAMERA_SKIP_COUNTER;
      robot.info.state_info.camera_state = ROBOT_CAMERA_STATE_P_30;
      robot.theta_offset                 = ROBOT_CAMERA_MODE_P_30_OFFSET;
      sys_servo_set(ROBOT_CAMERA_SERVO_ANGLE_P_30, SERVO_CAMERA);
      sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                     protobuf_message_type_t_MESSAGE_INFO,
                                     "Camera rotate to P30 position.");
      break;
    }
    case ROBOT_CAMERA_STATE_P_45:
    {
      robot.robot_rotate_camera_skip_cnt = ROBOT_CAMERA_SKIP_COUNTER;
      robot.info.state_info.camera_state = ROBOT_CAMERA_STATE_P_45;
      robot.theta_offset                 = ROBOT_CAMERA_MODE_P_45_OFFSET;
      sys_servo_set(ROBOT_CAMERA_SERVO_ANGLE_P_45, SERVO_CAMERA);
      sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                     protobuf_message_type_t_MESSAGE_INFO,
                                     "Camera rotate to P45 position.");
      break;
    }
    case ROBOT_CAMERA_STATE_N_30:
    {
      robot.robot_rotate_camera_skip_cnt = ROBOT_CAMERA_SKIP_COUNTER;
      robot.info.state_info.camera_state = ROBOT_CAMERA_STATE_N_30;
      robot.theta_offset                 = ROBOT_CAMERA_MODE_N_30_OFFSET;
      sys_servo_set(ROBOT_CAMERA_SERVO_ANGLE_N_30, SERVO_CAMERA);
      sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                     protobuf_message_type_t_MESSAGE_INFO,
                                     "Camera rotate to N30 position.");
      break;
    }
    case ROBOT_CAMERA_STATE_N_45:
    {
      robot.robot_rotate_camera_skip_cnt = ROBOT_CAMERA_SKIP_COUNTER;
      robot.info.state_info.camera_state = ROBOT_CAMERA_STATE_N_45;
      robot.theta_offset                 = ROBOT_CAMERA_MODE_N_45_OFFSET;
      sys_servo_set(ROBOT_CAMERA_SERVO_ANGLE_N_45, SERVO_CAMERA);
      sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                     protobuf_message_type_t_MESSAGE_INFO,
                                     "Camera rotate to N45 position.");
      break;
    }
    default:
    {
      break;
    }
  }
}

static void robot_camera_rotate_fsm(float angle_error)
{
  if (robot.robot_rotate_camera_skip_cnt)
  {
    robot.robot_rotate_camera_skip_cnt--;
    return;
  }

  switch (robot.info.state_info.camera_state)
  {
  case ROBOT_CAMERA_STATE_0:
  {
    if (angle_error < ROBOT_CAMERA_0_TO_P_30_CONDITION)
    {
      robot_camera_state_set(ROBOT_CAMERA_STATE_P_30);
    }
    else if (angle_error > ROBOT_CAMERA_0_TO_N_30_CONDITION)
    {
      robot_camera_state_set(ROBOT_CAMERA_STATE_N_30);
    }
    break;
  }
  case ROBOT_CAMERA_STATE_P_30:
  {
    if (angle_error < ROBOT_CAMERA_P_30_TO_P_45_CONDITION)
    {
      robot_camera_state_set(ROBOT_CAMERA_STATE_P_45);
    }
    else if (angle_error > ROBOT_CAMERA_P_30_TO_0_CONDITION)
    {
      robot_camera_state_set(ROBOT_CAMERA_STATE_0);
    }
    break;
  }
  case ROBOT_CAMERA_STATE_P_45:
  {
    if (angle_error > ROBOT_CAMERA_P_45_TO_P_30_CONDITION)
    {
      robot_camera_state_set(ROBOT_CAMERA_STATE_P_30);
    }
    break;
  }
  case ROBOT_CAMERA_STATE_N_30:
  {
    if (angle_error < ROBOT_CAMERA_N_30_TO_0_CONDITION)
    {
      robot_camera_state_set(ROBOT_CAMERA_STATE_0);
    }
    else if (angle_error > ROBOT_CAMERA_N_30_TO_N_45_CONDITION)
    {
      robot_camera_state_set(ROBOT_CAMERA_STATE_N_45);
    }
    break;
  }
  case ROBOT_CAMERA_STATE_N_45:
  {
    if (angle_error < ROBOT_CAMERA_N_45_TO_N_30_CONDITION)
    {
      robot_camera_state_set(ROBOT_CAMERA_STATE_N_30);
    }
    break;
  }
  default:
  {
    break;
  }
  }
  return;
}

static void robot_direct_fsm_stop(protobuf_vision_coordinate_detected_t *coor)
{
  return;
}

static void robot_direct_fsm_follow_lane(protobuf_vision_coordinate_detected_t *coor)
{
  if (robot.is_first_time_enter_state)
  {
    robot.is_first_time_enter_state = false;
    fuzzy_controller_set_following_coef();
    sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                   protobuf_message_type_t_MESSAGE_INFO,
                                   "Following lane. Using 3 inputs controller.");
  }
  robot_process_input_lane_center(coor, &robot.info.var_info.center);
  robot_process_input_coor_error(coor, &robot.info.var_info.e,
                                 protobuf_robot_direct_state_t_ROBOT_STATE_FOLLOW_LANE);
  robot_process_input_lane_angle(coor, &robot.info.var_info.e_phi);

  fuzzy_controller_process_3_input(&robot.info.var_info, robot.theta_offset);

  sys_servo_set(robot.info.var_info.theta, SERVO_DRIVE);
  sys_dcmotor_speed_set(robot.info.var_info.current_setpoint_speed);

  robot_control_update(&robot.info);
  robot_camera_rotate_fsm(robot.info.var_info.e_phi);

  return;
}

static void robot_direct_fsm_move_forward(protobuf_vision_coordinate_detected_t *coor)
{
  if (robot.is_first_time_enter_state)
  {
    robot.is_first_time_enter_state = false;
    robot_camera_state_set(ROBOT_CAMERA_STATE_0);
    fuzzy_controller_set_following_coef();
    sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                   protobuf_message_type_t_MESSAGE_INFO,
                                   "Moving forawrd. Using 3 inputs controller.");

    robot_control_max_speed_set(robot.info.var_info.max_setpoint_speed *
                                ROBOT_SPEED_MOVE_FORWARD_PERCENT);
  }
  robot_process_input_lane_center(coor, &robot.info.var_info.center);
  robot_process_input_coor_error(coor, &robot.info.var_info.e,
                                 protobuf_robot_direct_state_t_ROBOT_STATE_MOVE_FORWARD);
  robot_process_input_lane_angle(coor, &robot.info.var_info.e_phi);

  // robot.info.var_info.e = robot_cal_coor_error(coor, protobuf_robot_direct_state_t_ROBOT_STATE_MOVE_FORWARD);
  // robot.info.var_info.e_phi = robot_cal_lane_angle_error(coor);

  fuzzy_controller_process_3_input(&robot.info.var_info, robot.theta_offset);

  sys_servo_set(robot.info.var_info.theta, SERVO_DRIVE);
  sys_dcmotor_speed_set(robot.info.var_info.current_setpoint_speed);

  robot_control_update(&robot.info);

  if ((robot.pre_coor_recv.is_crossing_crossroads == true) &&
      (coor->is_crossing_crossroads == false))
  {
    robot_control_max_speed_set(robot.info.var_info.max_setpoint_speed /
                                ROBOT_SPEED_MOVE_FORWARD_PERCENT);
    robot_control_set_direction(protobuf_direction_cmd_t_DIRECTION_CMD_MOVE_FOLLOW_LANE);
  }

  return;
}

static void robot_direct_fsm_turn_right(protobuf_vision_coordinate_detected_t *coor)
{
  if (robot.is_first_time_enter_state)
  {
    robot.is_first_time_enter_state = false;
    robot_camera_state_set(ROBOT_CAMERA_STATE_N_30);

    robot.first_not_see_left_lane   = false;
    robot.first_see_left_lane_again = false;

    fuzzy_controller_set_turning_coef();
    sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                   protobuf_message_type_t_MESSAGE_INFO,
                                   "Turning right. Using 2 inputs controller.");

    robot.is_ready_to_return_following_state = false;
    robot.turn_skip_cnt                      = ROBOT_TURN_SKIP_COUNTER;
    robot.turn_continous_cnt                 = 0;
  }
  robot_process_input_lane_left(coor, &robot.info.var_info.left_down_coor);
  robot_process_input_lane_right(coor, &robot.info.var_info.right_down_coor);
  robot_process_input_coor_error(coor, &robot.info.var_info.e,
                                 protobuf_robot_direct_state_t_ROBOT_STATE_TURN_RIGHT);
  // robot_process_input_lane_angle(coor, &robot.info.var_info.e_phi);

  fuzzy_controller_process_2_input(&robot.info.var_info, robot.theta_offset);

  sys_servo_set(robot.info.var_info.theta, SERVO_DRIVE);
  sys_dcmotor_speed_set(robot.info.var_info.current_setpoint_speed);

  robot_control_update(&robot.info);

  if (robot.turn_skip_cnt)
  {
    robot.turn_skip_cnt--;
    return;
  }

  if ((robot.pre_coor_recv.is_left_lane_available == false) &&
      (coor->is_left_lane_available == true))
  {
    robot.is_ready_to_return_following_state = true;
    robot.turn_continous_cnt                 = 0;
  }
  else if ((robot.pre_coor_recv.is_left_lane_available == true) &&
           (coor->is_left_lane_available == false))
  {
    robot.is_ready_to_return_following_state = false;
    robot.turn_continous_cnt                 = 0;
  }

  if (!robot.is_ready_to_return_following_state)
  {
    return;
  }

  robot.turn_continous_cnt++;

  if (robot.turn_continous_cnt >= ROBOT_TURN_COUNTINOUS_COUNTER)
  {
    robot_control_set_direction(protobuf_direction_cmd_t_DIRECTION_CMD_MOVE_FOLLOW_LANE);
  }

  return;
}

static void robot_direct_fsm_turn_left(protobuf_vision_coordinate_detected_t *coor)
{
  if (robot.is_first_time_enter_state)
  {
    robot.is_first_time_enter_state = false;
    robot_camera_state_set(ROBOT_CAMERA_STATE_P_30);

    robot.first_not_see_right_lane   = false;
    robot.first_see_right_lane_again = false;

    fuzzy_controller_set_turning_coef();
    sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                   protobuf_message_type_t_MESSAGE_INFO,
                                   "Turning left. Using 2 inputs controller.");

    robot.is_ready_to_return_following_state = false;
    robot.turn_skip_cnt                      = ROBOT_TURN_SKIP_COUNTER;
    robot.turn_continous_cnt                 = 0;
  }
  robot_process_input_lane_left(coor, &robot.info.var_info.left_down_coor);
  robot_process_input_lane_right(coor, &robot.info.var_info.right_down_coor);
  robot_process_input_coor_error(coor, &robot.info.var_info.e,
                                 protobuf_robot_direct_state_t_ROBOT_STATE_TURN_LEFT);
  // robot_process_input_lane_angle(coor, &robot.info.var_info.e_phi);

  fuzzy_controller_process_2_input(&robot.info.var_info, robot.theta_offset);

  sys_servo_set(robot.info.var_info.theta, SERVO_DRIVE);
  sys_dcmotor_speed_set(robot.info.var_info.current_setpoint_speed);

  robot_control_update(&robot.info);

  if (robot.turn_skip_cnt)
  {
    robot.turn_skip_cnt--;
    return;
  }

  if ((robot.pre_coor_recv.is_right_lane_available == false) &&
      (coor->is_right_lane_available == true))
  {
    robot.is_ready_to_return_following_state = true;
    robot.turn_continous_cnt                 = 0;
  }
  else if ((robot.pre_coor_recv.is_right_lane_available == true) &&
           (coor->is_right_lane_available == false))
  {
    robot.is_ready_to_return_following_state = false;
    robot.turn_continous_cnt                 = 0;
  }

  if (!robot.is_ready_to_return_following_state)
  {
    return;
  }

  robot.turn_continous_cnt++;

  if (robot.turn_continous_cnt >= ROBOT_TURN_COUNTINOUS_COUNTER)
  {
    robot_control_set_direction(protobuf_direction_cmd_t_DIRECTION_CMD_MOVE_FOLLOW_LANE);
  }

  return;
}

static void robot_control_cmd_process_crossroads(bool is_crossroads)
{
  protobuf_network_packet_t p;
  p.which_params = protobuf_network_packet_t_cv_command_tag;
  p.params.cv_command.process_crossroads = is_crossroads;
  
  sys_network_send_packet(&p, protobuf_device_addr_t_DEVICE_ADDR_TX2);
  sys_network_send_packet(&p, protobuf_device_addr_t_DEVICE_ADDR_TX2);
  sys_network_send_packet(&p, protobuf_device_addr_t_DEVICE_ADDR_TX2);
}

static void robot_reset_direction_queue(void)
{
  robot.direction_queue_data.queue_index = 0;
  robot.direction_queue_data.next_direction_cmd = protobuf_direction_cmd_t_DIRECTION_CMD_NONE;
  robot.direction_queue_data.direction_cmd_after_stop_temp =
  protobuf_direction_cmd_t_DIRECTION_CMD_MOVE_FOLLOW_LANE;

  for (uint8_t i = 0; i < DIRECTION_QUEUE_SIZE; i++)
  {
    robot.direction_queue_data.direction_queue[i] = protobuf_direction_cmd_t_DIRECTION_CMD_NONE;
  }
}

static void robot_control_set_direction(protobuf_direction_cmd_t direct_cmd)
{
  robot.is_first_time_enter_state = true;
  switch (direct_cmd)
  {
  case protobuf_direction_cmd_t_DIRECTION_CMD_MOVE_FORWARD:
  {
    sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                   protobuf_message_type_t_MESSAGE_INFO, "Moving forward.");
    robot.info.state_info.direct_state = protobuf_robot_direct_state_t_ROBOT_STATE_MOVE_FORWARD;

    robot.input_data.center.first_value     = true;
    robot.input_data.coor_error.first_value = true;
    robot.input_data.phi_error.first_value  = true;

    robot_control_cmd_process_crossroads(true);
    break;
  }
  case protobuf_direction_cmd_t_DIRECTION_CMD_MOVE_FOLLOW_LANE:
  {
    sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                   protobuf_message_type_t_MESSAGE_INFO, "Following lane.");
    robot.info.state_info.direct_state = protobuf_robot_direct_state_t_ROBOT_STATE_FOLLOW_LANE;

    robot.input_data.center.first_value     = true;
    robot.input_data.coor_error.first_value = true;
    robot.input_data.phi_error.first_value  = true;

    robot_control_cmd_process_crossroads(false);
    break;
  }
  case protobuf_direction_cmd_t_DIRECTION_CMD_MOVE_TURN_LEFT:
  {
    sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                   protobuf_message_type_t_MESSAGE_INFO, "Turning left.");
    robot.info.state_info.direct_state = protobuf_robot_direct_state_t_ROBOT_STATE_TURN_LEFT;

    robot.input_data.left_down.first_value  = true;
    robot.input_data.coor_error.first_value = true;
    robot.input_data.phi_error.first_value  = true;

    robot_control_cmd_process_crossroads(false);
    break;
  }
  case protobuf_direction_cmd_t_DIRECTION_CMD_MOVE_TURN_RIGHT:
  {
    sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                   protobuf_message_type_t_MESSAGE_INFO, "Turning right.");
    robot.info.state_info.direct_state = protobuf_robot_direct_state_t_ROBOT_STATE_TURN_RIGHT;

    robot.input_data.right_down.first_value = true;
    robot.input_data.coor_error.first_value = true;
    robot.input_data.phi_error.first_value  = true;

    robot_control_cmd_process_crossroads(false);
    break;
  }
  default:
  {
    sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                   protobuf_message_type_t_MESSAGE_ERR, "Invalid direction.");
    break;
  }
  }
}

static void robot_get_next_direction_cmd(protobuf_direction_cmd_t *direct_cmd)
{
  *direct_cmd = robot.direction_queue_data.next_direction_cmd;

  for (uint8_t i = 0; i < (DIRECTION_QUEUE_SIZE - 1); i++)
  {
    robot.direction_queue_data.direction_queue[i] =
    robot.direction_queue_data.direction_queue[i + 1];
  }

  robot.direction_queue_data.direction_queue[(DIRECTION_QUEUE_SIZE - 1)] =
  protobuf_direction_cmd_t_DIRECTION_CMD_NONE;

  if (robot.direction_queue_data.queue_index > 0)
  {
    robot.direction_queue_data.queue_index--;
  }

  robot.direction_queue_data.next_direction_cmd =
  robot.direction_queue_data.direction_queue[0];

  if (*direct_cmd == protobuf_direction_cmd_t_DIRECTION_CMD_NONE)
  {
    *direct_cmd = protobuf_direction_cmd_t_DIRECTION_CMD_MOVE_FORWARD;
  }

  return;
}

static void robot_control_set_pre_speed(void)
{
  robot_control_max_speed_set(robot.max_speed_temp);
}

/* End of file -------------------------------------------------------- */
