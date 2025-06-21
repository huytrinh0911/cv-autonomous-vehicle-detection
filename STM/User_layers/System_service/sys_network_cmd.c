/**
 * @file        sys_network_cmd.c
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
#include "sys_network_cmd.h"
#include "fuzzy_controller.h"
#include "network_msg.h"
#include "robot_control.h"
#include "sys_dcmotor.h"
#include "platform.h"
#include "bsp_time.h"

// -------------
// #include "protocol.pb.h"

/* Private defines ---------------------------------------------------- */

/* Private enumerate/structure ---------------------------------------- */
/**
 * @brief Function pointer type for send packet  handlers.
 */
typedef void (*cmd_hdl_t)(network_stream_t *ns, const protobuf_network_packet_t *packet);

/**
 * @brief
 *
 */
typedef struct
{
  uint8_t   cmd_id;
  cmd_hdl_t cmd_hdl;
} network_cmd_hdl_t;

/* Private macros ----------------------------------------------------- */
#define CMD_INFO(_cmd_id, _cmd_hdl) \
  [_cmd_id] = { .cmd_id = _cmd_id, .cmd_hdl = _cmd_hdl }

/* Private packet function prototypes ---------------------------------------- */
static void network_cmd_none(network_stream_t *ns, const protobuf_network_packet_t *packet);
static void network_cmd_network_status_get(network_stream_t *ns, const protobuf_network_packet_t *packet);
static void network_cmd_robot_cmd_start(network_stream_t *ns, const protobuf_network_packet_t *packet);
static void network_cmd_robot_cmd_stop(network_stream_t *ns, const protobuf_network_packet_t *packet);
static void network_cmd_fuzzy_coef_set(network_stream_t *ns, const protobuf_network_packet_t *packet);
static void network_cmd_fuzzy_coef_get(network_stream_t *ns, const protobuf_network_packet_t *packet);
static void network_cmd_speed_get(network_stream_t *ns, const protobuf_network_packet_t *packet);
static void network_cmd_speed_max_set(network_stream_t *ns, const protobuf_network_packet_t *packet);
static void network_cmd_speed_max_get(network_stream_t *ns, const protobuf_network_packet_t *packet);
static void network_cmd_time_get(network_stream_t *ns, const protobuf_network_packet_t *packet);
static void network_cmd_time_set(network_stream_t *ns, const protobuf_network_packet_t *packet);
static void network_cmd_robot_state_get(network_stream_t *ns, const protobuf_network_packet_t *packet);
static void network_cmd_traffic_detected(network_stream_t *ns, const protobuf_network_packet_t *packet);
static void network_cmd_vision_coordinate_detected(network_stream_t *ns, const protobuf_network_packet_t *packet);
static void network_cmd_robot_control_cmd(network_stream_t *ns, const protobuf_network_packet_t *packet);
static void network_cmd_vision_start(network_stream_t *ns, const protobuf_network_packet_t *packet);
static void network_cmd_vision_stop(network_stream_t *ns, const protobuf_network_packet_t *packet);
static void network_cmd_robot_angle_get(network_stream_t *ns, const protobuf_network_packet_t *packet);
static void network_cmd_robot_max_angle_get(network_stream_t *ns, const protobuf_network_packet_t *packet);
static void network_cmd_robot_info_get(network_stream_t *ns, const protobuf_network_packet_t *packet);
static void network_cmd_robot_max_angle_set(network_stream_t *ns, const protobuf_network_packet_t *packet);
static void network_cmd_debug_msg(network_stream_t *ns, const protobuf_network_packet_t *packet);
static void network_cmd_robot_direction_cmd(network_stream_t *ns, const protobuf_network_packet_t *packet);
static void network_cmd_robot_signs_info(network_stream_t *ns, const protobuf_network_packet_t *packet);

/* Public variables --------------------------------------------------- */
static const network_cmd_hdl_t network_cmd_hdl[] = {
  //    +=================================================================+======================================================+
  //    | cmd_id                           	                              | Handler                                              |          
  //    |                                                                 |                                                      | 
  //    +-----------------------------------------------------------------+------------------------------------------------------+
  CMD_INFO(protobuf_network_packet_t_none_tag                             ,  network_cmd_none                                   ),
  CMD_INFO(protobuf_network_packet_t_network_ack_tag                      ,  NULL                                               ),
  CMD_INFO(protobuf_network_packet_t_network_status_get_tag               ,  network_cmd_network_status_get                     ),
  CMD_INFO(protobuf_network_packet_t_netowrk_status_rsp_tag               ,  NULL                                               ),
  CMD_INFO(protobuf_network_packet_t_robot_cmd_start_tag                  ,  network_cmd_robot_cmd_start                        ),
  CMD_INFO(protobuf_network_packet_t_robot_cmd_stop_tag                   ,  network_cmd_robot_cmd_stop                         ),
  CMD_INFO(protobuf_network_packet_t_fuzzy_coef_set_tag                   ,  network_cmd_fuzzy_coef_set                         ),
  CMD_INFO(protobuf_network_packet_t_fuzzy_coef_get_tag                   ,  network_cmd_fuzzy_coef_get                         ),
  CMD_INFO(protobuf_network_packet_t_fuzzy_coef_resp_tag                  ,  NULL                                               ),
  CMD_INFO(protobuf_network_packet_t_speed_get_tag                        ,  network_cmd_speed_get                              ),
  CMD_INFO(protobuf_network_packet_t_speed_resp_tag                       ,  NULL                                               ),
  CMD_INFO(protobuf_network_packet_t_speed_max_set_tag                    ,  network_cmd_speed_max_set                          ),
  CMD_INFO(protobuf_network_packet_t_speed_max_get_tag                    ,  network_cmd_speed_max_get                          ),
  CMD_INFO(protobuf_network_packet_t_speed_max_resp_tag                   ,  NULL                                               ),
  CMD_INFO(protobuf_network_packet_t_time_get_tag                         ,  network_cmd_time_get                               ),
  CMD_INFO(protobuf_network_packet_t_time_set_tag                         ,  network_cmd_time_set                               ),
  CMD_INFO(protobuf_network_packet_t_time_rsp_tag                         ,  NULL                                               ),
  CMD_INFO(protobuf_network_packet_t_robot_process_state_get_tag          ,  NULL                                               ),
  CMD_INFO(protobuf_network_packet_t_robot_process_state_set_tag          ,  NULL                                               ),
  CMD_INFO(protobuf_network_packet_t_robot_process_state_rsp_tag          ,  NULL                                               ),
  CMD_INFO(protobuf_network_packet_t_robot_direct_state_get_tag           ,  NULL                                               ),
  CMD_INFO(protobuf_network_packet_t_robot_direct_state_set_tag           ,  NULL                                               ),
  CMD_INFO(protobuf_network_packet_t_robot_direct_state_rsp_tag           ,  NULL                                               ),
  CMD_INFO(protobuf_network_packet_t_robot_state_get_tag                  ,  network_cmd_robot_state_get                        ),
  CMD_INFO(protobuf_network_packet_t_robot_state_rsp_tag                  ,  NULL                                               ),
  CMD_INFO(protobuf_network_packet_t_traffic_detected_tag                 ,  network_cmd_traffic_detected                       ),
  CMD_INFO(protobuf_network_packet_t_vision_coordinate_detected_tag       ,  network_cmd_vision_coordinate_detected             ),
  CMD_INFO(protobuf_network_packet_t_robot_control_cmd_tag                ,  network_cmd_robot_control_cmd                      ),
  CMD_INFO(protobuf_network_packet_t_vision_start_tag                     ,  network_cmd_vision_start                           ),
  CMD_INFO(protobuf_network_packet_t_vision_stop_tag                      ,  network_cmd_vision_stop                            ),
  CMD_INFO(protobuf_network_packet_t_robot_angle_get_tag                  ,  network_cmd_robot_angle_get                        ),
  CMD_INFO(protobuf_network_packet_t_robot_angle_rsp_tag                  ,  NULL                                               ),
  CMD_INFO(protobuf_network_packet_t_robot_max_angle_get_tag              ,  network_cmd_robot_max_angle_get                    ),
  CMD_INFO(protobuf_network_packet_t_robot_max_angle_set_tag              ,  network_cmd_robot_max_angle_set                    ),
  CMD_INFO(protobuf_network_packet_t_robot_max_angle_rsp_tag              ,  NULL                                               ),
  CMD_INFO(protobuf_network_packet_t_robot_info_get_tag                   ,  network_cmd_robot_info_get                         ),
  CMD_INFO(protobuf_network_packet_t_robot_info_rsp_tag                   ,  NULL                                               ),
  CMD_INFO(protobuf_network_packet_t_debug_msg_tag                        ,  network_cmd_debug_msg                              ),
  CMD_INFO(protobuf_network_packet_t_robot_direction_cmd_tag              ,  network_cmd_robot_direction_cmd                    ),
  CMD_INFO(protobuf_network_packet_t_cv_command_tag                       ,  NULL                                               ),
  CMD_INFO(protobuf_network_packet_t_signs_info_tag                       ,  network_cmd_robot_signs_info                       ),
  CMD_INFO(protobuf_network_packet_t_direction_list_tag                   ,  NULL                                               ),
};

static const uint8_t number_of_cmd = sizeof(network_cmd_hdl) / sizeof(network_cmd_hdl_t);

/* Private variables -------------------------------------------------- */

/* Private function prototypes ---------------------------------------- */

/* Function definitions ----------------------------------------------- */
void sys_network_cmd_process(network_stream_t *ns, protobuf_network_packet_t *packet)
{
  if((packet->hdr.addr.dst != LOCAL_DEVICE_ADDR)&&(packet->hdr.addr.dst != protobuf_device_addr_t_DEVICE_ADDR_BCAST))
    return;
   
  if (packet->which_params < number_of_cmd)
    {
      if(network_cmd_hdl[packet->which_params].cmd_hdl != NULL)
      {
        network_cmd_hdl[packet->which_params].cmd_hdl(ns, packet);
      }
    }
}

void sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t interface_dst,
                                    protobuf_message_type_t type,
                                    const char             *format,
                                    ...)
{
  protobuf_network_packet_t p;
  p.which_params                   = protobuf_network_packet_t_debug_msg_tag;
  p.params.debug_msg.message_type  = type;
  p.params.debug_msg.interface_dst = interface_dst;
  va_list args;
  va_start(args, format);
  vsnprintf((char *)&p.params.debug_msg.message.bytes,
            sizeof(p.params.debug_msg.message.bytes), format, args);
  va_end(args);
  p.params.debug_msg.message.size = strlen((char *)&p.params.debug_msg.message.bytes);
  sys_network_send_packet(&p, protobuf_device_addr_t_DEVICE_ADDR_APP);
}

/* Private definitions ----------------------------------------------- */
static void network_cmd_none(network_stream_t *ns, const protobuf_network_packet_t *packet)
{
}

static void network_cmd_network_status_get(network_stream_t *ns,
                                                  const protobuf_network_packet_t *packet)
{
  protobuf_network_packet_t p;
  p.which_params = protobuf_network_packet_t_netowrk_status_rsp_tag;
  if(!ns->is_connected)
  {
    ns->is_connected = true;
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
  }
  ns->connection_cnt = SYS_NETWORK_CONNTECTION_COUNTER_TRACK;
  ns->last_time_connect = HAL_GetTick();
  sys_network_send_packet(&p, packet->hdr.addr.src);

  robot_update_direction_queue();

  return;
}

static void network_cmd_robot_cmd_start(network_stream_t                *ns,
                                        const protobuf_network_packet_t *packet)
{
  sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                 protobuf_message_type_t_MESSAGE_INFO, "Sending start vision process command to TX2.");
  robot_control_start();
  protobuf_network_packet_t p;
  p.which_params = protobuf_network_packet_t_vision_start_tag;
  p.params.vision_start.dummy = 0;
  sys_network_send_packet(&p, protobuf_device_addr_t_DEVICE_ADDR_TX2);

  return;
}

static void network_cmd_robot_cmd_stop(network_stream_t                *ns,
                                       const protobuf_network_packet_t *packet)
{
  robot_control_stop();
  
  protobuf_network_packet_t p;
  p.which_params = protobuf_network_packet_t_vision_stop_tag;
  sys_network_send_packet(&p, protobuf_device_addr_t_DEVICE_ADDR_TX2);

  return;
}

static void network_cmd_fuzzy_coef_set(network_stream_t                *ns,
                                       const protobuf_network_packet_t *packet)
{
  HAL_GPIO_TogglePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin);
  fuzzy_controller_coef_set(
  (protobuf_fuzzy_coef_t *)&packet->params.fuzzy_coef_set.fuzzy_coef);
  sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_PROGRAMMER,
                                 protobuf_message_type_t_MESSAGE_INFO, "Set fuzzy controller coefficient successfully.");
  return;
}

static void network_cmd_fuzzy_coef_get(network_stream_t                *ns,
                                       const protobuf_network_packet_t *packet)
{
  sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_PROGRAMMER,
                                 protobuf_message_type_t_MESSAGE_NOR,
                                 "Getting fuzzy controller coefficient");
  protobuf_network_packet_t p;
  p.which_params = protobuf_network_packet_t_fuzzy_coef_resp_tag;
  p.params.fuzzy_coef_resp.has_fuzzy_coef = true;
  if (fuzzy_controller_coef_get(&p.params.fuzzy_coef_resp.fuzzy_coef))
  {
    sys_network_send_packet(&p, packet->hdr.addr.src);
  }

  return;
}

static void network_cmd_speed_get(network_stream_t *ns, const protobuf_network_packet_t *packet)
{
  protobuf_network_packet_t p;
  p.which_params = protobuf_network_packet_t_speed_resp_tag;
  sys_dcmotor_speed_get(&p.params.speed_resp.speed);
  sys_network_send_packet(&p, packet->hdr.addr.src);

  return;
}

static void network_cmd_speed_max_set(network_stream_t                *ns,
                                      const protobuf_network_packet_t *packet)
{
  robot_control_max_speed_set(packet->params.speed_max_set.speed);
  sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                 protobuf_message_type_t_MESSAGE_INFO,
                                 "Set robot max speed successfully");
}

static void network_cmd_speed_max_get(network_stream_t                *ns,
                                      const protobuf_network_packet_t *packet)
{
}

static void network_cmd_time_get(network_stream_t *ns, const protobuf_network_packet_t *packet)
{
}

static void network_cmd_time_set(network_stream_t *ns, const protobuf_network_packet_t *packet)
{
  bsp_timestamp_set(packet->params.time_set.epoch_time);
  sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_PROGRAMMER,
                                 protobuf_message_type_t_MESSAGE_INFO,
                                 "Sync time successfully.");
}

static void network_cmd_robot_state_get(network_stream_t *ns,
                                               const protobuf_network_packet_t *packet)
{
}

static void network_cmd_traffic_detected(network_stream_t                *ns,
                                      const protobuf_network_packet_t *packet)
{
}

static void network_cmd_vision_coordinate_detected(network_stream_t *ns,
                                                   const protobuf_network_packet_t *packet)
{
  protobuf_vision_coordinate_detected_t coor_p;
  memcpy((uint8_t *)&coor_p, &packet->params.vision_coordinate_detected,
         sizeof(protobuf_vision_coordinate_detected_t));
  robot_control_add_coor(&coor_p);
}

static void network_cmd_robot_control_cmd(network_stream_t *ns,
                                          const protobuf_network_packet_t *packet)
{
}

static void network_cmd_vision_start(network_stream_t                *ns,
                                     const protobuf_network_packet_t *packet)
{
}

static void network_cmd_vision_stop(network_stream_t *ns, const protobuf_network_packet_t *packet)
{
}

static void network_cmd_robot_angle_get(network_stream_t                *ns,
                                        const protobuf_network_packet_t *packet)
{
}

static void network_cmd_robot_max_angle_get(network_stream_t *ns,
                                            const protobuf_network_packet_t *packet)
{
  sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                 protobuf_message_type_t_MESSAGE_INFO,
                                 "Max rotation angle is sent.");
}

static void network_cmd_robot_max_angle_set(network_stream_t *ns,
                                            const protobuf_network_packet_t *packet)
{
  robot_control_max_rotation_angle_set(packet->params.robot_max_angle_set.angle);
  sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                 protobuf_message_type_t_MESSAGE_INFO,
                                 "Set max rotation angle successfully.");
}

static void network_cmd_robot_info_get(network_stream_t                *ns,
                                       const protobuf_network_packet_t *packet)
{
  robot_control_info_get();
}

static void network_cmd_debug_msg(network_stream_t *ns, const protobuf_network_packet_t *packet)
{
}

static void network_cmd_robot_direction_cmd(network_stream_t *ns,
                                            const protobuf_network_packet_t *packet)
{
  sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t_MESSAGE_USER_NOTIF,
                                 protobuf_message_type_t_MESSAGE_INFO,
                                 "Received direction command.");
  robot_add_direction_queue(packet->params.robot_direction_cmd.cmd);
  return;
}

static void network_cmd_robot_signs_info(network_stream_t *ns,
                                         const protobuf_network_packet_t *packet)
{
  protobuf_signs_info_t signs_p;
  memcpy((uint8_t *)&signs_p, &packet->params.signs_info, sizeof(protobuf_signs_info_t));
  robot_control_add_signs(&signs_p);
}


/* End of file -------------------------------------------------------- */
