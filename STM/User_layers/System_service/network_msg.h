/**
 * @file        network_msg.h
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
#ifndef __NETWORK_MSG_H
#define __NETWORK_MSG_H

/* Includes ----------------------------------------------------------- */
#include "common.h"
#include "fuzzy.h"

/* Public defines ----------------------------------------------------- */
#define DATA_BUF_SIZE         (256)
#define NETWORK_FRAME_HDR_LEN (sizeof(network_hdr_t))

/* Public enumerate/structure ----------------------------------------- */
typedef uint8_t network_serial_sof_t;
#define NETWORK_SERIAL_SOF (network_serial_sof_t)(0x55)

typedef uint8_t network_serial_type_t;
#define NETWORK_SERIAL_TYPE_HANDLE  (network_serial_type_t)(0x00)
#define NETWORK_SERIAL_TYPE_FORWARD (network_serial_type_t)(0x01)

typedef uint8_t network_data_device_address_t;
#define NETWORK_DEVICE_ADDR_JETSON (network_data_device_address_t)(0x00)
#define NETWORK_DEVICE_ADDR_STM    (network_data_device_address_t)(0x01)
#define NETWORK_DEVICE_ADDR_PC     (network_data_device_address_t)(0x02)

typedef uint8_t network_data_which_params_t;
#define NETWORK_COORDINATE_TAG              (network_data_which_params_t)(0x00)
#define NETWORK_ROBOT_START_TAG             (network_data_which_params_t)(0x01)
#define NETWORK_ROBOT_STOP_TAG              (network_data_which_params_t)(0x02)
#define NETWORK_ROBOT_FUZZY_COEFFICIENT_TAG (network_data_which_params_t)(0x03)

// +=============================+=============================+=============================+=============================+
// Serial header
typedef struct __attribute__((packed))
{
  network_serial_sof_t  sof;
  network_serial_type_t type;
  uint8_t               len;
} network_serial_hdr_t;
// +=============================+=============================+=============================+=============================+

// +=============================+=============================+=============================+=============================+
// Serial data
//------------------------------+------------------------------+
// Data header
typedef struct __attribute__((packed))
{
  network_data_device_address_t src;
  network_data_device_address_t dst;
} network_data_hdr_t;
//------------------------------+------------------------------+

//------------------------------+------------------------------+
// Data packet params
typedef struct __attribute__((packed))
{
  float lane_center;
} network_coordinate_t;

typedef struct __attribute__((packed))
{
  uint8_t dummy;
} network_robot_start_t;

typedef struct __attribute__((packed))
{
  uint8_t dummy;
} network_robot_stop_t;

typedef struct __attribute__((packed))
{
  fuzzy_5_rule_coef_t          trapezium_e_nor;
  fuzzy_5_rule_coef_t          trapezium_e_dot_nor;
  fuzzy_5_rule_coef_t          trapezium_theta_nor;
  fuzzy_3_positive_coef_rule_t trapezium_velo_nor;
  fuzzy_5_rule_t               theta_y;
  fuzzy_3_positive_rule_t      velo_y;
} network_fuzzy_coef_t;
//------------------------------+------------------------------+

//------------------------------+------------------------------+
// Data packet
typedef struct __attribute__((packed))
{
  network_data_hdr_t hdr;
  uint8_t            which_params;
  union
  {
    network_coordinate_t  coordinate;
    network_robot_start_t robot_start;
    network_robot_stop_t  robot_stop;
    network_fuzzy_coef_t  robot_fz_coef;
  } params;
} network_data_packet_t;
//------------------------------+------------------------------+

// +=============================+=============================+=============================+=============================+
// Network frame
typedef struct __attribute__((packed))
{
  network_serial_hdr_t  serial_hdr;
  network_data_packet_t serial_data;
} network_frame;
// +=============================+=============================+=============================+=============================+

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */

#endif // __NETWORK_MSG_H

/* End of file -------------------------------------------------------- */
