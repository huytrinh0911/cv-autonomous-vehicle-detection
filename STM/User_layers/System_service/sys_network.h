/**
 * @file        sys_network.h
 * @copyrightk
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
#ifndef __NETWORK__H
#define __NETWORK__H

/* Includes ----------------------------------------------------------- */
#include "common.h"

/* Public defines ----------------------------------------------------- */
#define SYS_NETWORK_RX_MAX_LEN                (1024)
#define SYS_NETWORK_CONNTECTION_COUNTER_TRACK (4)

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  NET_INTERFACE_JETSON,
  NET_INTERFACE_BLUETOOTH,
  NET_INTERFACE_MAX
} network_stream_interface;

typedef struct
{
  network_stream_interface stream_interface;
  UART_HandleTypeDef      *huart;
  DMA_HandleTypeDef       *hdma_uart_rx;
  DMA_HandleTypeDef       *hdma_uart_tx;
  uint8_t                  rx_raw_packet[SYS_NETWORK_RX_MAX_LEN];
  uint16_t                 rx_raw_packet_len;
  uint16_t                 rx_raw_packet_expect_len;
  uint16_t                 rx_raw_packet_read_len;
  bool                     is_connected;
  uint8_t                  connection_cnt;
  uint64_t                 last_time_connect;
  uint64_t                 duration_from_last_time_connect;
} network_stream_t;


/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */
void sys_network_init(void);
void sys_network_process(void);
void sys_network_send_packet(protobuf_network_packet_t *packet, protobuf_device_addr_t dst);

#endif // __NETWORK__H

/* End of file -------------------------------------------------------- */
