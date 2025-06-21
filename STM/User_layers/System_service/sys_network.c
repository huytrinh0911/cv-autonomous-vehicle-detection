/**
 * @file        sys_network.c
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
#include "sys_network.h"
#include "platform.h"
#include "sys_network_cmd.h"
#include "sys_uart.h"
#include "uart.h"
#include "bsp_time.h"

/* Private defines ---------------------------------------------------- */
#define BUFF_SIZE           (1024)
#define CONNECTION_TIME_OUT (250) // ms

/* Private enumerate/structure ---------------------------------------- */

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */
static network_stream_t network_stream[NET_INTERFACE_MAX];

/* Private function prototypes ---------------------------------------- */
static void network_stream_process(network_stream_t *ns);
static bool network_encode_packet(protobuf_network_packet_t *encode_msg,
                                  uint8_t                   *buff,
                                  uint16_t                   buff_len,
                                  uint16_t                  *len);
static bool network_decode_packet(const uint8_t             *buff,
                                       uint16_t                   len,
                                       protobuf_network_packet_t *decode_msg);
static void network_forward_packet(network_stream_t *ns, protobuf_network_packet_t *packet);
static void network_stream_check_connection(network_stream_t *ns);

/* Function definitions ----------------------------------------------- */
void sys_network_init(void)
{
  memset((uint8_t *)&network_stream[NET_INTERFACE_JETSON], 0, sizeof(network_stream_t));
  network_stream[NET_INTERFACE_JETSON].stream_interface   = NET_INTERFACE_JETSON;
  network_stream[NET_INTERFACE_JETSON].huart              = &HUART_JETSON;
  network_stream[NET_INTERFACE_JETSON].hdma_uart_rx       = &HDMA_UART_RX_JETSON;
  network_stream[NET_INTERFACE_JETSON].hdma_uart_rx       = &HDMA_UART_TX_JETSON;
  network_stream[NET_INTERFACE_JETSON].is_connected       = false;


  memset((uint8_t *)&network_stream[NET_INTERFACE_BLUETOOTH], 0, sizeof(network_stream_t));
  network_stream[NET_INTERFACE_BLUETOOTH].stream_interface  = NET_INTERFACE_BLUETOOTH;
  network_stream[NET_INTERFACE_BLUETOOTH].huart             = &HUART_BLUETOOTH;
  network_stream[NET_INTERFACE_BLUETOOTH].hdma_uart_rx      = &HDMA_UART_RX_BLUETOOTH;
  network_stream[NET_INTERFACE_BLUETOOTH].hdma_uart_rx      = &HDMA_UART_TX_BLUETOOTH;
  network_stream[NET_INTERFACE_BLUETOOTH].is_connected      = false;
}

void sys_network_process(void)
{
  for (int i = 0; i < NET_INTERFACE_MAX; i++)
  {
    network_stream_check_connection(&network_stream[i]);
    network_stream_process(&network_stream[i]);
  }
}

void sys_network_send_packet(protobuf_network_packet_t *packet, protobuf_device_addr_t dst)
{
  packet->has_hdr        = true;
  packet->hdr.has_addr   = true;
  packet->hdr.addr.dst   = dst;
  packet->hdr.addr.src   = LOCAL_DEVICE_ADDR;
  packet->hdr.epoch_time = bsp_timestamp_get();

  if ((packet->hdr.addr.dst == protobuf_device_addr_t_DEVICE_ADDR_TX2) &&
      (!network_stream[NET_INTERFACE_JETSON].is_connected))
    return;

  if ((packet->hdr.addr.dst == protobuf_device_addr_t_DEVICE_ADDR_APP) &&
      (!network_stream[NET_INTERFACE_BLUETOOTH].is_connected))
    return;

  uint8_t  buff[BUFF_SIZE];
  uint16_t len = 0;
  if (!network_encode_packet(packet, buff, BUFF_SIZE, &len))
    return;

  switch (packet->hdr.addr.dst)
  {
  case protobuf_device_addr_t_DEVICE_ADDR_APP:
  {
    sys_uart_write(&HUART_BLUETOOTH, buff, (uint32_t)len);
    break;
  }
  case protobuf_device_addr_t_DEVICE_ADDR_TX2:
  {
    sys_uart_write(&HUART_JETSON, buff, (uint32_t)len);
    break;
  }
  default:
  {
    break;
  }
  }
  return;
}

/* Private definitions ----------------------------------------------- */
bool network_encode_packet(protobuf_network_packet_t *encode_msg,
                                uint8_t                   *buff,
                                uint16_t                   buff_len,
                                uint16_t                  *len)
{
  bool status = false;

  /* Create a stream that will write to our buff. */
  pb_ostream_t stream = pb_ostream_from_buffer(buff, buff_len);

  /* Now we are ready to encode the msg! */
  status = pb_encode(&stream, protobuf_network_packet_t_fields, encode_msg);
  *len   = stream.bytes_written;

  return status;
}

bool network_decode_packet(const uint8_t *buff, uint16_t len, protobuf_network_packet_t *decode_msg)
{
  bool status = false;

  /* Create a stream that reads from the buff. */
  pb_istream_t stream = pb_istream_from_buffer(buff, len);

  /* Now we are ready to decode the msg. */
  status = pb_decode(&stream, protobuf_network_packet_t_fields, decode_msg);

  return status;
}

static void network_stream_process(network_stream_t *ns)
{
  if (!sys_uart_read(ns->huart, ns->rx_raw_packet, &ns->rx_raw_packet_len, &ns->rx_raw_packet_expect_len, &ns->rx_raw_packet_read_len))
    return;

  protobuf_network_packet_t rx_protobuf_packet;
  if (!network_decode_packet(ns->rx_raw_packet, ns->rx_raw_packet_len, &rx_protobuf_packet))
    return;

  sys_network_cmd_process(ns, &rx_protobuf_packet);
  network_forward_packet(ns, &rx_protobuf_packet);
  return;
}

static void network_stream_check_connection(network_stream_t *ns)
{
  if (!ns->is_connected)
    return;

  ns->duration_from_last_time_connect = HAL_GetTick() - ns->last_time_connect;
  if (ns->duration_from_last_time_connect > CONNECTION_TIME_OUT)
  {
    ns->connection_cnt--;
    ns->last_time_connect = HAL_GetTick();
    if (ns->connection_cnt == 0)
    {
      ns->is_connected = false;

      if (ns->stream_interface == NET_INTERFACE_BLUETOOTH)
      {
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
      }
    }
  }

  return;
}

static void network_forward_packet(network_stream_t *ns, protobuf_network_packet_t *packet)
{
  if ((packet->hdr.addr.src == packet->hdr.addr.dst) || (packet->hdr.addr.dst == LOCAL_DEVICE_ADDR))
    return;

  HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);

  if ((packet->hdr.addr.src == protobuf_device_addr_t_DEVICE_ADDR_TX2) &&
      (packet->which_params == protobuf_network_packet_t_netowrk_status_rsp_tag))
  {
    if (!network_stream[NET_INTERFACE_JETSON].is_connected)
    {
      network_stream[NET_INTERFACE_JETSON].is_connected = true;
    }
    network_stream[NET_INTERFACE_JETSON].connection_cnt = SYS_NETWORK_CONNTECTION_COUNTER_TRACK;
    network_stream[NET_INTERFACE_JETSON].last_time_connect = HAL_GetTick();
  }

  switch (packet->hdr.addr.dst)
  {
  case protobuf_device_addr_t_DEVICE_ADDR_APP:
  {
    sys_uart_write(&HUART_BLUETOOTH, ns->rx_raw_packet, ns->rx_raw_packet_len);
    break;
  }
  case protobuf_device_addr_t_DEVICE_ADDR_TX2:
  {
    sys_uart_write(&HUART_JETSON, ns->rx_raw_packet, ns->rx_raw_packet_len);
    break;
  }
  case protobuf_device_addr_t_DEVICE_ADDR_BCAST:
  {
    if (packet->hdr.addr.src == protobuf_device_addr_t_DEVICE_ADDR_TX2)
    {
      sys_uart_write(&HUART_BLUETOOTH, ns->rx_raw_packet, ns->rx_raw_packet_len);
    }
    else if (packet->hdr.addr.src == protobuf_device_addr_t_DEVICE_ADDR_APP)
    {
      sys_uart_write(&HUART_JETSON, ns->rx_raw_packet, ns->rx_raw_packet_len);
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

/* End of file -------------------------------------------------------- */
