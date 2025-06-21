/**
 * @file        sys_uart.c
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
#include "sys_uart.h"
#include "common.h"
#include "platform.h"
#include "sys_network.h"
#include "uart.h"

/* Private defines ---------------------------------------------------- */
#define SYS_UART_BUFF_SIZE (1024)

/* Private enumerate/structure ---------------------------------------- */

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */

/* Private function prototypes ---------------------------------------- */
uint8_t sys_uart_checksum(uint8_t *buf, uint16_t len);

/* Function definitions ----------------------------------------------- */
void sys_uart_init(void)
{
  UART_HandleTypeDef *huart[BSP_UART_ID_MAX] = { &HUART_JETSON, &HUART_BLUETOOTH };
  DMA_HandleTypeDef *dma_huart[BSP_UART_ID_MAX] = { &HDMA_UART_RX_JETSON, &HDMA_UART_RX_BLUETOOTH };
  bsp_uart_init(huart, dma_huart, BSP_UART_ID_MAX);
}

void sys_uart_loop(void)
{
  bsp_uart_loop();
}


bool sys_uart_read(UART_HandleTypeDef *huart, uint8_t *buff, uint16_t *len, uint16_t *expect_len, uint16_t *read_len)
{
  uint8_t uart_raw_frame[SYS_UART_BUFF_SIZE];
  memset(uart_raw_frame, 0, SYS_UART_BUFF_SIZE);
  uint16_t raw_len = 0;
  raw_len          = bsp_uart_read(huart, uart_raw_frame, SYS_UART_BUFF_SIZE);

  if (!raw_len)
    return false;

  uart_hdr_frame_t *hdr_ptr = (uart_hdr_frame_t *)uart_raw_frame;

  if (hdr_ptr->sof == UART_SOF)
  {
    *expect_len = hdr_ptr->len + sizeof(uart_hdr_frame_t) + sizeof(uint8_t);

    if(*expect_len > SYS_NETWORK_RX_MAX_LEN)
    {
      *expect_len = 0;
      return false;
    }

    if (*expect_len == raw_len) // The packet is not split into multiple packets.
    {
      uint8_t check_sum = 0;
      check_sum = sys_uart_checksum(uart_raw_frame, (raw_len - sizeof(uint8_t)));
      uint8_t check_sum_recv = uart_raw_frame[raw_len - sizeof(uint8_t)];
      if (check_sum == check_sum_recv)
      {
        memcpy(buff, &uart_raw_frame[sizeof(uart_hdr_frame_t)], hdr_ptr->len);
        *len = hdr_ptr->len;
        return true;
      }
      else
      {
        *expect_len = 0;
        *read_len   = 0;
        return false;
      }
    }
    else
    {
      // The probability of skipping a data frame exists, but it is extremely low.
      memcpy(buff, uart_raw_frame, raw_len); // max = 126 bytes (can be changed)
      *read_len = raw_len;
      return false;
    }
  }

  if ((*expect_len) == 0)
    return false;

  *read_len += raw_len;
  if ((*read_len) < (*expect_len))
  {
    memcpy(&buff[(*read_len - raw_len)], uart_raw_frame, raw_len);
    return false;
  }
  else if ((*read_len) > (*expect_len))
  {
    *read_len   = 0;
    *expect_len = 0;
    return false;
  }

  // (*read_len) == (*expect_len)
  memcpy(&buff[(*read_len - raw_len)], uart_raw_frame, raw_len);
  uint8_t check_sum = 0;
  check_sum = sys_uart_checksum(buff, (*expect_len - sizeof(uint8_t)));
  uint8_t check_sum_recv = buff[*expect_len - sizeof(uint8_t)];
  if (check_sum != check_sum_recv)
  {
    *read_len   = 0;
    *expect_len = 0;
    return false;
  }

  uint32_t encoded_packet_len = *expect_len - sizeof(uart_hdr_frame_t) - sizeof(uint8_t);
  memmove(buff, &buff[sizeof(uart_hdr_frame_t)], encoded_packet_len);
  *len = encoded_packet_len;
  return true;
}

bool sys_uart_write(UART_HandleTypeDef *huart, uint8_t *buff, uint32_t size)
{
  uint8_t           uart_raw_frame[2 * SYS_UART_BUFF_SIZE];   // 4 * 256 = 1024 bytes
  uart_hdr_frame_t *hdr_ptr   = (uart_hdr_frame_t *)uart_raw_frame;
  uint8_t           check_sum = 0;

  hdr_ptr->sof = UART_SOF;
  hdr_ptr->len = size;
  memcpy(&uart_raw_frame[sizeof(uart_hdr_frame_t)], buff, size);
  check_sum = sys_uart_checksum(uart_raw_frame, (size + sizeof(uart_hdr_frame_t)));
  memcpy(&uart_raw_frame[(sizeof(uart_hdr_frame_t) + size)], &check_sum, sizeof(uint8_t));

  uint16_t len = sizeof(uart_hdr_frame_t) + size + sizeof(uint8_t);
  if (bsp_uart_write(huart, uart_raw_frame, len))
    return true;

  return false;
}

/* Private definitions ----------------------------------------------- */
uint8_t sys_uart_checksum(uint8_t *buf, uint16_t len)
{
  unsigned int sum; // nothing gained in using smaller types!
  for (sum = 0; len != 0; len--)
    sum += *(buf++); // parenthesis not required!
  return (uint8_t)sum;
}

/* End of file -------------------------------------------------------- */
