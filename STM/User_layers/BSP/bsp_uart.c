/**
 * @file       bsp_uart.c
 * @copyright
 * @license    This project is released under the Fiot License.
 * @version    0.0.0
 * @date       2025-02-16
 * @author     Khoi Nguyen Thanh

 *
 * @brief      <A brief description of the content of the file>
 *
 * @note
 *
 * @example    example_file_1.c
 *             Example_1 description
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_uart.h"
#include "uart.h"

/* Private defines ---------------------------------------------------- */

/* Private enumerate/structure ---------------------------------------- */

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */
static bsp_uart_t buart[BSP_UART_ID_MAX];

/* Private function prototypes ---------------------------------------- */
static void bsp_uart_rx_callback(UART_HandleTypeDef *huart, uint16_t size);

/* Function definitions ----------------------------------------------- */
void bsp_uart_init(UART_HandleTypeDef **huart, DMA_HandleTypeDef **dma_huart, uint8_t number)
{

  for (int i = 0; i < number; i++)
  {

    buart[i].huart     = huart[i];
    buart[i].dma_huart = dma_huart[i];
    vl_cbuffer_init(&buart[i].rx_vl_cb, buart[i].rx_vl_buf, UART_RX_VL_CBUFFER_SIZE);
    vl_cbuffer_init(&buart[i].tx_vl_cb, buart[i].tx_vl_buf, UART_TX_VL_CBUFFER_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(buart[i].huart, buart[i].rx_dma_buf, UART_RX_DMA_BUFF_SIZE);
    __HAL_DMA_DISABLE_IT(buart[i].dma_huart, DMA_IT_HT);
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  bsp_uart_rx_callback(huart, Size);
}

uint32_t bsp_uart_read(UART_HandleTypeDef *huart, uint8_t *buff, uint32_t buf_size)
{
  uint32_t data_cnt = 0;
  uint32_t len      = 0;

  for (int i = 0; i < BSP_UART_ID_MAX; i++)
  {
    if (huart == buart[i].huart)
    {
      data_cnt = vl_cbuffer_count(&buart[i].rx_vl_cb);
      if (data_cnt)
      {
        len = vl_cbuffer_read(&buart[i].rx_vl_cb, buff, buf_size);
      }
    }
  }
  return len;
}

bool bsp_uart_write(UART_HandleTypeDef *huart, uint8_t *buff, uint32_t size)
{
  for (int i = 0; i < BSP_UART_ID_MAX; i++)
  {
    if (huart == buart[i].huart)
    {
      vl_cbuffer_write(&buart[i].tx_vl_cb, buff, size);
    }
  }
  return true;
}

void bsp_uart_loop(void)
{
  for (int i = 0; i < BSP_UART_ID_MAX; i++)
  {
    uint32_t data_cnt;
    data_cnt = vl_cbuffer_count(&buart[i].tx_vl_cb);
    if (data_cnt)
    {
      if ((buart[i].huart->gState == HAL_UART_STATE_READY) &&
          (__HAL_UART_GET_FLAG(buart[i].huart, UART_FLAG_TC)) &&
          (buart[i].huart->hdmatx->State == HAL_DMA_STATE_READY))
      {
        uint32_t len = 0;
        len = vl_cbuffer_read(&buart[i].tx_vl_cb, buart[i].tx_write_buf, UART_WRITE_BUFF_SIZE);
        HAL_UART_Transmit_DMA(buart[i].huart, buart[i].tx_write_buf, len);
      }
    }
  }
}


/* Private definitions ----------------------------------------------- */
static void bsp_uart_rx_callback(UART_HandleTypeDef *huart, uint16_t size)
{
  for (int i = 0; i < BSP_UART_ID_MAX; i++)
  {
    if (huart == buart[i].huart)
    {
      vl_cbuffer_write(&buart[i].rx_vl_cb, buart[i].rx_dma_buf, size);
      HAL_UARTEx_ReceiveToIdle_DMA(buart[i].huart, buart[i].rx_dma_buf, UART_RX_DMA_BUFF_SIZE);
      return;
    }
  }
}


/* End of file -------------------------------------------------------- */
