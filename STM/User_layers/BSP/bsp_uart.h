/**
 * @file       bsp_uart.h
 * @copyright  Copyright (C) 2019 ITRVN. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    0.0.0
 * @date       yyyy-mm-dd
 * @author     <first_name_1> <last_name_1>
 * @author     <first_name_2> <last_name_2>
 * @author     <first_name_3> <last_name_3>
 *
 * @brief      <A brief description of the content of the file>
 *
 * @note
 *
 * @example    example_file_1.c
 *             Example_1 description
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __BSP_UART_H
#define __BSP_UART_H

/* Includes ----------------------------------------------------------- */
#include "common.h"

/* Public defines ----------------------------------------------------- */
#define UART_RX_VL_CBUFFER_SIZE (2048)
#define UART_TX_VL_CBUFFER_SIZE (4096)
#define UART_RX_DMA_BUFF_SIZE   (2048)
#define UART_WRITE_BUFF_SIZE    (1024)

/* Public enumerate/structure ----------------------------------------- */
typedef struct
{
  UART_HandleTypeDef *huart;
  DMA_HandleTypeDef  *dma_huart;
  vl_cbuffer_t        rx_vl_cb;
  vl_cbuffer_t        tx_vl_cb;
  uint8_t             rx_vl_buf[UART_RX_VL_CBUFFER_SIZE];
  uint8_t             tx_vl_buf[UART_TX_VL_CBUFFER_SIZE];
  uint8_t             rx_dma_buf[UART_RX_DMA_BUFF_SIZE];
  uint8_t             tx_write_buf[UART_WRITE_BUFF_SIZE];
} bsp_uart_t;

typedef enum
{
  BSP_UART_ID_1 = 0,
  BSP_UART_ID_2,
  BSP_UART_ID_MAX,
} bsp_uart_identify_t;

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */
/**
 * @brief         Init UART
 *
 * @param[in]     buart         Pointer to bsp_uart_t structure
 * @param[in]     huart         Pointer to UART_HandleTypeDef structure
 *
 * @return
 *  - (0) : Success
 *  - (-1): Error
 */
void bsp_uart_init(UART_HandleTypeDef **huart, DMA_HandleTypeDef **dma_huart, uint8_t number);

/**
 * @brief         Transmit UART
 *
 * @param[in]     buart         Pointer to bsp_uart_t structure
 * @param[in]     buf           Pointer to data buffer
 * @param[in]     size          Size of data buffer
 *
 * @return
 *  - (0) : Success
 *  - (-1): Error
 */
bool bsp_uart_write(UART_HandleTypeDef *huart, uint8_t *buff, uint32_t size);

/**
 * @brief         Receive UART
 *
 * @param[in]     buart         Pointer to bsp_uart_t structure
 * @param[out]    buf           Pointer to data buffer
 * @param[in]     size          Size of data buffer
 *
 * @return
 *  - (-1): Error
 *  - Number of Received bytes, maximum value is BSP_UART_RX_BUFF_SIZE = 32
 */
uint32_t bsp_uart_read(UART_HandleTypeDef *huart, uint8_t *buff, uint32_t buf_size);

void bsp_uart_loop(void);

#endif // __BSP_UART_H

/* End of file -------------------------------------------------------- */
