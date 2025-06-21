/**
 * @file        sys_uart.h
 * @copyright
 * @license
 * @version     0.0.0
 * @date
 * @author      Minh Phung Nhat
 *              Hung Pham Duc
 *              Khoi Nguyen Thanh
 * @brief       none
 *
 * @note        none
 *
 * @example     none
 *
 */
/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __SYS_UART_H
#define __SYS_UART_H

/* Includes ----------------------------------------------------------- */
#include "bsp_uart.h"
#include "common.h"
#include "network_msg.h"

/* Public defines ----------------------------------------------------- */
#define UART_SOF  (0xAA55CDEF)

/* Public enumerate/structure ----------------------------------------- */
typedef struct
{
  uint32_t sof;
  uint32_t len;
} uart_hdr_frame_t;

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */

/* Public function prototypes ----------------------------------------- */

void sys_uart_init(void);
bool sys_uart_read(UART_HandleTypeDef *huart, uint8_t *buff, uint16_t *len, uint16_t *expect_len, uint16_t *read_len);
bool sys_uart_write(UART_HandleTypeDef *huart, uint8_t *buff, uint32_t size);
void sys_uart_loop(void);

#endif // __SYS_UART_H

/* End of file -------------------------------------------------------- */
