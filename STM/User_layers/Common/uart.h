/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    uart.h
 * @brief   This file contains all the function prototypes for
 *          the uart.c file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_H__
#define __UART_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

  /* USER CODE BEGIN Includes */
  extern UART_HandleTypeDef huart5;
  extern UART_HandleTypeDef huart1;
  extern UART_HandleTypeDef huart3;
  extern DMA_HandleTypeDef  hdma_uart5_rx;
  extern DMA_HandleTypeDef  hdma_uart5_tx;
  extern DMA_HandleTypeDef  hdma_usart3_rx;
  extern DMA_HandleTypeDef  hdma_usart3_tx;

  /* USER CODE END Includes */

  /* Public defines ----------------------------------------------------- */
/* USER CODE BEGIN Private defines */
#define HUART_JETSON        huart3
#define HDMA_UART_RX_JETSON hdma_usart3_rx
#define HDMA_UART_TX_JETSON hdma_usart3_tx

#define HUART_BLUETOOTH        huart5
#define HDMA_UART_RX_BLUETOOTH hdma_uart5_rx
#define HDMA_UART_TX_BLUETOOTH hdma_uart5_tx

/*
USART1_RX  - PA10
USART1_TX  - PA9

USART3_RX  - PB11
USART3_TX  - PB10

UART5_RX  - PD2
UART5_TX  - PC12
*/

  /* USER CODE END Private defines */

  /* Public function prototypes ----------------------------------------- */
  /* USER CODE BEGIN Prototypes */

  /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __UART_H__ */
