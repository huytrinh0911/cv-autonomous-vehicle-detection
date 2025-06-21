/**
 * @file       bsp_flash.c
 * @copyright
 * @license    This project is released under the Fiot License.
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
#include "bsp_flash.h"
#include "protocol.pb.h"

/* Private defines ---------------------------------------------------- */
#define FLASH_SECTOR6_ADDR   (0x08040000)
#define FLASH_SECTOR7_ADDR   (0x08060000)
#define FLASH_WRITE_BUF_SIZE (1024 / 4)

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  protobuf_fuzzy_coef_t fuzzy_config;
  uint8_t               checksum;
} bsp_flash_fuzzy_config_t;

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */

/* Private function prototypes ---------------------------------------- */
static uint8_t bsp_flash_checksum(uint8_t *buf, uint16_t len);

/* Function definitions ----------------------------------------------- */
bool bsp_flash_fuzzy_read(protobuf_fuzzy_coef_t *fuzzy_config)
{
    bsp_flash_fuzzy_config_t flash_fzy_cfg;
    memcpy((uint8_t *)&flash_fzy_cfg, (uint8_t *)FLASH_SECTOR6_ADDR, sizeof(bsp_flash_fuzzy_config_t));
    uint8_t checksum = bsp_flash_checksum((uint8_t *)&flash_fzy_cfg.fuzzy_config, sizeof(protobuf_fuzzy_coef_t));
    if (checksum != flash_fzy_cfg.checksum)
    {
        return false;
    }
    memcpy((uint8_t *)fuzzy_config, (uint8_t *)&flash_fzy_cfg.fuzzy_config, sizeof(protobuf_fuzzy_coef_t));
    return true;
}

bool bsp_flash_fuzzy_write(protobuf_fuzzy_coef_t *fuzzy_config)
{
  __disable_irq();
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR);
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t               SectorError;

  EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.Sector       = FLASH_SECTOR_6;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Banks        = FLASH_BANK_1;
  EraseInitStruct.NbSectors    = 1;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  {
    HAL_FLASH_Lock();
    __enable_irq();
    return false;
  }

  bsp_flash_fuzzy_config_t flash_fzy_cfg;
  memcpy((uint8_t *)&flash_fzy_cfg.fuzzy_config, (uint8_t *)fuzzy_config,
         sizeof(protobuf_fuzzy_coef_t));
  flash_fzy_cfg.checksum =
  bsp_flash_checksum((uint8_t *)fuzzy_config, sizeof(protobuf_fuzzy_coef_t));

  uint32_t write_buf[FLASH_WRITE_BUF_SIZE] = { 0 };
  uint16_t len = (sizeof(bsp_flash_fuzzy_config_t) + 3) / 4;
  memcpy((uint8_t *)write_buf, (uint8_t *)&flash_fzy_cfg, sizeof(bsp_flash_fuzzy_config_t));

  for (uint16_t i = 0; i < len; i++)        // write 4 byte each time
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (FLASH_SECTOR6_ADDR + (i * 4)),
                          *(write_buf + i)) != HAL_OK)
    {
      HAL_FLASH_Lock();
      __enable_irq();
      return false;
    }
  }

  HAL_FLASH_Lock();
  __enable_irq();
  return true;
}

/* Private definitions ----------------------------------------------- */
static uint8_t bsp_flash_checksum(uint8_t *buf, uint16_t len)
{
  unsigned int sum; // nothing gained in using smaller types!
  for (sum = 0; len != 0; len--)
    sum += *(buf++); // parenthesis not required!
  return (uint8_t)sum;
}

/* End of file -------------------------------------------------------- */
