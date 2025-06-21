/**
 * \file       vl_cbuffer.c
 * \copyright  
 * \license    
 * \version    1.3
 * \date       2024-05-24
 * \author     1.0 - Khang Vo
 * \brief      This Circular Buffer is safe to use in IRQ with single reader,
 *             single write. No need to disable any IRQ.
 */

/* Includes ----------------------------------------------------------------- */
#include "vl_cbuffer.h"
#include "cmsis_compiler.h"


#define CONFIG_VERIFY_ITEM_HEADER    0

/* Private defines ---------------------------------------------------------- */
#define ITEM_HEADER_SIZE    (4)

#if !defined(MIN)
  #define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#if (CONFIG_VERIFY_ITEM_HEADER == 1)
#define MAGIC_BYTE          (0xA5)

typedef union
{
  uint32_t word;
  uint8_t arr[4];
  struct {
    uint16_t length;
    uint8_t magic;
    uint8_t checksum;
  } fields;
}
vl_cbuffer_item_header_t;

static inline uint8_t m_checksum(uint8_t *buf, uint32_t len)
{
  uint8_t checksum = 0;
  for (uint32_t i = 0; i < len; i++)
  {
    checksum ^= buf[i];
  }
  return checksum;
}
#endif // #if (CONFIG_VERIFY_ITEM_HEADER == 1)

/* Private functions prototypes --------------------------------------------- */

/* Public function prototypes ----------------------------------------------- */
/**
 * \brief   Initializes the CB structure with the given buffer and size \n
 *
 * \param[in]    cb         vl_cbuffer Pointer
 * \param[in]    buf        vl_cbuffer Buffer
 * \param[in]    size       size of vl_cbuffer Buffer
 *
 * \return  None
 */
void vl_cbuffer_init(vl_cbuffer_t *cb, void *buf, uint32_t size)
{
  cb->data       = (uint8_t *) buf;
  cb->size       = (size / 4) * 4;    // Make sure size is multiple of 4
  cb->reader     = 0;
  cb->writer     = 0;
  cb->item_count = 0;
  cb->data_count = 0;
  cb->lock       = false;
  cb->active     = true;
}
/**
 * \brief   Reset the CB structure
 *
 * \param[in]    cb         vl_cbuffer Pointer
 *
 * \return  None
 */
void vl_cbuffer_clear(vl_cbuffer_t *cb)
{
  cb->active     = false;
  cb->reader     = 0;
  cb->writer     = 0;
  cb->item_count = 0;
  cb->data_count = 0;
  cb->lock       = false;
  cb->active     = true;
}

/**
 * \brief   Read upto <n_bytes> from the circular buffer
 *          BUT NOT MOVE THE READ POINTER
 *
 * \param[in]    cb         vl_cbuffer Pointer
 * \param[in]    buf        Pointer of read buffer
 *
 * \return  Number of read bytes
 */
uint32_t vl_cbuffer_top(vl_cbuffer_t *cb, void *buf, uint32_t buf_size)
{
  if (!(cb->active) || (cb->lock) || (cb->item_count == 0) || (buf_size == 0))
  {
    return 0;
  }

  __disable_irq();
  cb->lock = true;

  // Read data length at the beginning of the buffer
  uint32_t n_bytes = 0;
  uint32_t tmp_reader = cb->reader;
#if (CONFIG_VERIFY_ITEM_HEADER == 1)
  vl_cbuffer_item_header_t header  = *((vl_cbuffer_item_header_t *) &cb->data[tmp_reader]);
  n_bytes = header.fields.length;
  // Verify the magic byte and checksum
  if ((header.fields.magic != MAGIC_BYTE) || (m_checksum(header.arr, 3) != header.fields.checksum))
  {
    // Checksum is invalid, return 0
    n_bytes = 0;
  }
#else
  n_bytes = *((uint32_t *) &cb->data[tmp_reader]);
#endif

  if (0 < n_bytes && n_bytes <= buf_size && n_bytes <= cb->data_count)
  {
    tmp_reader += ITEM_HEADER_SIZE;

    uint32_t btr, br;
    btr = n_bytes;
    while (btr)
    {
      br = MIN(btr, cb->size - tmp_reader);  // End of linear buffer limit

      memcpy(buf, &(cb->data[tmp_reader]), br);
      buf = (uint8_t *)buf + br;
      btr = btr - br;

      // Check for wrap-around
      tmp_reader = (tmp_reader + br == cb->size) ? 0 : tmp_reader + br;
    }
  }
  else
  {
    // Item length is invalid, return 0
    n_bytes = 0;
  }

  cb->lock = false;
  __enable_irq();

  return n_bytes;
}

/**
 * \brief   Read upto bytes from the CB. \n
 *          Actual number of read bytes is returned
 *
 * \param[in]    cb         vl_cbuffer Pointer
 * \param[in]    buf        Pointer of read buffer
 *
 * \return  Number of read bytes
 */
uint32_t vl_cbuffer_read(vl_cbuffer_t *cb, void *buf, uint32_t buf_size)
{
  if (!(cb->active) || (cb->lock) || (cb->item_count == 0) || (buf_size == 0))
  {
    return 0;
  }

  __disable_irq();
  cb->lock = true;

  // Read data length at the beginning of the buffer
  uint32_t n_bytes = 0;
#if (CONFIG_VERIFY_ITEM_HEADER == 1)
  vl_cbuffer_item_header_t header  = *((vl_cbuffer_item_header_t *) &cb->data[cb->reader]);
  n_bytes = header.fields.length;
  // Verify the magic byte and checksum
  if ((header.fields.magic != MAGIC_BYTE) || (m_checksum(header.arr, 3) != header.fields.checksum))
  {
    // Checksum is invalid, return 0
    n_bytes = 0;
  }
#else
  n_bytes = *((uint32_t *) &cb->data[cb->reader]);
#endif

  if (0 < n_bytes && n_bytes <= buf_size && n_bytes <= cb->data_count)
  {
    cb->reader += ITEM_HEADER_SIZE;

    uint32_t btr, br;
    btr = n_bytes;
    while (btr)
    {
      br = MIN(btr, cb->size - cb->reader); // End of linear buffer limit

      memcpy(buf, &(cb->data[cb->reader]), br);
      buf = (uint8_t *)buf + br;
      btr = btr - br;

      // Check for wrap-around
      cb->reader = (cb->reader + br == cb->size) ? 0 : cb->reader + br;
    }

    // Skip padding bytes
    uint32_t padding_size = 0;
    if ((n_bytes & 3) != 0)    // n_bytes is not multiple of 4
    {
      padding_size = ITEM_HEADER_SIZE - (n_bytes % ITEM_HEADER_SIZE);
    }
    cb->reader = (cb->reader + padding_size) % cb->size;
    cb->item_count--;
    cb->data_count -= (n_bytes + ITEM_HEADER_SIZE + padding_size);
  }
  else
  {
    // Item length is invalid, return 0
    n_bytes = 0;
  }

  cb->lock = false;
  __enable_irq();

  return n_bytes;
}

/**
 *
 * \brief   Write upto n_bytes to the CB. \n
 *          Actual number of written byte is returned.
 *
 * \param[in]    cb         vl_cbuffer Pointer
 * \param[in]    buf        Pointer of write buffer
 * \param[in]    n_bytes     Number of bytes want to write
 *
 * \return  Number of write bytes
 */
uint32_t vl_cbuffer_write(vl_cbuffer_t *cb, void *buf, uint16_t n_bytes)
{
  if (!(cb->active) || (cb->lock) || vl_cbuffer_space_count(cb) < (n_bytes + ITEM_HEADER_SIZE))
  {
    return 0;
  }

  __disable_irq();
  cb->lock = true;

  if (cb->writer <= (cb->size - ITEM_HEADER_SIZE))
  {
    uint32_t *p_wpos = (uint32_t *) &cb->data[cb->writer];
    // Write data length at the beginning of the buffer
#if (CONFIG_VERIFY_ITEM_HEADER == 1)
    vl_cbuffer_item_header_t header;
    header.fields.length = n_bytes;
    header.fields.magic = MAGIC_BYTE;
    header.fields.checksum = m_checksum(header.arr, 3);
    *p_wpos = header.word;
#else
    *p_wpos = n_bytes;
#endif

    cb->writer = (cb->writer + ITEM_HEADER_SIZE) % cb->size;

    uint32_t btw, bw;
    btw = n_bytes;

    while (btw)
    {
      bw = MIN(btw, cb->size - cb->writer); // End of linear buffer limit

      memcpy(&(cb->data[cb->writer]), buf, bw);
      buf = (uint8_t *)buf + bw;
      btw = btw - bw;

      // Check for wrap-around
      cb->writer = (cb->writer + bw == cb->size) ? 0 : cb->writer + bw;
    }

    uint32_t padding_size = 0;
    if ((n_bytes & 3) != 0)  // n_bytes is not multiple of 4
    {
      padding_size = ITEM_HEADER_SIZE - (n_bytes % ITEM_HEADER_SIZE);
    }
    for (uint32_t i = 0; i < padding_size; i++)
    {
      // Write a byte to the internal buffer
      cb->data[cb->writer] = 0xFF;

      // Check for wrap-around
      cb->writer = (cb->writer + 1 == cb->size) ? 0 : cb->writer + 1;
    }

    cb->item_count++;
    cb->data_count += (n_bytes + ITEM_HEADER_SIZE + padding_size);
  }
  else
  {
    // Failed to update writer position, return 0
    n_bytes = 0;
  }

  cb->lock = false;
  __enable_irq();

  return n_bytes;
}

/**
 * \brief   Return number of items in the CB.
 *
 * \param[in]    cb         vl_cbuffer Pointer
 *
 * \return  number of items in the CB.
 */
uint32_t vl_cbuffer_count(vl_cbuffer_t *cb)
{
  return cb->item_count;
}

/**
 * \brief   Return number of unread bytes in the CB.
 *
 * \param[in]    cb         vl_cbuffer Pointer
 *
 * \return  unread bytes in the CB.
 */
uint32_t vl_cbuffer_data_count(vl_cbuffer_t *cb)
{
  if ( !(cb->active) )
    return 0;
  
  return cb->data_count;
}

/**
 * \brief   Return number of free bytes in the CB.
 *
 * \param[in]    cb         vl_cbuffer Pointer
 *
 * \return  free bytes in the CB.
 */
uint32_t vl_cbuffer_space_count(vl_cbuffer_t *cb)
{
  return (cb->size - vl_cbuffer_data_count(cb));
}

/* End of file -------------------------------------------------------------- */
