/**
 * \file       vl_cbuffer.h
 * \copyright  
 * \license    
 * \version    1.3
 * \date       2024-05-24
 * \author     1.0 - Khang Vo
 * \brief      This Circular Buffer is safe to use in IRQ with single reader,
 *             single write. No need to disable any IRQ.
 */
#ifndef __VL_CBUFFER_H
#define __VL_CBUFFER_H
#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/* Includes ----------------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Public enumerate/structure ----------------------------------------------- */
/**
 * \brief vl_cbuffer manage handler
 * 
 */
typedef struct
{
  uint8_t *data;      /**< Poiter which data log to */
  uint32_t size;      /**< size of buffer */
  uint32_t writer;    /**< Writing position */
  uint32_t reader;    /**< Reading position */
  uint32_t data_count; /**< Number of written data */
  uint32_t item_count; /**< Number of items in the buffer */
  bool     lock;      /**< Guard condition */
  bool     active;    /**< Guard condition */
  // NOTE: As described in section 6.2.5 of the C99 Standard:
  // 2 An object declared as type _Bool is large enough to store the values 0 and 1.
  // ==> sizeof(bool) = 1 in modern compiler with C standard >= C99
  // ==> Additional padding bytes will be added
  uint8_t  padding[2]; // NOTE: NOT USED
}
vl_cbuffer_t;

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
void vl_cbuffer_init(vl_cbuffer_t *cb, void *buf, uint32_t size);
/**
 * \brief   Reset the CB structure
 *
 * \param[in]    cb         vl_cbuffer Pointer
 *
 * \return  None
 */
void vl_cbuffer_clear(vl_cbuffer_t *cb);

/**
 * \brief   Read upto <nBytes> from the circular buffer
 *          BUT NOT MOVE THE READ POINTER
 *
 * \param[in]    cb         vl_cbuffer Pointer
 * \param[in]    buf        Pointer of read buffer
 * \param[in]    nBytes     Number of bytes want to read
 *
 * \return  Number of read bytes
 */
uint32_t vl_cbuffer_top(vl_cbuffer_t *cb, void *buf, uint32_t buf_size);

/**
 * \brief   Read upto nBytes from the CB. \n
 *          Actual number of read bytes is returned
 *
 * \param[in]    cb         vl_cbuffer Pointer
 * \param[in]    buf        Pointer of read buffer
 * \param[in]    nBytes     Number of bytes want to read
 *
 * \return  Number of read bytes
 */
uint32_t vl_cbuffer_read(vl_cbuffer_t *cb, void *buf, uint32_t buf_size);

/**
 *
 * \brief   Write upto nBytes to the CB. \n
 *          Actual number of written byte is returned.
 *
 * \param[in]    cb         vl_cbuffer Pointer
 * \param[in]    buf        Pointer of write buffer
 * \param[in]    nBytes     Number of bytes want to write
 *
 * \return  Number of write bytes
 */
uint32_t vl_cbuffer_write(vl_cbuffer_t *cb, void *buf, uint16_t bytes);

/**
 * \brief   Return number of items in the CB.
 *
 * \param[in]    cb         vl_cbuffer Pointer
 *
 * \return  number of items in the CB.
 */
uint32_t vl_cbuffer_count(vl_cbuffer_t *cb);

/**
 * \brief   Return number of unread bytes in the CB.
 *
 * \param[in]    cb         vl_cbuffer Pointer
 *
 * \return  unread bytes in the CB.
 */
uint32_t vl_cbuffer_data_count(vl_cbuffer_t *cb);

/**
 * \brief   Return number of free bytes in the CB.
 *
 * \param[in]    cb         vl_cbuffer Pointer
 *
 * \return  free bytes in the CB.
 */
uint32_t vl_cbuffer_space_count(vl_cbuffer_t *cb);

#ifdef __cplusplus
  } // extern "C" {
#endif
#endif // __VL_CBUFFER_H
/* End of file -------------------------------------------------------------- */
