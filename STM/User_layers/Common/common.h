/**
 * @file        common.h
 * @copyright
 * @license
 * @version     0.0.0
 * @date
 * @author
 *
 * @brief       none
 *
 * @note        none
 *
 * @example     none
 *
 */
/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __COMMON_
#define __COMMON_

/* Includes ----------------------------------------------------------- */
#include "cbuffer.h"
#include "vl_cbuffer.h"
#include "platform.h"
#include "pb_common.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "protocol.pb.h"
#include "pb.h"
#include "main.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <float.h>
#include <stdarg.h>

/* Public defines ----------------------------------------------------- */
#define UART_DEBUG_ENABLE 1 /* Enable UART debug */
#define ZERO              0

/* Public enumerate/structure ----------------------------------------- */

/* Public macros ------------------------------------------------------ */
// Assert input parameter
#define ASSERT(_expr_, _rc_) \
  do                         \
  {                          \
    if (!(_expr_))           \
      return (_rc_);         \
  } while (0)

// Check callback function and call that function
#define CALLBACK(_cb_func_, ...) \
  do                             \
  {                              \
    if (((_cb_func_) != NULL))   \
      _cb_func_(__VA_ARGS__);    \
  } while (0)

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */

/* Public function prototypes ----------------------------------------- */

#endif // __COMMON_

/* End of file -------------------------------------------------------- */
