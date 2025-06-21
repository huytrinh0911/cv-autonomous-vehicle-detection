/**
 * @file        sys_network_cmd.h
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
#ifndef __NETWORK_CMD_H
#define __NETWORK_CMD_H

/* Includes ----------------------------------------------------------- */
#include "common.h"
#include "sys_network.h"
#include "protocol.pb.h"

/* Public defines ----------------------------------------------------- */

/* Public enumerate/structure ----------------------------------------- */


/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */
void sys_network_cmd_process(network_stream_t *ns, protobuf_network_packet_t *packet);
void sys_network_cmd_send_debug_msg(protobuf_message_interface_dst_t interface_dst,
                                    protobuf_message_type_t type,
                                    const char             *format,
                                    ...);
#endif // __NETWORK_CMD_H

/* End of file -------------------------------------------------------- */
