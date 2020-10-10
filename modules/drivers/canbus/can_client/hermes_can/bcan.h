/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <sys/ioctl.h>
#include <sys/types.h>
#include <cstdint>
#include <cstdlib>

/* bcan_msg_t and bcan_err_code definitions. */
// #include "linux/bcan_defs.h"
#include "modules/drivers/canbus/can_client/hermes_can/bcan_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BCAN_MAX_TX_MSG 256
#define BCAN_MAX_RX_MSG 256

// Channel states
#define BCAN_DEV_UNINIT -1
#define BCAN_DEV_OPEN (1 << 0)
#define BCAN_DEV_CLOSE (1 << 1)
#define BCAN_DEV_BAUD_SET (1 << 2)
#define BCAN_DEV_NORMAL (1 << 3)
#define BCAN_DEV_LOOPBACK (1 << 4)
#define BCAN_DEV_CONFIG (1 << 5)
#define BCAN_DEV_START (1 << 6)
#define BCAN_DEV_STOP (1 << 7)
#define BCAN_DEV_ACTIVE (1 << 8)
#define BCAN_DEV_RECVD (1 << 9)

typedef uint64_t bcan_hdl_t;

enum bcan_baudrate_val {
  BCAN_BAUDRATE_1M,
  BCAN_BAUDRATE_500K,
  BCAN_BAUDRATE_250K,
  BCAN_BAUDRATE_150K,
  BCAN_BAUDRATE_NUM
};

/* Returns bcan library version. */
const char *bcan_get_libversion(void);

/* Returns detailed bcan library build info. */
const char *bcan_bld_info(void);

/* Returns brief bcan library build info. */
const char *bcan_bld_info_short(void);

/* Returns error message corresponding to the given error code. */
const char *bcan_get_err_msg(int err_code);

int bcan_open(uint32_t dev_index, uint32_t flags, uint64_t tx_to,
              uint64_t rx_to, bcan_hdl_t *hdl);
int bcan_close(bcan_hdl_t hdl);
int bcan_start(bcan_hdl_t hdl);
int bcan_stop(bcan_hdl_t hdl);

int bcan_set_loopback(bcan_hdl_t hdl);
int bcan_unset_loopback(bcan_hdl_t hdl);
int bcan_set_baudrate(bcan_hdl_t hdl, uint32_t rate);
int bcan_get_baudrate(bcan_hdl_t hdl, uint32_t *rate);
int bcan_recv(bcan_hdl_t hdl, bcan_msg_t *buf, uint32_t num_msg);
int bcan_send(bcan_hdl_t hdl, bcan_msg_t *buf, uint32_t num_msg);
int bcan_send_hi_pri(bcan_hdl_t hdl, bcan_msg_t *buf);
int bcan_get_status(bcan_hdl_t hdl);
int bcan_get_err_counter(bcan_hdl_t hdl, uint8_t *rx_err, uint8_t *tx_err);

/* The following APIs are not implemented yet. { */
int bcan_id_add(bcan_hdl_t hdl, uint32_t id_start, uint32_t id_end);
int bcan_id_add_all(bcan_hdl_t hdl);
int bcan_id_remove(bcan_hdl_t hdl, uint32_t id_start, uint32_t id_end);
int bcan_id_remove_all(bcan_hdl_t hdl);
/* } Not implemented. */

#ifdef __cplusplus
}
#endif
