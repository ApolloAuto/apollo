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

#include "linux/zynq_api.h"

#ifdef DEBUG
#define BLOG_DBG0(s...) syslog(LOG_DEBUG, s);
#else
#define BLOG_DBG0(s...) \
  do {                  \
  } while (0);
#endif
#define BLOG_ERR(s...) syslog(LOG_ERR, s);

typedef uint64_t bcan_hdl_t;

#define BCAN_MAX_TX_MSG 256
#define BCAN_MAX_RX_MSG 256

typedef struct bcan_ihdl {
  int dev_index;
  int dev_state;
  int fd;
  uint32_t baudrate;
  uint32_t tx_to;
  uint32_t rx_to;
} bcan_ihdl_t;

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
