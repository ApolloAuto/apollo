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

#ifndef __KERNEL__
#include <sys/time.h>
#else
#include <linux/time.h>
#endif

/*
 * Baidu CAN message definition
 */
typedef struct bcan_msg {
  unsigned int bcan_msg_id;       /* source CAN node id */
  unsigned char bcan_msg_datalen; /* message data len */
  unsigned char bcan_msg_rsv[3];
  unsigned char bcan_msg_data[8]; /* message data */
  struct timeval bcan_msg_timestamp;
} bcan_msg_t;

/*
 * CAN error code
 */
enum bcan_err_code {
  BCAN_PARAM_INVALID = -12,
  BCAN_HDL_INVALID,
  BCAN_DEV_INVALID,
  BCAN_DEV_ERR,
  BCAN_DEV_BUSY,
  BCAN_TIMEOUT,
  BCAN_FAIL,
  BCAN_NOT_SUPPORTED,
  BCAN_NOT_IMPLEMENTED,
  BCAN_INVALID,
  BCAN_NO_BUFFERS,
  BCAN_ERR,
  BCAN_OK, /* 0 */
  BCAN_PARTIAL_OK
};
