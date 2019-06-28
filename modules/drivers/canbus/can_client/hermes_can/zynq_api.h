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

#include "bcan_defs.h"

#define ZYNQ_MOD_VER "1.6.1.2"

#define ZYNQ_DRV_NAME "zynq"

#define ZYNQ_DEV_NAME_FW "zynq_fw"
#define ZYNQ_DEV_NAME_TRIGGER "zynq_trigger"
#define ZYNQ_DEV_NAME_GPS "zynq_gps"
#define ZYNQ_DEV_NAME_REG "zynq_reg"
#define ZYNQ_DEV_NAME_CAN "zynq_can"

/*
 * ioctl argument definition for CAN send/recv
 */
typedef struct ioc_bcan_msg {
  bcan_msg_t *ioc_msgs;
  unsigned int ioc_msg_num;
  unsigned int ioc_msg_num_done;
  int ioc_msg_err;
  int ioc_msg_rx_clear;
} ioc_bcan_msg_t;

/*
 * CAN error and status
 */
typedef struct ioc_bcan_status_err {
  unsigned int bcan_status;
  unsigned int bcan_err_status;
  unsigned int bcan_err_count;
  int bcan_ioc_err;
} ioc_bcan_status_err_t;

/* ioctl command list */
#define ZYNQ_IOC_MAGIC ('z' << 12 | 'y' << 8 | 'n' << 4 | 'q')
enum ZYNQ_IOC_GPS_CMD { IOC_GPS_GET = 1, IOC_GPS_GPRMC_GET, IOC_GPS_CMD_MAX };

enum ZYNQ_IOC_TRIGGER_CMD {
  IOC_TRIGGER_DISABLE = IOC_GPS_CMD_MAX,
  IOC_TRIGGER_ENABLE_GPS,
  IOC_TRIGGER_ENABLE_NOGPS,
  IOC_TRIGGER_ENABLE_ONE_GPS,
  IOC_TRIGGER_ENABLE_ONE_NOGPS,
  IOC_TRIGGER_TIMESTAMP,
  IOC_TRIGGER_STATUS,
  IOC_TRIGGER_STATUS_GPS,
  IOC_TRIGGER_STATUS_PPS,
  IOC_TRIGGER_FPS_SET,
  IOC_TRIGGER_FPS_GET,
  IOC_TRIGGER_CMD_MAX
};

enum ZYNQ_IOC_FW_CMD {
  IOC_FW_IMAGE_UPLOAD_START = IOC_TRIGGER_CMD_MAX,
  IOC_FW_IMAGE_UPLOAD,
  IOC_FW_PL_UPDATE, /* PL FPGA FW image update */
  IOC_FW_PS_UPDATE, /* PS OS image update */
  IOC_FW_GET_VER,   /* get the image version */
  IOC_FW_CMD_MAX
};

enum ZYNQ_IOC_CAN_CMD {
  IOC_CAN_TX_TIMEOUT_SET = IOC_FW_CMD_MAX, /* in milli-seconds */
  IOC_CAN_RX_TIMEOUT_SET,                  /* in milli-seconds */
  IOC_CAN_DEV_START,
  IOC_CAN_DEV_STOP,
  IOC_CAN_DEV_RESET,
  IOC_CAN_ID_ADD,
  IOC_CAN_ID_DEL,
  IOC_CAN_BAUDRATE_SET,
  IOC_CAN_BAUDRATE_GET,
  IOC_CAN_LOOPBACK_SET,
  IOC_CAN_LOOPBACK_UNSET,
  IOC_CAN_RECV,
  IOC_CAN_SEND,
  IOC_CAN_SEND_HIPRI,
  IOC_CAN_GET_STATUS_ERR,
  IOC_CAN_CMD_MAX
};

enum ZYNQ_IOC_REG_CMD {
  IOC_REG_READ = IOC_CAN_CMD_MAX,
  IOC_REG_WRITE,
  IOC_REG_I2C_READ,
  IOC_REG_I2C_WRITE,
  IOC_REG_GPSPPS_EVENT_WAIT,
  IOC_REG_CMD_MAX
};

enum zynq_baudrate_val {
  ZYNQ_BAUDRATE_1M,
  ZYNQ_BAUDRATE_500K,
  ZYNQ_BAUDRATE_250K,
  ZYNQ_BAUDRATE_150K,
  ZYNQ_BAUDRATE_NUM
};

/* GPS update ioctl cmds */
#define ZYNQ_GPS_VAL_SZ 12
#define ZYNQ_IOC_GPS_GET _IOR(ZYNQ_IOC_MAGIC, IOC_GPS_GET, unsigned char *)
#define ZYNQ_GPS_GPRMC_VAL_SZ 68
#define ZYNQ_IOC_GPS_GPRMC_GET \
  _IOR(ZYNQ_IOC_MAGIC, IOC_GPS_GPRMC_GET, unsigned char *)

/* Trigger ioctl cmds */
#define ZYNQ_IOC_TRIGGER_DISABLE \
  _IOW(ZYNQ_IOC_MAGIC, IOC_TRIGGER_DISABLE, unsigned long)
#define ZYNQ_IOC_TRIGGER_ENABLE_GPS \
  _IOW(ZYNQ_IOC_MAGIC, IOC_TRIGGER_ENABLE_GPS, unsigned long)
#define ZYNQ_IOC_TRIGGER_ENABLE_NOGPS \
  _IOW(ZYNQ_IOC_MAGIC, IOC_TRIGGER_ENABLE_NOGPS, unsigned long)
#define ZYNQ_IOC_TRIGGER_ENABLE_ONE_GPS \
  _IOW(ZYNQ_IOC_MAGIC, IOC_TRIGGER_ENABLE_ONE_GPS, unsigned long)
#define ZYNQ_IOC_TRIGGER_ENABLE_ONE_NOGPS \
  _IOW(ZYNQ_IOC_MAGIC, IOC_TRIGGER_ENABLE_ONE_NOGPS, unsigned long)
#define ZYNQ_IOC_TRIGGER_TIMESTAMP \
  _IOW(ZYNQ_IOC_MAGIC, IOC_TRIGGER_TIMESTAMP, unsigned long)
#define ZYNQ_IOC_TRIGGER_STATUS _IOR(ZYNQ_IOC_MAGIC, IOC_TRIGGER_STATUS, int *)
#define ZYNQ_IOC_TRIGGER_STATUS_GPS \
  _IOR(ZYNQ_IOC_MAGIC, IOC_TRIGGER_STATUS_GPS, int *)
#define ZYNQ_IOC_TRIGGER_STATUS_PPS \
  _IOR(ZYNQ_IOC_MAGIC, IOC_TRIGGER_STATUS_PPS, int *)
#define ZYNQ_IOC_TRIGGER_FPS_SET \
  _IOW(ZYNQ_IOC_MAGIC, IOC_TRIGGER_FPS_SET, int *)
#define ZYNQ_IOC_TRIGGER_FPS_GET \
  _IOW(ZYNQ_IOC_MAGIC, IOC_TRIGGER_FPS_GET, int *)

/* supported GrassHopper fps */
#define GH_FPS_30_DEFAULT 0 /* 30Hz */
#define GH_FPS_20 1         /* 20Hz */
#define GH_FPS_15 2         /* 15Hz */
#define GH_FPS_10 3         /* 10Hz */
/* supported Leopard Imaging fps */
#define LI_FPS_30_DEFAULT 0 /* 30Hz */
#define LI_FPS_20 1         /* 20Hz */
#define LI_FPS_15 2         /* 15Hz */
#define LI_FPS_10 3         /* 10Hz */
/* supported BumbleBee fps */
#define BB_FPS_15_DEFAULT 0 /* 15Hz */
#define BB_FPS_14 1         /* 14Hz */
#define BB_FPS_16 2         /* 16Hz */
/* supported LadyBug fps */
#define LD_FPS_5_DEFAULT 0 /* 5Hz */
#define LD_FPS_7 1         /* 7Hz */
#define LD_FPS_9 2         /* 9Hz */
/* adv_trigger specify fps in format of <GH><LI><BB><LD> */
#define ZYNQ_FPS_GH(fps) ((fps >> 12) & 0xf)
#define ZYNQ_FPS_LI(fps) ((fps >> 8) & 0xf)
#define ZYNQ_FPS_BB(fps) ((fps >> 4) & 0xf)
#define ZYNQ_FPS_LD(fps) (fps & 0xf)
#define ZYNQ_FPS_KEEP 0xf
#define ZYNQ_FPS_KEEP_ALL 0xffff
#define ZYNQ_FPS_ALL_DEFAULT 0
#define ZYNQ_FPS_LI_DEFAULT 0xf0ff
/* Set LI fps only and keep other fps unchanged: 0xf */
#define ZYNQ_FPS_SET_LI_ONLY(li_fps) (0xf0ff | (li_fps << 8))
#define ZYNQ_FPS_VALIDATE_FAIL(fps)                               \
  ((ZYNQ_FPS_GH(fps) > 3 && ZYNQ_FPS_GH(fps) != ZYNQ_FPS_KEEP) || \
   (ZYNQ_FPS_LI(fps) > 3 && ZYNQ_FPS_LI(fps) != ZYNQ_FPS_KEEP) || \
   (ZYNQ_FPS_BB(fps) > 2 && ZYNQ_FPS_BB(fps) != ZYNQ_FPS_KEEP) || \
   (ZYNQ_FPS_LD(fps) > 2 && ZYNQ_FPS_LD(fps) != ZYNQ_FPS_KEEP))

/* FW update ioctl cmds */
#define ZYNQ_FW_PADDING 0x00000000
#define ZYNQ_FW_MSG_SZ 16
typedef struct ioc_zynq_fw_upload {
  /*
   * image data size must be multiple of 4 as each polling transfer is in
   * 16-byte, so padding the data if needed.
   */
  unsigned int *ioc_zynq_fw_data;
  unsigned int ioc_zynq_fw_num;
  unsigned int ioc_zynq_fw_done;
  int ioc_zynq_fw_err;
} ioc_zynq_fw_upload_t;

#define ZYNQ_IOC_FW_IMAGE_UPLOAD_START \
  _IOW(ZYNQ_IOC_MAGIC, IOC_FW_IMAGE_UPLOAD_START, unsigned long)
#define ZYNQ_IOC_FW_IMAGE_UPLOAD \
  _IOW(ZYNQ_IOC_MAGIC, IOC_FW_IMAGE_UPLOAD, ioc_zynq_fw_upload_t *)
#define ZYNQ_IOC_FW_PL_UPDATE \
  _IOW(ZYNQ_IOC_MAGIC, IOC_FW_PL_UPDATE, unsigned long)
#define ZYNQ_IOC_FW_PS_UPDATE \
  _IOW(ZYNQ_IOC_MAGIC, IOC_FW_PS_UPDATE, unsigned long)
#define ZYNQ_IOC_FW_GET_VER _IOW(ZYNQ_IOC_MAGIC, IOC_FW_GET_VER, unsigned int *)

/* CAN channel ioctl cmds */
#define ZYNQ_IOC_CAN_TX_TIMEOUT_SET \
  _IOW(ZYNQ_IOC_MAGIC, IOC_CAN_TX_TIMEOUT_SET, unsigned long)

#define ZYNQ_IOC_CAN_RX_TIMEOUT_SET \
  _IOW(ZYNQ_IOC_MAGIC, IOC_CAN_RX_TIMEOUT_SET, unsigned long)

#define ZYNQ_IOC_CAN_DEV_START \
  _IOW(ZYNQ_IOC_MAGIC, IOC_CAN_DEV_START, unsigned long)

#define ZYNQ_IOC_CAN_DEV_STOP \
  _IOW(ZYNQ_IOC_MAGIC, IOC_CAN_DEV_STOP, unsigned long)

#define ZYNQ_IOC_CAN_DEV_RESET \
  _IOW(ZYNQ_IOC_MAGIC, IOC_CAN_DEV_RESET, unsigned long)

#define ZYNQ_IOC_CAN_ID_ADD _IOW(ZYNQ_IOC_MAGIC, IOC_CAN_ID_ADD, unsigned long)

#define ZYNQ_IOC_CAN_ID_DEL _IOW(ZYNQ_IOC_MAGIC, IOC_CAN_ID_DEL, unsigned long)

#define ZYNQ_IOC_CAN_BAUDRATE_SET \
  _IOW(ZYNQ_IOC_MAGIC, IOC_CAN_BAUDRATE_SET, unsigned long)

#define ZYNQ_IOC_CAN_BAUDRATE_GET \
  _IOW(ZYNQ_IOC_MAGIC, IOC_CAN_BAUDRATE_GET, unsigned long)

#define ZYNQ_IOC_CAN_LOOPBACK_SET \
  _IOW(ZYNQ_IOC_MAGIC, IOC_CAN_LOOPBACK_SET, unsigned long)

#define ZYNQ_IOC_CAN_LOOPBACK_UNSET \
  _IOW(ZYNQ_IOC_MAGIC, IOC_CAN_LOOPBACK_UNSET, unsigned long)

#define ZYNQ_IOC_CAN_RECV _IOWR(ZYNQ_IOC_MAGIC, IOC_CAN_RECV, ioc_bcan_msg_t *)

#define ZYNQ_IOC_CAN_SEND _IOWR(ZYNQ_IOC_MAGIC, IOC_CAN_SEND, ioc_bcan_msg_t *)

#define ZYNQ_IOC_CAN_SEND_HIPRI \
  _IOWR(ZYNQ_IOC_MAGIC, IOC_CAN_SEND_HIPRI, ioc_bcan_msg_t *)

#define ZYNQ_IOC_CAN_GET_STATUS_ERR \
  _IOR(ZYNQ_IOC_MAGIC, IOC_CAN_GET_STATUS_ERR, ioc_bcan_status_err_t *)

/* register read/write ioctl cmds */
typedef struct ioc_zynq_reg_acc {
  unsigned int reg_bar;
  unsigned int reg_offset;
  unsigned int reg_data;
} ioc_zynq_reg_acc_t;

/* I2C ID definitions */
#define ZYNQ_I2C_ID_JANUS 0x5c
#define ZYNQ_I2C_ID_MAX 0x7f /* 7-bit */

typedef struct ioc_zynq_i2c_acc {
  unsigned char i2c_id; /* 7-bit */
  unsigned char i2c_addr;
  unsigned char i2c_data;
} ioc_zynq_i2c_acc_t;

#define ZYNQ_IOC_REG_READ \
  _IOR(ZYNQ_IOC_MAGIC, IOC_REG_READ, ioc_zynq_reg_acc_t *)
#define ZYNQ_IOC_REG_WRITE \
  _IOW(ZYNQ_IOC_MAGIC, IOC_REG_WRITE, ioc_zynq_reg_acc_t *)
#define ZYNQ_IOC_REG_I2C_READ \
  _IOR(ZYNQ_IOC_MAGIC, IOC_REG_I2C_READ, ioc_zynq_i2c_acc_t *)
#define ZYNQ_IOC_REG_I2C_WRITE \
  _IOW(ZYNQ_IOC_MAGIC, IOC_REG_I2C_WRITE, ioc_zynq_i2c_acc_t *)
/* wait for GPS/PPS status change event notification */
#define ZYNQ_IOC_REG_GPSPPS_EVENT_WAIT \
  _IOW(ZYNQ_IOC_MAGIC, IOC_REG_GPSPPS_EVENT_WAIT, unsigned long)
