/************************************************************************
*
* Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
*
************************************************************************/

/**
* @file bcan.h
*
* Baidu ADV (Autonomous Driving Vehicle) bcan library definitions.
**/

#ifndef ADU_PLAT_SW_LIB_BCAN_BCAN_H
#define ADU_PLAT_SW_LIB_BCAN_BCAN_H

#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/ioctl.h>

/* bcan_msg_t and bcan_err_code definitions. */
#include "linux/bcan_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BCAN_MAX_TX_MSG		256
#define BCAN_MAX_RX_MSG		256

// Channel states
#define BCAN_DEV_UNINIT		-1
#define BCAN_DEV_OPEN		(1 << 0)
#define BCAN_DEV_CLOSE		(1 << 1)
#define BCAN_DEV_BAUD_SET	(1 << 2)
#define BCAN_DEV_NORMAL		(1 << 3)
#define BCAN_DEV_LOOPBACK	(1 << 4)
#define BCAN_DEV_CONFIG		(1 << 5)
#define BCAN_DEV_START		(1 << 6)
#define BCAN_DEV_STOP		(1 << 7)
#define BCAN_DEV_ACTIVE		(1 << 8)
#define BCAN_DEV_RECVD		(1 << 9)


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

int bcan_open(uint32_t dev_index, uint32_t flags, uint64_t tx_to, uint64_t rx_to,
	bcan_hdl_t *hdl);
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

#endif  /* ADU_PLAT_SW_LIB_BCAN_BCAN_LIB_H */
