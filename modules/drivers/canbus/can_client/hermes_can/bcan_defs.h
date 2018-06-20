/************************************************************************
*
* Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
*
************************************************************************/

/**
* @file bcan_defs.h
*
* Baidu ADV (Autonomous Driving Vehicle) bcan basic definitions.
**/

#ifndef BCAN_DEFS_H
#define BCAN_DEFS_H

#ifndef __KERNEL__
#include <sys/time.h>
#else
#include <linux/time.h>
#endif

/*
 * Baidu CAN message definition
 */
typedef struct bcan_msg {
	unsigned int	bcan_msg_id; /* source CAN node id */
	unsigned char	bcan_msg_datalen; /* message data len */
	unsigned char	bcan_msg_rsv[3];
	unsigned char	bcan_msg_data[8]; /* message data */
	struct timeval	bcan_msg_timestamp;
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

#endif
