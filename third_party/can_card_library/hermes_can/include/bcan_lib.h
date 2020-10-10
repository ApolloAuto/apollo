#ifndef _BCAN_LIB_H_
#define _BCAN_LIB_H_

#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include "linux/zynq_api.h"

#ifdef DEBUG
#define BLOG_DBG0(s...) syslog(LOG_DEBUG, s);
#else
#define BLOG_DBG0(s...) do{}while(0);
#endif
#define BLOG_ERR(s...) syslog(LOG_ERR, s);

typedef uint64_t bcan_hdl_t;

#define BCAN_MAX_TX_MSG		256
#define BCAN_MAX_RX_MSG		256

typedef struct bcan_ihdl {
	int		dev_index;
	int		dev_state;
	int		fd;
	uint32_t	baudrate;
	uint32_t	tx_to;
	uint32_t	rx_to;
} bcan_ihdl_t;

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

#endif  /* _BCAN_LIB_H_ */
