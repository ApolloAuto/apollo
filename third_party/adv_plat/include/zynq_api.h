/*
 * zynq_api.h: Zynq driver APIs(exported ioctls and structures)
 *
 * @author Kerry Shu (shuguoli@baidu.com)
 * @date June 2016
 *
 * Modified by youxiangtao@
 * @date October 2017
 */

#ifndef _ZYNQ_API_H_
#define	_ZYNQ_API_H_

#include "bcan_defs.h"

#define	ZYNQ_DEV_NAME_FW	"zynq_fw"
#define	ZYNQ_DEV_NAME_TRIGGER	"zynq_trigger"
#define	ZYNQ_DEV_NAME_GPS	"zynq_gps"
#define	ZYNQ_DEV_NAME_REG	"zynq_reg"
#define	ZYNQ_DEV_NAME_CAN	"zynq_can"
#define	ZYNQ_DEV_NAME_I2C	"zynq_i2c"

/*
 * ioctl argument defintion for CAN send/recv
 */
typedef struct ioc_bcan_msg {
	bcan_msg_t	*ioc_msgs;
	unsigned int	ioc_msg_num;
	unsigned int	ioc_msg_num_done;
	int		ioc_msg_err;
	int		ioc_msg_rx_clear;
} ioc_bcan_msg_t;

/*
 * CAN error and status
 */
typedef struct ioc_bcan_status_err {
	unsigned int	bcan_status;
	unsigned int	bcan_err_status;
	unsigned int	bcan_err_count;
	int		bcan_ioc_err;
} ioc_bcan_status_err_t;

/* ioctl command list */
#define	ZYNQ_IOC_MAGIC	('z' << 12 | 'y' << 8 | 'n' << 4 | 'q')
enum ZYNQ_IOC_GPS_CMD {
	IOC_GPS_GET = 1,
	IOC_GPS_GPRMC_GET,
	IOC_GPS_CMD_MAX
};

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
	IOC_FW_GET_VER, /* get the image version */
	IOC_FW_CMD_MAX
};

enum ZYNQ_IOC_CAN_CMD {
	IOC_CAN_TX_TIMEOUT_SET = IOC_FW_CMD_MAX, /* in milli-seconds */
	IOC_CAN_RX_TIMEOUT_SET,	/* in milli-seconds */
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

enum ZYNQ_IOC_TRIGGER_EXT_CMD {
	IOC_TRIGGER_INIT_USB = IOC_REG_CMD_MAX,
	IOC_TRIGGER_ENABLE_ONE,
	IOC_TRIGGER_DISABLE_ONE,
	IOC_TRIGGER_ENABLE,
	IOC_TRIGGER_DELAY_SET,
	IOC_TRIGGER_DELAY_GET,
	IOC_TRIGGER_DEV_NAME,
	IOC_TRIGGER_EXT_MAX
};

enum ZYNQ_IOC_CAM_CMD {
	IOC_CAM_REG_READ = IOC_TRIGGER_EXT_MAX,
	IOC_CAM_REG_WRITE,
	IOC_CAM_FLASH_INIT,
	IOC_CAM_FLASH_FINI,
	IOC_CAM_FLASH_READ,
	IOC_CAM_FLASH_WRITE,
	IOC_CAM_CAPS,
	IOC_CAM_RESET
};

enum zynq_baudrate_val {
	ZYNQ_BAUDRATE_1M,
	ZYNQ_BAUDRATE_500K,
	ZYNQ_BAUDRATE_250K,
	ZYNQ_BAUDRATE_150K,
	ZYNQ_BAUDRATE_NUM
};

/* GPS update ioctl cmds */
#define	ZYNQ_GPS_VAL_SZ			12
#define	ZYNQ_IOC_GPS_GET		\
		_IOR(ZYNQ_IOC_MAGIC, IOC_GPS_GET,  unsigned char *)
#define ZYNQ_GPS_GPRMC_VAL_SZ		68
#define	ZYNQ_IOC_GPS_GPRMC_GET		\
		_IOR(ZYNQ_IOC_MAGIC, IOC_GPS_GPRMC_GET, unsigned char *)

/* Trigger ioctl cmds */
#define	ZYNQ_IOC_TRIGGER_DISABLE	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_TRIGGER_DISABLE, unsigned long)
#define	ZYNQ_IOC_TRIGGER_ENABLE_GPS	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_TRIGGER_ENABLE_GPS, unsigned long)
#define	ZYNQ_IOC_TRIGGER_ENABLE_NOGPS	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_TRIGGER_ENABLE_NOGPS, unsigned long)
#define	ZYNQ_IOC_TRIGGER_ENABLE_ONE_GPS	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_TRIGGER_ENABLE_ONE_GPS, unsigned long)
#define	ZYNQ_IOC_TRIGGER_ENABLE_ONE_NOGPS	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_TRIGGER_ENABLE_ONE_NOGPS, unsigned long)
#define	ZYNQ_IOC_TRIGGER_TIMESTAMP	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_TRIGGER_TIMESTAMP, unsigned long)
#define	ZYNQ_IOC_TRIGGER_STATUS		\
		_IOR(ZYNQ_IOC_MAGIC, IOC_TRIGGER_STATUS, int *)
#define	ZYNQ_IOC_TRIGGER_STATUS_GPS	\
		_IOR(ZYNQ_IOC_MAGIC, IOC_TRIGGER_STATUS_GPS, int *)
#define	ZYNQ_IOC_TRIGGER_STATUS_PPS	\
		_IOR(ZYNQ_IOC_MAGIC, IOC_TRIGGER_STATUS_PPS, int *)
#define	ZYNQ_IOC_TRIGGER_FPS_SET	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_TRIGGER_FPS_SET, int *)
#define	ZYNQ_IOC_TRIGGER_FPS_GET	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_TRIGGER_FPS_GET, int *)
#define	ZYNQ_IOC_TRIGGER_INIT_USB	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_TRIGGER_INIT_USB, unsigned long)
#define	ZYNQ_IOC_TRIGGER_ENABLE_ONE	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_TRIGGER_ENABLE_ONE, unsigned long)
#define	ZYNQ_IOC_TRIGGER_DISABLE_ONE	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_TRIGGER_DISABLE_ONE, unsigned long)
#define	ZYNQ_IOC_TRIGGER_ENABLE		\
		_IOW(ZYNQ_IOC_MAGIC, IOC_TRIGGER_ENABLE, unsigned long)
#define	ZYNQ_IOC_TRIGGER_DELAY_SET      \
		_IOW(ZYNQ_IOC_MAGIC, IOC_TRIGGER_DELAY_SET, unsigned long)
#define	ZYNQ_IOC_TRIGGER_DELAY_GET      \
		_IOW(ZYNQ_IOC_MAGIC, IOC_TRIGGER_DELAY_GET, unsigned long)
#define	ZYNQ_IOC_TRIGGER_DEV_NAME	\
		_IOR(ZYNQ_IOC_MAGIC, IOC_TRIGGER_DEV_NAME, char *)

/* Camera register I2C cmds */
#define	ZYNQ_IOC_CAM_REG_READ	\
		_IOWR(ZYNQ_IOC_MAGIC, IOC_CAM_REG_READ, unsigned long)
#define	ZYNQ_IOC_CAM_REG_WRITE	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_CAM_REG_WRITE, unsigned long)
#define	ZYNQ_IOC_CAM_FLASH_INIT \
		_IOW(ZYNQ_IOC_MAGIC, IOC_CAM_FLASH_INIT, unsigned long)
#define	ZYNQ_IOC_CAM_FLASH_FINI \
		_IO(ZYNQ_IOC_MAGIC, IOC_CAM_FLASH_FINI)
#define	ZYNQ_IOC_CAM_FLASH_READ \
		_IOW(ZYNQ_IOC_MAGIC, IOC_CAM_FLASH_READ, unsigned long)
#define	ZYNQ_IOC_CAM_FLASH_WRITE \
		_IOW(ZYNQ_IOC_MAGIC, IOC_CAM_FLASH_WRITE, unsigned long)
#define	ZYNQ_IOC_CAM_CAPS \
		_IOWR(ZYNQ_IOC_MAGIC, IOC_CAM_CAPS, unsigned long)
#define	ZYNQ_IOC_CAM_RESET \
		_IO(ZYNQ_IOC_MAGIC, IOC_CAM_RESET)

#define	ZYNQ_CAM_FW_BLOCK_SIZE	250

typedef struct zynq_cam_fw {
	unsigned char	*data;
	unsigned int	size;
	unsigned int	address;
} zynq_cam_fw_t;

#define	ZYNQ_TRIGGER_DEV_NUM	4

/* supported Leopard Imaging fps */
#define	LI_FPS_30_DEFAULT	0	/* 30Hz */
#define	LI_FPS_20		1	/* 20Hz */
#define	LI_FPS_15		2	/* 15Hz */
#define	LI_FPS_10		3	/* 10Hz */
/* adv_trigger specify fps in format of <GH><LI><BB><LD> */
#define	ZYNQ_FPS_LI(fps)	((fps >> 8) & 0xf)
#define	ZYNQ_FPS_KEEP		0xf
#define	ZYNQ_FPS_KEEP_ALL	0xffff
#define	ZYNQ_FPS_ALL_DEFAULT	0
#define	ZYNQ_FPS_LI_DEFAULT	0xf0ff

#define	ZYNQ_USB_FPS_MAX	30
#define	ZYNQ_USB_FPS_DEFAULT	30
#define	ZYNQ_USB_TRIG_NUM	5
#define	ZYNQ_USB_TRIG_TOTAL	ZYNQ_USB_TRIG_NUM * ZYNQ_TRIGGER_DEV_NUM

#define	ZYNQ_FPD_FPS_MAX	20
#define	ZYNQ_FPD_FPS_DEFAULT	10	/* 10Hz default */
#define	ZYNQ_FPD_TRIG_NUM	10
#define	ZYNQ_FPD_TRIG_TOTAL	ZYNQ_FPD_TRIG_NUM * ZYNQ_TRIGGER_DEV_NUM
#define	ZYNQ_FPD_CAM_NAME	"FPD"

#define	ZYNQ_DEV_NAME_LEN	64
#define	ZYNQ_DEV_CODE_NAME_LEN	32
#define	ZYNQ_VDEV_NAME_LEN	56

/*
 * For USB cameras, we assign trigger IDs to designated trigger lines.
 * The user applications and kernel driver should always use the same
 * mappings:
 *  0: LI trigger
 *  1: GH trigger
 *  2: BB trigger
 *  3: LB trigger
 *  4: Test trigger
 * For FPD-link cameras, the trigger id mapping is always fixed:
 *  0: 1st FPD-link
 *  1: 2nd FPD-link
 *  ...
 */
typedef struct zynq_trigger {
	unsigned char	id;		/* Trigger id */
	unsigned char	fps;		/* Frame-Per-Second */
	unsigned char	internal;	/* 1: Internal PPS; 0: GPS PPS */
	unsigned char	enabled;	/* 1: Enabled; 0: Disabled */
	unsigned int	trigger_delay;
	unsigned int	exposure_time;
	/* Video number, e.g. 0 for /dev/video0 */
	int		vnum;
	/* Camera name, e.g. AR023ZWDR(Rev663) */
	char		name[ZYNQ_VDEV_NAME_LEN];
} zynq_trigger_t;

/*
 * Camera trigger delay, currently used by sensor_sync
 */
typedef struct zynq_trigger_delay {
	int vnum;
	unsigned int trigger_delay;
	unsigned int exposure_time;
} zynq_trigger_delay_t;

/* FW update ioctl cmds */
#define	ZYNQ_FW_PADDING		0x00000000
#define	ZYNQ_FW_MSG_SZ		16
typedef struct ioc_zynq_fw_upload {
	/*
	 * image data size must be multiple of 4 as each polling transfer is in
	 * 16-byte, so padding the data if needed.
	 */
	unsigned int	*ioc_zynq_fw_data;
	unsigned int	ioc_zynq_fw_num;
	unsigned int	ioc_zynq_fw_done;
	int		ioc_zynq_fw_err;
} ioc_zynq_fw_upload_t;

#define	ZYNQ_IOC_FW_IMAGE_UPLOAD_START	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_FW_IMAGE_UPLOAD_START, unsigned long)
#define	ZYNQ_IOC_FW_IMAGE_UPLOAD	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_FW_IMAGE_UPLOAD, ioc_zynq_fw_upload_t *)
#define	ZYNQ_IOC_FW_PL_UPDATE		\
		_IOW(ZYNQ_IOC_MAGIC, IOC_FW_PL_UPDATE, unsigned long)
#define	ZYNQ_IOC_FW_PS_UPDATE		\
		_IOW(ZYNQ_IOC_MAGIC, IOC_FW_PS_UPDATE, unsigned long)
#define	ZYNQ_IOC_FW_GET_VER		\
		_IOW(ZYNQ_IOC_MAGIC, IOC_FW_GET_VER, unsigned int *)

/* CAN channel ioctl cmds */
#define	ZYNQ_IOC_CAN_TX_TIMEOUT_SET	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_CAN_TX_TIMEOUT_SET, unsigned long)

#define	ZYNQ_IOC_CAN_RX_TIMEOUT_SET	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_CAN_RX_TIMEOUT_SET, unsigned long)

#define	ZYNQ_IOC_CAN_DEV_START	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_CAN_DEV_START, unsigned long)

#define	ZYNQ_IOC_CAN_DEV_STOP	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_CAN_DEV_STOP, unsigned long)

#define	ZYNQ_IOC_CAN_DEV_RESET	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_CAN_DEV_RESET, unsigned long)

#define	ZYNQ_IOC_CAN_ID_ADD	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_CAN_ID_ADD, unsigned long)

#define	ZYNQ_IOC_CAN_ID_DEL	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_CAN_ID_DEL, unsigned long)

#define	ZYNQ_IOC_CAN_BAUDRATE_SET	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_CAN_BAUDRATE_SET, unsigned long)

#define	ZYNQ_IOC_CAN_BAUDRATE_GET	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_CAN_BAUDRATE_GET, unsigned long)

#define	ZYNQ_IOC_CAN_LOOPBACK_SET	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_CAN_LOOPBACK_SET, unsigned long)

#define	ZYNQ_IOC_CAN_LOOPBACK_UNSET	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_CAN_LOOPBACK_UNSET, unsigned long)

#define	ZYNQ_IOC_CAN_RECV		\
		_IOWR(ZYNQ_IOC_MAGIC, IOC_CAN_RECV, ioc_bcan_msg_t *)

#define	ZYNQ_IOC_CAN_SEND		\
		_IOWR(ZYNQ_IOC_MAGIC, IOC_CAN_SEND, ioc_bcan_msg_t *)

#define	ZYNQ_IOC_CAN_SEND_HIPRI		\
		_IOWR(ZYNQ_IOC_MAGIC, IOC_CAN_SEND_HIPRI, ioc_bcan_msg_t *)

#define	ZYNQ_IOC_CAN_GET_STATUS_ERR	\
		_IOR(ZYNQ_IOC_MAGIC, IOC_CAN_GET_STATUS_ERR, \
		ioc_bcan_status_err_t *)

/* register read/write ioctl cmds */
typedef struct ioc_zynq_reg_acc {
	unsigned int	reg_bar;
	unsigned int	reg_offset;
	unsigned int	reg_data;
} ioc_zynq_reg_acc_t;

/* I2C ID definitions */
#define	ZYNQ_I2C_ID_JANUS	0x5c
#define	ZYNQ_I2C_ID_MAX		0x7f /* 7-bit */

typedef struct ioc_zynq_i2c_acc {
	unsigned char	i2c_id; /* 7-bit */
	unsigned char	i2c_addr_hi;
	unsigned char	i2c_addr;
	unsigned char	i2c_data;
	unsigned char	i2c_addr_16;
	unsigned char	i2c_bus;
} ioc_zynq_i2c_acc_t;

typedef struct zynq_cam_acc {
	unsigned short	addr;
	unsigned short	data_sz;
	unsigned int	data;
} zynq_cam_acc_t;

#define	ZYNQ_IOC_REG_READ		\
		_IOR(ZYNQ_IOC_MAGIC, IOC_REG_READ, ioc_zynq_reg_acc_t *)
#define	ZYNQ_IOC_REG_WRITE		\
		_IOW(ZYNQ_IOC_MAGIC, IOC_REG_WRITE, ioc_zynq_reg_acc_t *)
#define	ZYNQ_IOC_REG_I2C_READ		\
		_IOR(ZYNQ_IOC_MAGIC, IOC_REG_I2C_READ, ioc_zynq_i2c_acc_t *)
#define	ZYNQ_IOC_REG_I2C_WRITE		\
		_IOW(ZYNQ_IOC_MAGIC, IOC_REG_I2C_WRITE, ioc_zynq_i2c_acc_t *)
/* wait for GPS/PPS status change event notification */
#define	ZYNQ_IOC_REG_GPSPPS_EVENT_WAIT	\
		_IOW(ZYNQ_IOC_MAGIC, IOC_REG_GPSPPS_EVENT_WAIT, unsigned long)

#define	ZVIDEO_EXT_ERR_FRAME_FORMAT	0x01
#define	ZVIDEO_EXT_ERR_SHORT_FRAME	0x02
#define	ZVIDEO_EXT_ERR_LONG_FRAME	0x04
#define	ZVIDEO_EXT_ERR_ALL		0x07
#define	ZVIDEO_EXT_ERR_INVALID		0xFF
#define	ZVIDEO_FRAME_CORRUPTED(ext)	\
	((*(unsigned int *)ext->rsv1) ||	\
	(*(unsigned int *)&ext->rsv2[6]) ||	\
	(*(unsigned int *)&ext->rsv2[9]) ||	\
	(ext->error & ~(unsigned char)ZVIDEO_EXT_ERR_ALL))

typedef struct zynq_video_ext_meta_data {
	unsigned int trigger_cnt;
	struct {
		unsigned int usec:20;
		unsigned int :12;
		unsigned int sec;
	} time_stamp;
	unsigned char rsv1[4];
	struct {
		unsigned short us_cnt:12;
		unsigned short sec:4;
	} debug_ts;
	unsigned char rsv2[13];
	unsigned char error;
} zynq_video_ext_meta_data_t;

/* Trigger mode */
#define	CAM_CAP_TRIGGER_STANDARD	0
#define	CAM_CAP_TRIGGER_DETERMINISTIC	1
#define	CAM_CAP_TRIGGER_SLAVE_STANDARD	2
#define	CAM_CAP_TRIGGER_SLAVE_SHUTTER_SYNC 3

/* Timestamp type */
#define	CAM_CAP_TIMESTAMP_FPGA		0
#define	CAM_CAP_TIMESTAMP_TRIGGER	1
#define	CAM_CAP_TIMESTAMP_FORMATION	2
#define	CAM_CAP_TIMESTAMP_HOST		3

/* Interafce type */
#define	CAM_CAP_INTERFACE_PARALLEL	0
#define	CAM_CAP_INTERFACE_MIPI		1

/* Camera capabilities */
typedef struct zynq_camera_capabilities {
	char		name[ZYNQ_VDEV_NAME_LEN];
	char		trigger_mode;
	char		embedded_data;
	unsigned char	link_up;
	unsigned char	interface_type;
	unsigned char	timestamp_type;
	unsigned short	frame_len_lines;
	unsigned short	line_len_pck;
	unsigned int	pixel_clock;
} zynq_cam_caps_t;

/* Fixed embedded data header: 00 0A 00 AA 00 30 00 A5 00 00 00 5A */
#define	EM_HDR_LEN			12
#define	EM_HDR_DWORD0			0xAA000A00
#define	EM_HDR_DWORD1			0xA5003000
#define	EM_HDR_DWORD2			0x5A000000
#define	EM_REG_BASE			0x3000
#define	EM_REG_CHIP_VER			0x3000
#define	EM_REG_FRAME_LEN_LINES		0x300A
#define	EM_REG_LINE_LEN_PCK		0x300C
#define	EM_REG_LOCK_CONTROL		0x3010
#define	EM_REG_COARSE_INT		0x3012
#define	EM_REG_FRAME_COUNT		0x303A
#define	EM_REG_FRAME_STATUS		0x303C
#define	EM_REG_EXTRA_DELAY		0x3042
#define	EM_REG_OFFSET(x)		(((x) - EM_REG_BASE) << 2)
#define	EM_REG_VAL_MSB(buf, x)		\
	(*((unsigned char *)(buf) + EM_HDR_LEN + EM_REG_OFFSET(x) + 1))
#define	EM_REG_VAL_LSB(buf, x)		\
	(*((unsigned char *)(buf) + EM_HDR_LEN + EM_REG_OFFSET(x) + 5))
#define	EM_REG_VAL(buf, x)	\
	((EM_REG_VAL_MSB(buf, x) << 8) + EM_REG_VAL_LSB(buf, x))

#define	EM_DATA_VALID(buf)	\
	((((unsigned int *)(buf))[0] == EM_HDR_DWORD0) && \
	(((unsigned int *)(buf))[1] == EM_HDR_DWORD1) && \
	(((unsigned int *)(buf))[2] == EM_HDR_DWORD2))

#define	DEFAULT_PIXCLK			79312500
#define	T_ROW(llp, pck)			\
	div_u64((u64)(llp) * USEC_PER_SEC, (pck))
#define	T_COARSE_INT(cit, llp, pck)	\
	mul_u64_u32_div((u64)(llp) * USEC_PER_SEC, (cit), (pck))

#endif	/* _ZYNQ_API_H_ */
