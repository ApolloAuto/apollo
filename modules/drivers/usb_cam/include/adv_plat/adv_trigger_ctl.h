/************************************************************************
*
* Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
*
************************************************************************/

/**
* @file adv_trigger_ctl.h
* @author feiaiguo@, shuguoli@
* @date 2016/07/11
*
* Baidu ADV (Autonomous Driving Vehicle) hardware trigger library, definitions.
*
* Modified by youxiangtao@
* @date 2017/09/10
**/

#ifndef ADU_PLAT_SW_LIB_ADV_TRIGGER_ADV_TRIGGER_CTL_H
#define ADU_PLAT_SW_LIB_ADV_TRIGGER_ADV_TRIGGER_CTL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "linux/zynq_api.h"

#define	LIB_NAME	"adv_trigger"

/*
 * @brief Main functions to control hardware triggers.
 *
 * @param video_path: path of the video device file, e.g. /dev/video0.
 *  When NULL, all triggers are enabled/disabled.
 * @param fps: FPS (frame-per-second) for the specified video device (camera).
 *  The supported values are: 1 ~ 30.
 *  All other values set FPS to the default value 30.
 * @param internal: source of the PPS.
 *  0: use GPS generated PPS;
 *  1: use FPGA internal PPS.
 *
 * @return:
 *  0: success;
 *  non-0: there is an error.
 */
int adv_trigger_enable(const char *video_path,
		unsigned char fps, unsigned char internal);
int adv_trigger_disable(const char *video_path);
int adv_trigger_delay_ctl(const char *video_path, int action,
		unsigned int *trigger_delay, unsigned int *exp_time);

#define	FLAG_GPS_VALID		(1 << 0)
#define	FLAG_PPS_VALID		(1 << 1)

/* FPGA board trigger status. */
struct adv_zynq_status {
	struct {
		int flags;
		char zdev_name[ZYNQ_DEV_NAME_LEN];
		zynq_trigger_t fpd_triggers[ZYNQ_FPD_TRIG_NUM];
		zynq_trigger_t usb_triggers[ZYNQ_USB_TRIG_NUM];
	} status[ZYNQ_TRIGGER_DEV_NUM];
};

/*
 * @brief Gets the status of all triggers.
 *
 * @param status: pointer to the struct adv_zynq_status.
 *
 * @return:
 *  0: successful;
 *  non-0: there is an error.
 */
int adv_trigger_status(struct adv_zynq_status *status);

const char *get_adv_trigger_version(void);
const char *get_adv_trigger_bld_info(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ADU_PLAT_SW_LIB_ADV_TRIGGER_ADV_TRIGGER_CTL_H
