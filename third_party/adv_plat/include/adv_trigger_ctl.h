/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#ifndef THIRD_PARTY_ADV_PLAT_INCLUDE_ADV_TRIGGER_CTL_H
#define THIRD_PARTY_ADV_PLAT_INCLUDE_ADV_TRIGGER_CTL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "zynq_api.h"

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

#endif  // THIRD_PARTY_ADV_PLAT_INCLUDE_ADV_TRIGGER_CTL_H
