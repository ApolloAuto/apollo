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
#ifndef THIRD_PARTY_ADV_PLAT_INCLUDE_ADV_TRIGGER_CTL_H_
#define THIRD_PARTY_ADV_PLAT_INCLUDE_ADV_TRIGGER_CTL_H_

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

#endif  // THIRD_PARTY_ADV_PLAT_INCLUDE_ADV_TRIGGER_CTL_H_
