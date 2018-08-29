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

#include "modules/drivers/camera/nodes/camera_nodelet.h"

#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/drivers/camera/common/camera_gflags.h"
#include "modules/drivers/camera/nodes/usb_cam_wrapper.h"

namespace apollo {
namespace drivers {
namespace camera {

CameraNodelet::CameraNodelet() {}

CameraNodelet::~CameraNodelet() {
  AINFO << "shutting down driver thread";
  if (device_thread_ != nullptr && device_thread_->joinable()) {
    device_thread_->join();
  }
  AINFO << "driver thread stopped";
}

void CameraNodelet::OnInit() {
  AINFO << "Usb cam nodelet init";
  CameraConf config;
  CHECK(apollo::common::util::GetProtoFromFile(FLAGS_camera_config_file,
                                               &config))
      << "failed to load camera config file: " << FLAGS_camera_config_file;

  // TODO(all)
  // usb_cam_wrapper_.reset(new UsbCamWrapper(nh_, private_nh_, config));
  // // spawn device poll thread
  // device_thread_ = boost::shared_ptr<boost::thread>(new boost::thread(
  //    boost::bind(&UsbCamWrapper::spin, usb_cam_wrapper_)));
}

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
