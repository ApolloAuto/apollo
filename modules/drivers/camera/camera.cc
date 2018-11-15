/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include "modules/drivers/camera/camera.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/drivers/camera/common/camera_gflags.h"

namespace apollo {
namespace drivers {
namespace camera {

using apollo::common::adapter::AdapterManager;
using apollo::common::ErrorCode;
using apollo::common::Status;

Camera::~Camera() { Stop(); }

std::string Camera::Name() const { return FLAGS_camera_module_name; }

Status Camera::Init() {
  CameraConf config;
  CHECK(apollo::common::util::GetProtoFromFile(FLAGS_camera_config_file,
                                               &config))
      << "failed to load camera config file " << FLAGS_camera_config_file;

  if (!AdapterManager::Initialized()) {
    AdapterManager::Init(FLAGS_camera_adapter_config_filename);
  }

  // TODO(all)

  AINFO << "camera init: starting...";
  return Status::OK();
}

Status Camera::Start() {
  running_ = true;
  ADEBUG << "camera start done";
  return Status::OK();
}

void Camera::Stop() {
  AINFO << "camera stopping...";
  running_ = false;

  for (size_t i = 0; i < threads_.size(); ++i) {
    if (threads_[i]->joinable()) {
      threads_[i]->join();
    }
  }
  threads_.clear();
  AINFO << "camera stopped.";
}

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
