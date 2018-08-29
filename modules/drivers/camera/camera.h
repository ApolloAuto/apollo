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

#ifndef MODULES_DRIVERS_CAMERA_CAMERA_H_
#define MODULES_DRIVERS_CAMERA_CAMERA_H_

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "modules/common/apollo_app.h"
#include "modules/common/util/util.h"
#include "modules/drivers/camera/nodes/camera_nodelet.h"

namespace apollo {
namespace drivers {
namespace camera {

class Camera : public apollo::common::ApolloApp {
 public:
  Camera() = default;
  virtual ~Camera();

  std::string Name() const override;
  apollo::common::Status Init() override;
  apollo::common::Status Start() override;
  void Stop() override;
  void RunOnce();
  void OnTimer(const ros::TimerEvent&);

 private:
  bool running_ = true;
  std::vector<std::shared_ptr<std::thread> > threads_;
  CameraNodelet camera_nodelet_;
};

}  // namespace camera
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_CAMERA_CAMERA_H_
