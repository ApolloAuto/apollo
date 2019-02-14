/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "cyber/cyber.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/perception/base/blob.h"
#include "modules/perception/base/lane_struct.h"
#include "modules/perception/base/object.h"
#include "modules/perception/base/object_types.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

namespace apollo {
namespace perception {
namespace onboard {

class CameraPerceptionVizMessage {
 public:
  CameraPerceptionVizMessage() { type_name_ = "CameraPerceptionVizMessage"; }
  ~CameraPerceptionVizMessage() = default;

  CameraPerceptionVizMessage& operator=(const CameraPerceptionVizMessage&) =
      delete;

  std::string GetTypeName() const { return type_name_; }

  CameraPerceptionVizMessage* New() const {
    return new CameraPerceptionVizMessage;
  }

  CameraPerceptionVizMessage(
      const std::string& camera_name, const double msg_timestamp,
      const Eigen::Matrix4d& pose_camera_to_world,
      const std::shared_ptr<base::Blob<uint8_t> >& image_blob,
      const std::vector<base::ObjectPtr>& camera_objects,
      const std::vector<base::LaneLine>& lane_objects,
      const apollo::common::ErrorCode& error_code);

 public:
  std::string camera_name_;
  std::string type_name_;
  double msg_timestamp_ = 0.0;

  Eigen::Matrix4d pose_camera_to_world_;
  std::shared_ptr<base::Blob<uint8_t> > image_blob_;
  std::vector<base::ObjectConstPtr> camera_objects_;
  std::vector<base::LaneLine> lane_objects_;
  apollo::common::ErrorCode error_code_ = apollo::common::ErrorCode::OK;
};

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
