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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/message/raw_message.h"

#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/localization/ndt/ndt_localization.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/transform/transform_broadcaster.h"

namespace apollo {
namespace localization {
namespace ndt {

class NDTLocalizationComponent final
    : public cyber::Component<localization::Gps> {
 public:
  NDTLocalizationComponent();
  ~NDTLocalizationComponent() = default;

  bool Init() override;

  bool Proc(const std::shared_ptr<localization::Gps> &odometry_msg) override;

 private:
  bool InitConfig();
  bool InitIO();

  void LidarCallback(const std::shared_ptr<drivers::PointCloud> &lidar_msg);
  void OdometryStatusCallback(
      const std::shared_ptr<drivers::gnss::InsStat> &status_msg);

  void PublishPoseBroadcastTF(const LocalizationEstimate &localization);
  void PublishPoseBroadcastTopic(const LocalizationEstimate &localization);
  void PublishLidarPoseBroadcastTopic(const LocalizationEstimate &localization);
  void PublishLocalizationStatusTopic(
      const LocalizationStatus &localization_status);

 private:
  std::shared_ptr<cyber::Reader<drivers::PointCloud>> lidar_listener_ = nullptr;

  std::shared_ptr<cyber::Reader<drivers::gnss::InsStat>>
      odometry_status_listener_ = nullptr;

  std::shared_ptr<cyber::Writer<LocalizationEstimate>> localization_talker_ =
      nullptr;

  std::shared_ptr<cyber::Writer<LocalizationEstimate>> lidar_pose_talker_ =
      nullptr;

  std::shared_ptr<cyber::Writer<LocalizationStatus>>
      localization_status_talker_ = nullptr;

  std::string lidar_topic_ = "";
  std::string odometry_topic_ = "";
  std::string localization_topic_ = "";
  std::string lidar_pose_topic_ = "";
  std::string odometry_status_topic_ = "";
  std::string localization_status_topic_ = "";

  std::string broadcast_tf_frame_id_ = "";
  std::string broadcast_tf_child_frame_id_ = "";
  std::unique_ptr<apollo::transform::TransformBroadcaster> tf2_broadcaster_;

  std::unique_ptr<NDTLocalization> localization_;
};

CYBER_REGISTER_COMPONENT(NDTLocalizationComponent);

}  // namespace ndt
}  // namespace localization
}  // namespace apollo
