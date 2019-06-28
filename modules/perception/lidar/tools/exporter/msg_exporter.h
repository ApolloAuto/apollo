/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>
#include "cyber/cyber.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/drivers/proto/sensor_image.pb.h"
#include "modules/perception/lidar/common/pcl_util.h"
#include "modules/perception/onboard/transform_wrapper/transform_wrapper.h"
#include "modules/transform/buffer.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::transform::Buffer;

class MsgExporter {
 public:
  typedef apollo::drivers::Image ImgMsg;
  typedef apollo::drivers::PointCloud PcMsg;

 public:
  MsgExporter(std::shared_ptr<apollo::cyber::Node> node,
              const std::vector<std::string> channels,
              const std::vector<std::string> child_frame_ids);
  ~MsgExporter() = default;

 private:
  void ImageMessageHandler(const std::shared_ptr<const ImgMsg>& img_msg,
                           const std::string& channel,
                           const std::string& child_frame_id,
                           const std::string& folder);
  void PointCloudMessageHandler(const std::shared_ptr<const PcMsg>& cloud_msg,
                                const std::string& channel,
                                const std::string& child_frame_id,
                                const std::string& folder);
  bool SavePointCloud(const pcl::PointCloud<PCLPointXYZIT>& point_cloud,
                      double timestamp, const std::string& folder);
  bool SaveImage(const unsigned char* color_image,
                 const unsigned char* range_image, std::size_t width,
                 std::size_t height, double timestamp,
                 const std::string& folder);
  bool QuerySensorToWorldPose(double timestamp,
                              const std::string& child_frame_id,
                              Eigen::Matrix4d* pose);
  bool QueryPose(double timestamp, const std::string& frame_id,
                 const std::string& child_frame_id, Eigen::Matrix4d* pose);
  bool SavePose(const Eigen::Matrix4d& pose, double timestamp,
                const std::string& folder);
  bool IsStereoCamera(const std::string& channel);
  bool IsCamera(const std::string& channel);
  bool IsLidar(const std::string& channel);
  std::string TransformChannelToFolder(const std::string& channel);

 private:
  std::shared_ptr<apollo::cyber::Node> _cyber_node;
  std::vector<std::string> _channels;
  std::vector<std::string> _child_frame_ids;
  std::string _localization_method;
  Buffer* tf2_buffer_ = Buffer::Instance();
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
