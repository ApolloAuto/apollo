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

#include <memory>

#include "modules/drivers/velodyne/fusion/fusion_component.h"

namespace apollo {
namespace drivers {
namespace velodyne {

bool FusionComponent::Init() {
  if (!GetProtoConfig(&conf_)) {
    AWARN << "Load config failed, config file" << ConfigFilePath();
    return false;
  }
  buffer_ptr_ = apollo::transform::Buffer::Instance();

  fusion_writer_ = node_->CreateWriter<PointCloud>(conf_.fusion_channel());

  for (const auto& channel : conf_.input_channel()) {
    auto reader = node_->CreateReader<PointCloud>(channel);
    readers_.emplace_back(reader);
  }
  return true;
}

bool FusionComponent::Proc(const std::shared_ptr<PointCloud>& point_cloud) {
  auto target = point_cloud;
  for (const auto& reader : readers_) {
    reader->Observe();
    if (!reader->Empty()) {
      auto source = reader->GetLatestObserved();
      if (conf_.drop_expired_data()) {
        if (IsExpired(target, source)) {
          continue;
        }
      }
      Fusion(target, source);
    }
  }
  fusion_writer_->Write(target);

  return true;
}

bool FusionComponent::IsExpired(const std::shared_ptr<PointCloud>& target,
                                const std::shared_ptr<PointCloud>& source) {
  auto diff = target->measurement_time() - source->measurement_time();
  return diff * 1000 > conf_.max_interval_ms();
}

bool FusionComponent::QueryPoseAffine(const std::string& target_frame_id,
                                      const std::string& source_frame_id,
                                      Eigen::Affine3d* pose) {
  std::string err_string;
  apollo::transform::TransformStamped stamped_transform;
  try {
    stamped_transform = buffer_ptr_->lookupTransform(
        target_frame_id, source_frame_id, cybertron::Time(0));
  } catch (tf2::TransformException& ex) {
    AERROR << ex.what();
    return false;
  }
  *pose =
      Eigen::Translation3d(stamped_transform.transform().translation().x(),
                           stamped_transform.transform().translation().y(),
                           stamped_transform.transform().translation().z()) *
      Eigen::Quaterniond(stamped_transform.transform().rotation().qw(),
                         stamped_transform.transform().rotation().qx(),
                         stamped_transform.transform().rotation().qy(),
                         stamped_transform.transform().rotation().qz());
  return true;
}

void FusionComponent::AppendPointCloud(
    std::shared_ptr<PointCloud> point_cloud,
    std::shared_ptr<PointCloud> point_cloud_add, const Eigen::Affine3d& pose) {
  if (std::isnan(pose(0, 0))) {
    for (auto& point : point_cloud_add->point()) {
      PointXYZIT* point_new = point_cloud->add_point();
      point_new->set_intensity(point.intensity());
      point_new->set_timestamp(point.timestamp());
      point_new->set_x(point.x());
      point_new->set_y(point.y());
      point_new->set_z(point.z());
    }
  } else {
    for (auto& point : point_cloud_add->point()) {
      if (std::isnan(point.x())) {
        PointXYZIT* point_new = point_cloud->add_point();
        point_new->set_intensity(point.intensity());
        point_new->set_timestamp(point.timestamp());
        point_new->set_x(point.x());
        point_new->set_y(point.y());
        point_new->set_z(point.z());
      } else {
        PointXYZIT* point_new = point_cloud->add_point();
        point_new->set_intensity(point.intensity());
        point_new->set_timestamp(point.timestamp());
        Eigen::Matrix<float, 3, 1> pt(point.x(), point.y(), point.z());
        point_new->set_x(static_cast<float>(
            pose(0, 0) * pt.coeffRef(0) + pose(0, 1) * pt.coeffRef(1) +
            pose(0, 2) * pt.coeffRef(2) + pose(0, 3)));
        point_new->set_y(static_cast<float>(
            pose(1, 0) * pt.coeffRef(0) + pose(1, 1) * pt.coeffRef(1) +
            pose(1, 2) * pt.coeffRef(2) + pose(1, 3)));
        point_new->set_z(static_cast<float>(
            pose(2, 0) * pt.coeffRef(0) + pose(2, 1) * pt.coeffRef(1) +
            pose(2, 2) * pt.coeffRef(2) + pose(2, 3)));
      }
    }
  }

  int new_width = point_cloud->point_size() / point_cloud->height();
  point_cloud->set_width(new_width);
}

bool FusionComponent::Fusion(std::shared_ptr<PointCloud> target,
                             std::shared_ptr<PointCloud> source) {
  Eigen::Affine3d pose;
  if (QueryPoseAffine(target->frame_id(), source->frame_id(), &pose)) {
    AppendPointCloud(target, source, pose);
    return true;
  }
  return false;
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
