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

#include "modules/drivers/velodyne/fusion/pri_sec_fusion_component.h"

#include <memory>
#include <thread>

namespace apollo {
namespace drivers {
namespace velodyne {

using apollo::cyber::Time;

bool PriSecFusionComponent::Init() {
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

bool PriSecFusionComponent::Proc(
    const std::shared_ptr<PointCloud>& point_cloud) {
  auto target = point_cloud;
  auto fusion_readers = readers_;
  auto start_time = Time::Now().ToSecond();
  while ((Time::Now().ToSecond() - start_time) < conf_.wait_time_s() &&
         fusion_readers.size() > 0) {
    for (auto itr = fusion_readers.begin(); itr != fusion_readers.end();) {
      (*itr)->Observe();
      if (!(*itr)->Empty()) {
        auto source = (*itr)->GetLatestObserved();
        if (conf_.drop_expired_data() && IsExpired(target, source)) {
          ++itr;
        } else {
          Fusion(target, source);
          itr = fusion_readers.erase(itr);
        }
      } else {
        ++itr;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  auto diff = Time::Now().ToNanosecond() - target->header().lidar_timestamp();
  AINFO << "Pointcloud fusion diff: " << diff / 1000000 << "ms";
  fusion_writer_->Write(target);

  return true;
}

bool PriSecFusionComponent::IsExpired(
    const std::shared_ptr<PointCloud>& target,
    const std::shared_ptr<PointCloud>& source) {
  auto diff = target->measurement_time() - source->measurement_time();
  return diff * 1000 > conf_.max_interval_ms();
}

bool PriSecFusionComponent::QueryPoseAffine(const std::string& target_frame_id,
                                            const std::string& source_frame_id,
                                            Eigen::Affine3d* pose) {
  std::string err_string;
  if (!buffer_ptr_->canTransform(target_frame_id, source_frame_id,
                                 cyber::Time(0), 0.02f, &err_string)) {
    AERROR << "Can not find transform. "
           << "target_id:" << target_frame_id << " frame_id:" << source_frame_id
           << " Error info: " << err_string;
    return false;
  }
  apollo::transform::TransformStamped stamped_transform;
  try {
    stamped_transform = buffer_ptr_->lookupTransform(
        target_frame_id, source_frame_id, cyber::Time(0));
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

void PriSecFusionComponent::AppendPointCloud(
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

bool PriSecFusionComponent::Fusion(std::shared_ptr<PointCloud> target,
                                   std::shared_ptr<PointCloud> source) {
  Eigen::Affine3d pose;
  if (QueryPoseAffine(target->header().frame_id(), source->header().frame_id(),
                      &pose)) {
    AppendPointCloud(target, source, pose);
    return true;
  }
  return false;
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
