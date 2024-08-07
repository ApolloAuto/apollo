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

#include "modules/dreamview_plus/backend/point_cloud/point_cloud_updater.h"

#include <utility>
#include <vector>

#include "nlohmann/json.hpp"
#include "pcl/filters/voxel_grid.h"
#include "yaml-cpp/yaml.h"

#include "modules/dreamview/proto/point_cloud.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
namespace apollo {
namespace dreamview {

using apollo::localization::LocalizationEstimate;
using apollo::transform::TransformStampeds;
using Json = nlohmann::json;

boost::shared_mutex PointCloudUpdater::mutex_;

PointCloudUpdater::PointCloudUpdater(WebSocketHandler *websocket)
    : node_(cyber::CreateNode("point_cloud")),
      websocket_(websocket) {
  localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      FLAGS_localization_topic,
      [this](const std::shared_ptr<LocalizationEstimate> &msg) {
        UpdateLocalizationTime(msg);
      });
}

PointCloudUpdater::~PointCloudUpdater() { Stop(); }

PointCloudChannelUpdater* PointCloudUpdater::GetPointCloudChannelUpdater(
    const std::string &channel_name) {
  std::lock_guard<std::mutex> lck(channel_updater_map_mutex_);
  if (channel_updaters_.find(channel_name) == channel_updaters_.end()) {
    channel_updaters_[channel_name] =
        new PointCloudChannelUpdater(channel_name);
    channel_updaters_[channel_name]->point_cloud_reader_ =
        node_->CreateReader<drivers::PointCloud>(
            channel_name, [channel_name, this](
                              const std::shared_ptr<drivers::PointCloud> &msg) {
              UpdatePointCloud(msg, channel_name);
            });
  }
  return channel_updaters_[channel_name];
}

void PointCloudUpdater::StartStream(const double &time_interval_ms,
                                    const std::string &channel_name,
                                    nlohmann::json *subscribe_param) {
  if (channel_name.empty()) {
    AERROR << "Failed to subscribe channel for channel is empty!";
    return;
  }
  if (std::find(channels_.begin(), channels_.end(), channel_name) ==
      channels_.end()) {
    AERROR << "Failed to subscribe point cloud updater, for channel: "
           << channel_name << " is invalid.";
    return;
  }
  if (time_interval_ms > 0) {
    PointCloudChannelUpdater *updater =
        GetPointCloudChannelUpdater(channel_name);
    updater->timer_.reset(new cyber::Timer(
        time_interval_ms,
        [channel_name, this]() { this->OnTimer(channel_name); }, false));
    updater->timer_->Start();
  } else {
    this->OnTimer(channel_name);
  }
}
void PointCloudUpdater::Stop() { enabled_ = false; }
void PointCloudUpdater::OnTimer(const std::string &channel_name) {
  PublishMessage(channel_name);
}
void PointCloudUpdater::StopStream(const std::string& channel_name) {
  if (channel_name.empty()) {
    AERROR << "Failed to unsubscribe channel for channel is empty!";
    return;
  }
  PointCloudChannelUpdater *updater = GetPointCloudChannelUpdater(channel_name);
  if (updater->timer_) {
    updater->timer_->Stop();
  }
  // 回到初始值
  updater->last_point_cloud_time_ = 0.0;
  updater->point_cloud_str_ = "";
  updater->future_ready_ = true;
}
void PointCloudUpdater::PublishMessage(const std::string &channel_name) {
  PointCloudChannelUpdater *updater = GetPointCloudChannelUpdater(channel_name);
  std::string to_send;
  // the channel has no data input, clear the sending object.
  if (!updater->point_cloud_reader_->HasWriter()) {
    updater->last_point_cloud_time_ = 0.0;
    updater->point_cloud_str_ = "";
    to_send = "";
  } else {
    if (updater->point_cloud_str_ != "" &&
        std::fabs(last_localization_time_ - updater->last_point_cloud_time_) >
            2.0) {
      boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
      updater->point_cloud_str_ = "";
    }
    {
      boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
      to_send = updater->point_cloud_str_;
    }
  }
  StreamData stream_data;
  std::string stream_data_string;
  stream_data.set_action("stream");
  stream_data.set_data_name("pointcloud");
  stream_data.set_channel_name(channel_name);
  std::vector<uint8_t> byte_data(to_send.begin(), to_send.end());
  stream_data.set_data(&(byte_data[0]), byte_data.size());
  stream_data.set_type("pointcloud");
  stream_data.SerializeToString(&stream_data_string);
  websocket_->BroadcastBinaryData(stream_data_string);
}

void PointCloudUpdater::GetChannelMsg(std::vector<std::string> *channels) {
  enabled_ = true;
  GetChannelMsgWithFilter(channels, "PointCloud", "sensor");
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudUpdater::ConvertPCLPointCloud(
    const std::shared_ptr<drivers::PointCloud> &point_cloud,
    const std::string &channel_name) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl_ptr->width = point_cloud->width();
  pcl_ptr->height = point_cloud->height();
  pcl_ptr->is_dense = false;

  if (point_cloud->width() * point_cloud->height() !=
      static_cast<unsigned int>(point_cloud->point_size())) {
    pcl_ptr->width = 1;
    pcl_ptr->height = point_cloud->point_size();
  }
  pcl_ptr->points.resize(point_cloud->point_size());

  for (size_t i = 0; i < pcl_ptr->points.size(); ++i) {
    const auto &point = point_cloud->point(static_cast<int>(i));
    pcl_ptr->points[i].x = point.x();
    pcl_ptr->points[i].y = point.y();
    pcl_ptr->points[i].z = point.z();
  }

  TransformPointCloud(pcl_ptr, point_cloud->header().frame_id());

  return pcl_ptr;
}

void PointCloudUpdater::UpdatePointCloud(
    const std::shared_ptr<drivers::PointCloud> &point_cloud,
    const std::string& channel_name) {
  PointCloudChannelUpdater *updater = GetPointCloudChannelUpdater(channel_name);
  if (point_cloud->header().has_timestamp_sec()) {
    updater->last_point_cloud_time_ = point_cloud->header().timestamp_sec();
  } else {
    updater->last_point_cloud_time_ = point_cloud->measurement_time();
  }
  if (std::fabs(last_localization_time_ - updater->last_point_cloud_time_) >
      0.1) {
    AWARN << "skipping outdated point cloud data";
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr;
  // Check if last filter process has finished before processing new data.
  if (enable_voxel_filter_) {
    if (updater->future_ready_) {
      updater->future_ready_ = false;
      // transform from drivers::PointCloud to pcl::PointCloud
      pcl_ptr = ConvertPCLPointCloud(point_cloud, channel_name);
      std::future<void> f = cyber::Async(&PointCloudUpdater::FilterPointCloud,
                                         this, pcl_ptr, channel_name);
      updater->async_future_ = std::move(f);
    }
  } else {
    pcl_ptr = ConvertPCLPointCloud(point_cloud, channel_name);
    this->FilterPointCloud(pcl_ptr, channel_name);
  }
}

void PointCloudUpdater::FilterPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr,
    const std::string &channel_name) {
  PointCloudChannelUpdater *updater = GetPointCloudChannelUpdater(channel_name);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filtered_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);

  /*
      By default, disable voxel filter since it's taking more than 500ms
      ideally the most efficient sampling method is to
      use per beam random sample for organized cloud(TODO)
  */
  if (enable_voxel_filter_) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(pcl_ptr);
    voxel_grid.setLeafSize(static_cast<float>(FLAGS_voxel_filter_size),
                           static_cast<float>(FLAGS_voxel_filter_size),
                           static_cast<float>(FLAGS_voxel_filter_height));
    voxel_grid.filter(*pcl_filtered_ptr);
  } else {
    pcl_filtered_ptr = pcl_ptr;
  }

  apollo::dreamview::PointCloud point_cloud_pb;
  for (size_t idx = 0; idx < pcl_filtered_ptr->size(); ++idx) {
    pcl::PointXYZ &pt = pcl_filtered_ptr->points[idx];
    if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
      point_cloud_pb.add_num(pt.x);
      point_cloud_pb.add_num(pt.y);
      point_cloud_pb.add_num(pt.z);
    }
  }
  {
    boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
    point_cloud_pb.SerializeToString(&updater->point_cloud_str_);
    updater->future_ready_ = true;
  }
}

void PointCloudUpdater::UpdateLocalizationTime(
    const std::shared_ptr<LocalizationEstimate> &localization) {
  last_localization_time_ = localization->header().timestamp_sec();
}

void PointCloudUpdater::TransformPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud,
    const std::string &frame_id) {
  if (frame_id.empty()) {
    AERROR << "Failed to get frame id";
    return;
  }
  apollo::transform::Buffer *tf2_buffer = apollo::transform::Buffer::Instance();
  apollo::transform::TransformStamped stamped_transform;
  try {
    stamped_transform = tf2_buffer->lookupTransform("localization", frame_id,
                                                    apollo::cyber::Time(0));
  } catch (tf2::TransformException &ex) {
    AERROR << ex.what();
    AERROR << "Failed to get matrix of frame id: " << frame_id;
    return;
  }

  auto &matrix = stamped_transform.transform();

  // Define a translation vector
  Eigen::Vector3f translation(
      matrix.translation().x(), matrix.translation().y(),
      matrix.translation().z());  // Replace with your actual values

  // Define a quaternion for rotation
  Eigen::Quaternionf rotation(
      matrix.rotation().qw(), matrix.rotation().qx(), matrix.rotation().qy(),
      matrix.rotation().qz());  // Replace with your actual values (w, x, y, z)
  rotation.normalize();  // Ensure the quaternion is a valid unit quaternion

  // Combine into a transformation matrix
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << translation;
  transform.rotate(rotation);

  // Transform the point cloud
  pcl::transformPointCloud(*point_cloud, *point_cloud, transform);
  // Define the rotation matrix for 90 degree counter-clockwise rotation in
  // the xy-plane
  Eigen::Matrix4f counter_transform = Eigen::Matrix4f::Identity();
  counter_transform(0, 0) = 0;
  counter_transform(0, 1) = 1;
  counter_transform(1, 0) = -1;
  counter_transform(1, 1) = 0;

  // Perform the transformation
  pcl::transformPointCloud(*point_cloud, *point_cloud, counter_transform);
}

}  // namespace dreamview
}  // namespace apollo
