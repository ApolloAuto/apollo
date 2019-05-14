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

#include "modules/dreamview/backend/point_cloud/point_cloud_updater.h"

#include <utility>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/time/time.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/dreamview/proto/point_cloud.pb.h"
#include "pcl/filters/voxel_grid.h"
#include "third_party/json/json.hpp"
#include "yaml-cpp/yaml.h"

namespace apollo {
namespace dreamview {

using apollo::localization::LocalizationEstimate;
using Json = nlohmann::json;

float PointCloudUpdater::lidar_height_ = kDefaultLidarHeight;
boost::shared_mutex PointCloudUpdater::mutex_;

PointCloudUpdater::PointCloudUpdater(WebSocketHandler *websocket)
    : node_(cyber::CreateNode("point_cloud")),
      websocket_(websocket),
      point_cloud_str_(""),
      future_ready_(true) {
  RegisterMessageHandlers();
}

PointCloudUpdater::~PointCloudUpdater() { Stop(); }

void PointCloudUpdater::LoadLidarHeight(const std::string &file_path) {
  if (!cyber::common::PathExists(file_path)) {
    AWARN << "No such file: " << FLAGS_lidar_height_yaml
          << ". Using default lidar height:" << kDefaultLidarHeight;
    boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
    lidar_height_ = kDefaultLidarHeight;
    return;
  }

  YAML::Node config = YAML::LoadFile(file_path);
  if (config["vehicle"] && config["vehicle"]["parameters"]) {
    boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
    lidar_height_ = config["vehicle"]["parameters"]["height"].as<float>();
    AINFO << "Lidar height is updated to " << lidar_height_;
    return;
  }

  AWARN << "Fail to load the lidar height yaml file: "
        << FLAGS_lidar_height_yaml
        << ". Using default lidar height:" << kDefaultLidarHeight;
  boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
  lidar_height_ = kDefaultLidarHeight;
}

void PointCloudUpdater::RegisterMessageHandlers() {
  // Send current point_cloud status to the new client.
  websocket_->RegisterConnectionReadyHandler(
      [this](WebSocketHandler::Connection *conn) {
        Json response;
        response["type"] = "PointCloudStatus";
        response["enabled"] = enabled_;
        websocket_->SendData(conn, response.dump());
      });
  websocket_->RegisterMessageHandler(
      "RequestPointCloud",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        std::string to_send;
        // If there is no point_cloud data for more than 2 seconds, reset.
        if (point_cloud_str_ != "" &&
            std::fabs(last_localization_time_ - last_point_cloud_time_) > 2.0) {
          boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
          point_cloud_str_ = "";
        }
        {
          boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
          to_send = point_cloud_str_;
        }
        websocket_->SendBinaryData(conn, to_send, true);
      });
  websocket_->RegisterMessageHandler(
      "TogglePointCloud",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto enable = json.find("enable");
        if (enable != json.end() && enable->is_boolean()) {
          if (*enable) {
            enabled_ = true;
          } else {
            enabled_ = false;
          }
          if (websocket_) {
            Json response;
            response["type"] = "PointCloudStatus";
            response["enabled"] = enabled_;
            // Sync the point_cloud status across all the clients.
            websocket_->BroadcastData(response.dump());
          }
        }
      });
}

void PointCloudUpdater::Start() {
  localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      FLAGS_localization_topic,
      [this](const std::shared_ptr<LocalizationEstimate> &msg) {
        UpdateLocalizationTime(msg);
      });
  point_cloud_reader_ = node_->CreateReader<drivers::PointCloud>(
      FLAGS_pointcloud_topic,
      [this](const std::shared_ptr<drivers::PointCloud> &msg) {
        UpdatePointCloud(msg);
      });

  LoadLidarHeight(FLAGS_lidar_height_yaml);
}

void PointCloudUpdater::Stop() {
  if (enabled_) {
    async_future_.wait();
  }
}

void PointCloudUpdater::UpdatePointCloud(
    const std::shared_ptr<drivers::PointCloud> &point_cloud) {
  if (!enabled_) {
    return;
  }

  last_point_cloud_time_ = point_cloud->header().timestamp_sec();
  // Check if last filter process has finished before processing new data.
  if (future_ready_) {
    future_ready_ = false;
    // transform from drivers::PointCloud to pcl::PointCloud
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
    std::future<void> f =
        cyber::Async(&PointCloudUpdater::FilterPointCloud, this, pcl_ptr);
    async_future_ = std::move(f);
  }
}

void PointCloudUpdater::FilterPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr) {
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(pcl_ptr);
  voxel_grid.setLeafSize(static_cast<float>(FLAGS_voxel_filter_size),
                         static_cast<float>(FLAGS_voxel_filter_size),
                         static_cast<float>(FLAGS_voxel_filter_height));
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filtered_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid.filter(*pcl_filtered_ptr);
  AINFO << "filtered point cloud data size: " << pcl_filtered_ptr->size();

  float z_offset;
  {
    boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
    z_offset = lidar_height_;
  }
  apollo::dreamview::PointCloud point_cloud_pb;
  for (size_t idx = 0; idx < pcl_filtered_ptr->size(); ++idx) {
    pcl::PointXYZ &pt = pcl_filtered_ptr->points[idx];
    if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
      point_cloud_pb.add_num(pt.x);
      point_cloud_pb.add_num(pt.y);
      point_cloud_pb.add_num(pt.z + z_offset);
    }
  }
  {
    boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
    point_cloud_pb.SerializeToString(&point_cloud_str_);
    future_ready_ = true;
  }
}

void PointCloudUpdater::UpdateLocalizationTime(
    const std::shared_ptr<LocalizationEstimate> &localization) {
  last_localization_time_ = localization->header().timestamp_sec();
}

}  // namespace dreamview
}  // namespace apollo
