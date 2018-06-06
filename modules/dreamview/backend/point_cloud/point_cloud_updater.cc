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

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/dreamview/proto/point_cloud.pb.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"
#include "third_party/json/json.hpp"

namespace apollo {
namespace dreamview {

using apollo::common::adapter::AdapterManager;
using apollo::common::time::Clock;
using apollo::localization::LocalizationEstimate;
using sensor_msgs::PointCloud2;
using Json = nlohmann::json;

PointCloudUpdater::PointCloudUpdater(WebSocketHandler *websocket)
    : websocket_(websocket), point_cloud_str_(""), future_ready_(true) {
  RegisterMessageHandlers();
}

PointCloudUpdater::~PointCloudUpdater() { Stop(); }

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
  AdapterManager::AddPointCloudCallback(&PointCloudUpdater::UpdatePointCloud,
                                        this);
  AdapterManager::AddLocalizationCallback(
      &PointCloudUpdater::UpdateLocalizationTime, this);
}

void PointCloudUpdater::Stop() {
  if (enabled_) {
    async_future_.wait();
  }
}

void PointCloudUpdater::UpdatePointCloud(const PointCloud2 &point_cloud) {
  if (!enabled_) {
    return;
  }

  last_point_cloud_time_ = point_cloud.header.stamp.toSec();
  // Check if last filter process has finished before processing new data.
  if (future_ready_) {
    future_ready_ = false;
    // transform from ros to pcl
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(point_cloud, *pcl_ptr);
    std::future<void> f =
        std::async(std::launch::async, &PointCloudUpdater::FilterPointCloud,
                   this, pcl_ptr);
    async_future_ = std::move(f);
  }
}

void PointCloudUpdater::FilterPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr) {
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(pcl_ptr);
  voxel_grid.setLeafSize(FLAGS_voxel_filter_size, FLAGS_voxel_filter_size,
                         FLAGS_voxel_filter_height);
  voxel_grid.filter(*pcl_ptr);
  AINFO << "filtered point cloud data size: " << pcl_ptr->size();

  PointCloud point_cloud_pb;
  for (size_t idx = 0; idx < pcl_ptr->size(); ++idx) {
    pcl::PointXYZ &pt = pcl_ptr->points[idx];
    if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
      point_cloud_pb.add_num(pt.x);
      point_cloud_pb.add_num(pt.y);
      // TODO(unacao): velodyne height should be updated by hmi store
      // upon vehicle change.
      point_cloud_pb.add_num(pt.z + 1.91f);
    }
  }
  {
    boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
    point_cloud_pb.SerializeToString(&point_cloud_str_);
    future_ready_ = true;
  }
}

void PointCloudUpdater::UpdateLocalizationTime(
    const LocalizationEstimate &localization) {
  last_localization_time_ = localization.header().timestamp_sec();
}

}  // namespace dreamview
}  // namespace apollo
