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
using Json = nlohmann::json;

float PointCloudUpdater::lidar_height_ = kDefaultLidarHeight;
boost::shared_mutex PointCloudUpdater::mutex_;

PointCloudUpdater::PointCloudUpdater(WebSocketHandler *websocket,
                                     SimulationWorldUpdater *simworld_updater)
    : node_(cyber::CreateNode("point_cloud")),
      websocket_(websocket),
      point_cloud_str_(""),
      future_ready_(true),
      simworld_updater_(simworld_updater) {
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
  websocket_->RegisterMessageHandler(
      "GetPointCloudChannel",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        std::vector<std::string> channels;
        GetChannelMsg(&channels);
        Json response({});
        response["data"]["name"] = "GetPointCloudChannelListSuccess";
        for (unsigned int i = 0; i < channels.size(); i++) {
          response["data"]["info"]["channel"][i] = channels[i];
        }
        websocket_->SendData(conn, response.dump());
      });
  websocket_->RegisterMessageHandler(
      "ChangePointCloudChannel",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto channel_info = json.find("data");
        Json response({});
        if (channel_info == json.end()) {
          AERROR << "Cannot  retrieve point cloud channel info with unknown "
                    "channel.";
          response["type"] = "ChangePointCloudChannelFail";
          websocket_->SendData(conn, response.dump());
          return;
        }
        std::string channel =
            channel_info->dump().substr(1, channel_info->dump().length() - 2);
        if (ChangeChannel(channel)) {
          Json response({});
          response["type"] = "ChangePointCloudChannelSuccess";
          websocket_->SendData(conn, response.dump());
        } else {
          response["type"] = "ChangePointCloudChannelFail";
          websocket_->SendData(conn, response.dump());
        }
      });
}

void PointCloudUpdater::GetChannelMsg(std::vector<std::string> *channels) {
  enabled_ = true;
  auto channelManager =
      apollo::cyber::service_discovery::TopologyManager::Instance()
          ->channel_manager();
  std::vector<apollo::cyber::proto::RoleAttributes> role_attr_vec;
  channelManager->GetWriters(&role_attr_vec);
  for (auto &role_attr : role_attr_vec) {
    std::string messageType;
    messageType = role_attr.message_type();
    int index = messageType.rfind("PointCloud");
    int index_sensor = role_attr.channel_name().rfind("sensor");
    if (index != -1 && index_sensor != -1) {
      channels->push_back(role_attr.channel_name());
    }
  }
  channels_.clear();
  channels_ = {channels->begin(), channels->end()};
}

bool PointCloudUpdater::ChangeChannel(const std::string &channel) {
  if (curr_channel_name != "") {
    if (!node_->DeleteReader(curr_channel_name)) {
      AERROR << "delete reader failed";
      return false;
    }
  }
  point_cloud_reader_.reset();
  point_cloud_reader_ = node_->CreateReader<drivers::PointCloud>(
      channel, [this](const std::shared_ptr<drivers::PointCloud> &msg) {
        UpdatePointCloud(msg);
      });

  if (point_cloud_reader_ == nullptr) {
    return false;
  }
  curr_channel_name = channel;
  bool update_res = false;
  update_res = callback_api_(channel);
  if (!update_res) {
    AERROR << "update current point cloud channel fail";
    return false;
  }
  return true;
}

void PointCloudUpdater::Start(DvCallback callback_api) {
  callback_api_ = callback_api;
  localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      FLAGS_localization_topic,
      [this](const std::shared_ptr<LocalizationEstimate> &msg) {
        UpdateLocalizationTime(msg);
      });
  LoadLidarHeight(FLAGS_lidar_height_yaml);
}

void PointCloudUpdater::Stop() {
  if (enabled_) {
    async_future_.wait();
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudUpdater::ConvertPCLPointCloud(
    const std::shared_ptr<drivers::PointCloud> &point_cloud) {
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
  return pcl_ptr;
}

void PointCloudUpdater::UpdatePointCloud(
    const std::shared_ptr<drivers::PointCloud> &point_cloud) {
  if (!enabled_) {
    return;
  }
  if (point_cloud->header().has_timestamp_sec()) {
    last_point_cloud_time_ = point_cloud->header().timestamp_sec();
  } else {
    last_point_cloud_time_ = point_cloud->measurement_time();
  }
  if (simworld_updater_->LastAdcTimestampSec() == 0.0 ||
      simworld_updater_->LastAdcTimestampSec() - last_point_cloud_time_ > 0.1) {
    AWARN << "skipping outdated point cloud data";
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr;
  // Check if last filter process has finished before processing new data.
  if (enable_voxel_filter_) {
    if (future_ready_) {
      future_ready_ = false;
      // transform from drivers::PointCloud to pcl::PointCloud
      pcl_ptr = ConvertPCLPointCloud(point_cloud);
      std::future<void> f =
          cyber::Async(&PointCloudUpdater::FilterPointCloud, this, pcl_ptr);
      async_future_ = std::move(f);
    }
  } else {
    pcl_ptr = ConvertPCLPointCloud(point_cloud);
    this->FilterPointCloud(pcl_ptr);
  }
}

void PointCloudUpdater::FilterPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr) {
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
