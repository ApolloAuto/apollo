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

/**
 * @file
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"

#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "modules/common/util/string_util.h"
#include "modules/dreamview/backend/common/handlers/websocket_handler.h"
#include "modules/dreamview/backend/simulation_world/simulation_world_updater.h"
/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

/**
 * @class PointCloudUpdater
 * @brief A wrapper around WebSocketHandler to keep pushing PointCloud to
 * frontend via websocket while handling the response from frontend.
 */
class PointCloudUpdater {
 public:
  /**
   * @brief Constructor with the websocket handler.
   * @param websocket Pointer of the websocket handler that has been attached to
   * the server.
   * @param simulationworldupdater pointer
   */
  PointCloudUpdater(WebSocketHandler *websocket,
                    SimulationWorldUpdater *sim_world_updater);

  ~PointCloudUpdater();
  // dreamview callback function
  using DvCallback = std::function<bool(const std::string &string)>;
  static void LoadLidarHeight(const std::string &file_path);

  /**
   * @brief Starts to push PointCloud to frontend.
   */
  void Start(DvCallback callback_api);
  void Stop();
  // The height of lidar w.r.t the ground.
  static float lidar_height_;

  // Mutex to protect concurrent access to point_cloud_str_ and lidar_height_.
  // NOTE: Use boost until we have std version of rwlock support.
  static boost::shared_mutex mutex_;

 private:
  void RegisterMessageHandlers();

  void UpdatePointCloud(
      const std::shared_ptr<drivers::PointCloud> &point_cloud);

  void FilterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr);

  void UpdateLocalizationTime(
      const std::shared_ptr<apollo::localization::LocalizationEstimate>
          &localization);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ConvertPCLPointCloud(
      const std::shared_ptr<drivers::PointCloud> &point_cloud);

  void GetChannelMsg(std::vector<std::string> *channels);
  bool ChangeChannel(const std::string &channel);
  constexpr static float kDefaultLidarHeight = 1.91f;

  std::unique_ptr<cyber::Node> node_;

  WebSocketHandler *websocket_;

  std::vector<std::string> channels_;

  bool enabled_ = false;

  // The PointCloud to be pushed to frontend.
  std::string point_cloud_str_;

  std::future<void> async_future_;
  std::atomic<bool> future_ready_;

  // Cyber messsage readers.
  std::shared_ptr<cyber::Reader<apollo::localization::LocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<cyber::Reader<drivers::PointCloud>> point_cloud_reader_;
  double last_point_cloud_time_ = 0.0;
  double last_localization_time_ = 0.0;
  SimulationWorldUpdater *simworld_updater_;
  bool enable_voxel_filter_ = false;
  std::string curr_channel_name = "";
  DvCallback callback_api_;
};
}  // namespace dreamview
}  // namespace apollo
