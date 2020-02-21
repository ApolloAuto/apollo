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

#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <memory>
#include <string>

#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "modules/common/util/string_util.h"
#include "modules/dreamview/backend/handlers/websocket_handler.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

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
   */
  explicit PointCloudUpdater(WebSocketHandler *websocket);
  ~PointCloudUpdater();

  static void LoadLidarHeight(const std::string &file_path);

  /**
   * @brief Starts to push PointCloud to frontend.
   */
  void Start();
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

  constexpr static float kDefaultLidarHeight = 1.91f;

  std::unique_ptr<cyber::Node> node_;

  WebSocketHandler *websocket_;

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
};

}  // namespace dreamview
}  // namespace apollo
