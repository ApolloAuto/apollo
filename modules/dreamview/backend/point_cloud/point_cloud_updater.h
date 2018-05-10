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

#ifndef MODULES_DREAMVIEW_BACKEND_POINT_CLOUD_POINT_CLOUD_UPDATER_H_
#define MODULES_DREAMVIEW_BACKEND_POINT_CLOUD_POINT_CLOUD_UPDATER_H_

#include <string>

#include "boost/thread/locks.hpp"
#include "boost/thread/shared_mutex.hpp"

#include "modules/common/log.h"
#include "modules/common/util/string_util.h"
#include "modules/dreamview/backend/handlers/websocket_handler.h"
#include "modules/dreamview/proto/point_cloud.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "sensor_msgs/PointCloud2.h"

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

  /**
   * @brief Starts to push PointCloud to frontend.
   */
  void Start();

 private:
  void RegisterMessageHandlers();

  void UpdatePointCloud(const sensor_msgs::PointCloud2 &point_cloud);

  void UpdateLocalizationTime(
      const apollo::localization::LocalizationEstimate &localization);

  WebSocketHandler *websocket_;

  bool enabled_ = false;

  // The PointCloud to be pushed to frontend.
  std::string point_cloud_str_;
  PointCloud point_cloud_;

  // Mutex to protect concurrent access to point_cloud_str_.
  // NOTE: Use boost until we have std version of rwlock support.
  boost::shared_mutex mutex_;

  double last_point_cloud_time_ = 0.0;
  double last_localization_time_ = 0.0;
};

}  // namespace dreamview
}  // namespace apollo

#endif  // MODULES_DREAMVIEW_BACKEND_POINT_CLOUD_POINT_CLOUD_UPDATER_H_
