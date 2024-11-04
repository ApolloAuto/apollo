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

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <google/protobuf/util/message_differencer.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/common_msgs/transform_msgs/transform.pb.h"
#include "modules/dreamview_plus/proto/data_handler.pb.h"

#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "modules/transform/buffer.h"
#include "modules/common/util/string_util.h"
#include "modules/dreamview/backend/common/handlers/websocket_handler.h"
#include "modules/dreamview_plus/backend/updater/updater_with_channels_base.h"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

struct PointCloudChannelUpdater {
  std::string curr_channel_name_;
  std::shared_ptr<cyber::Reader<drivers::PointCloud>> point_cloud_reader_;
  double last_point_cloud_time_;
  // The PointCloud to be pushed to frontend.
  std::string point_cloud_str_;
  std::unique_ptr<cyber::Timer> timer_;
  std::atomic<bool> future_ready_;
  std::future<void> async_future_;
  explicit PointCloudChannelUpdater(std::string channel_name)
      : curr_channel_name_(channel_name),
        point_cloud_reader_(nullptr),
        last_point_cloud_time_(0.0),
        point_cloud_str_(""),
        future_ready_(true) {}
};

/**
 * @class PointCloudUpdater
 * @brief A wrapper around WebSocketHandler to keep pushing PointCloud to
 * frontend via websocket while handling the response from frontend.
 */
class PointCloudUpdater : public UpdaterWithChannelsBase {
 public:
  /**
   * @brief Constructor with the websocket handler.
   * @param websocket Pointer of the websocket handler that has been attached to
   * the server.
   * @param simulationworldupdater pointer
   */
  explicit PointCloudUpdater(WebSocketHandler *websocket);

  ~PointCloudUpdater();
  // dreamview callback function
  // using DvCallback = std::function<bool(const std::string &string)>;

  /**
   * @brief Starts to push PointCloud to frontend.
   */
  void Stop();
  void StartStream(const double &time_interval_ms,
                   const std::string &channel_name = "",
                   nlohmann::json *subscribe_param = nullptr) override;
  void StopStream(const std::string& channel_name = "") override;
  void OnTimer(const std::string& channel_name = "");
  void PublishMessage(const std::string& channel_name = "") override;
  void GetChannelMsg(std::vector<std::string> *channels) override;
  // The height of lidar w.r.t the ground.
  static float lidar_height_;

  // Mutex to protect concurrent access to point_cloud_str_ and lidar_height_.
  // NOTE: Use boost until we have std version of rwlock support.
  static boost::shared_mutex mutex_;

 private:
  void UpdatePointCloud(const std::shared_ptr<drivers::PointCloud> &point_cloud,
                        const std::string &channel_name);

  void FilterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr,
                        const std::string &channel_name);

  void UpdateLocalizationTime(
      const std::shared_ptr<apollo::localization::LocalizationEstimate>
          &localization);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ConvertPCLPointCloud(
      const std::shared_ptr<drivers::PointCloud> &point_cloud,
      const std::string &channel_name);

  PointCloudChannelUpdater* GetPointCloudChannelUpdater(
    const std::string &channel_name);

  void TransformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud,
                           const std::string &frame_id);

  constexpr static float kDefaultLidarHeight = 1.91f;

  std::unique_ptr<cyber::Node> node_;

  WebSocketHandler *websocket_;

  bool enabled_ = false;


  // Cyber messsage readers.
  std::shared_ptr<cyber::Reader<apollo::localization::LocalizationEstimate>>
      localization_reader_;
  std::map<std::string, PointCloudChannelUpdater *> channel_updaters_;
  double last_localization_time_ = 0.0;
  bool enable_voxel_filter_ = false;
  // DvCallback callback_api_;
  std::mutex channel_updater_map_mutex_;
};
}  // namespace dreamview
}  // namespace apollo
