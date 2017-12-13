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

#ifndef MODEULES_PERCEPTION_TOOL_EXPORT_SENSOR_DATA_EXPORT_SENSOR_DATA_H_
#define MODEULES_PERCEPTION_TOOL_EXPORT_SENSOR_DATA_EXPORT_SENSOR_DATA_H_

#include <boost/circular_buffer.hpp>
#include <memory>
#include <string>
#include <utility>

#include "modules/common/apollo_app.h"
#include "modules/common/macro.h"
#include "modules/perception/onboard/dag_streaming.h"
#include "ros/include/ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "modules/perception/obstacle/radar/modest/conti_radar_id_expansion.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/perception/lib/pcl_util/pcl_types.h"

/**
 * @namespace apollo::perception
 * @brief apollo::perception
 */
namespace apollo {
namespace perception {

class ExportSensorData{
 public:
  std::string Name() const;
  common::Status Init();

 private:
  void OnPointCloud(const sensor_msgs::PointCloud2& message);
  void OnRadar(const ContiRadar &radar_obs);
  void OnGps(const apollo::localization::Gps &gps);
  bool GetCarLinearSpeed(double timestamp,
    Eigen::Vector3f *car_linear_speed);
  void WriteRadar(const std::string &file_pre, const ContiRadar &radar_obs);
  void WritePose(const std::string &file_pre,
    const double timestamp, const int seq_num,
    const Eigen::Matrix4d& pose);
  void WriteVelocityInfo(const std::string &file_pre,
    const double& timestamp, const int seq_num,
    const Eigen::Vector3f& velocity);
  void WritePCD(const std::string &file_pre,
     const sensor_msgs::PointCloud2& in_msg);
  void TransPointCloudToPCL(
    const sensor_msgs::PointCloud2& in_msg,
    pcl_util::PointCloudPtr* out_cloud);
  typedef std::pair<double, apollo::localization::Gps> ObjectPair;
  boost::circular_buffer<ObjectPair> gps_buffer_;
  ContiRadarIDExpansion _conti_id_expansion;
  Mutex mutex_;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODEULES_PERCEPTION_TOOL_EXPORT_SENSOR_DATA_EXPORT_SENSOR_DATA_H_

