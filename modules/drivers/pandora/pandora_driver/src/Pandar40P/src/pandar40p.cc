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

#include "pandar40p/pandar40p.h"
#include "src/pandar40p_internal.h"

namespace apollo {
namespace drivers {
namespace hesai {

/**
 * @brief Constructor
 * @param device_ip         The ip of the device
 *        lidar_port        The port number of lidar data
 *        gps_port          The port number of gps data
 *        pcl_callback      The callback of PCL data structure
 *        gps_callback      The callback of GPS structure
 *        start_angle       The start angle of every point cloud
 */
Pandar40P::Pandar40P(
    std::string device_ip, uint16_t lidar_port, uint16_t gps_port,
    boost::function<void(boost::shared_ptr<PPointCloud>, double)> pcl_callback,
    boost::function<void(double)> gps_callback, uint16_t start_angle, int tz,
    std::string frame_id) {
  internal_ =
      new Pandar40P_Internal(device_ip, lidar_port, gps_port, pcl_callback,
                             gps_callback, start_angle, tz, frame_id);
}

/**
 * @brief deconstructor
 */
Pandar40P::~Pandar40P() { delete internal_; }

/**
 * @brief load the lidar correction file
 * @param contents The correction contents of lidar correction
 */
int Pandar40P::LoadCorrectionFile(std::string file) {
  return internal_->LoadCorrectionFile(file);
}

/**
 * @brief Reset Lidar's start angle.
 * @param angle The start angle
 */
void Pandar40P::ResetStartAngle(uint16_t start_angle) {
  internal_->ResetStartAngle(start_angle);
}

/**
 * @brief Run SDK.
 */
int Pandar40P::Start() { return internal_->Start(); }

/**
 * @brief Stop SDK.
 */
void Pandar40P::Stop() { internal_->Stop(); }

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo
