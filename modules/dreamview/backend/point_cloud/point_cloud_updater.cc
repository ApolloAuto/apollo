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

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "third_party/json/json.hpp"

namespace apollo {
namespace dreamview {

using apollo::common::adapter::AdapterManager;
using sensor_msgs::PointCloud2;
using Json = nlohmann::json;

PointCloudUpdater::PointCloudUpdater(WebSocketHandler *websocket)
    : websocket_(websocket) {
  RegisterMessageHandlers();
}

void PointCloudUpdater::RegisterMessageHandlers() {
  websocket_->RegisterMessageHandler(
      "RequestPointCloud",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        std::string to_send;
        {
          // Pay the price to copy the data instead of sending data over the
          // wire while holding the lock.
          boost::shared_lock<boost::shared_mutex> lock(mutex_);
          to_send = point_cloud_str_;
        }
        websocket_->SendData(conn, to_send, true);
      });
}

void PointCloudUpdater::Start() {
  AdapterManager::AddPointCloudCallback(&PointCloudUpdater::UpdatePointCloud,
                                        this);
}

void PointCloudUpdater::UpdatePointCloud(const PointCloud2 &point_cloud) {
  // transform from ros to pcl
  pcl::PointCloud < pcl::PointXYZ > pcl_data;
  pcl::fromROSMsg(point_cloud, pcl_data);

  if (pcl_data.size() == 0) {
    point_cloud_str_ = "[]";
    return;
  }

  std::stringstream stream;
  std::string data("[");
  for (size_t idx = 0; idx < pcl_data.size(); idx += 2) {
    pcl::PointXYZ& pt = pcl_data.points[idx];
    if (!isnan(pt.x) && !isnan(pt.y) && !isnan(pt.z)) {
      data.append("[");
      stream.str(std::string());
      stream << std::fixed << std::setprecision(4) << pt.x;
      data.append(stream.str());
      data.append(",");
      stream.str(std::string());
      stream << std::fixed << std::setprecision(4) << pt.y;
      data.append(stream.str());
      data.append(",");
      stream.str(std::string());
      stream << std::fixed << std::setprecision(4) << pt.z;
      data.append(stream.str());
      data.append("]");
      if ((idx + 1) == pcl_data.size()) {
        data.append("]");
      } else {
        data.append(",");
      }
    }
  }
  boost::shared_lock<boost::shared_mutex> lock(mutex_);
  point_cloud_str_ = data;
}

}  // namespace dreamview
}  // namespace apollo
