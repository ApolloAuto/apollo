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

#ifndef MODULES_DRIVERS_VELODYNE_VELODYNE_H_
#define MODULES_DRIVERS_VELODYNE_VELODYNE_H_

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "ros/include/sensor_msgs/PointCloud2.h"
#include "ros/include/velodyne_msgs/VelodyneScanUnified.h"

#include "modules/drivers/lidar_velodyne/proto/velodyne_conf.pb.h"

#include "modules/common/apollo_app.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/time/time.h"
#include "modules/common/util/blocking_queue.h"
#include "modules/common/util/util.h"

/**
 * @namespace apollo::velodyne
 * @brief apollo::velodyne
 */
namespace apollo {
namespace drivers {
namespace lidar_velodyne {

/**
 * @class Velodyne
 *
 * @brief velodyne module main class
 */
class Velodyne : public apollo::common::ApolloApp {
  friend class VelodyneTestBase;

 public:
  Velodyne() : monitor_logger_(common::monitor::MonitorMessageItem::CONTROL) {}

  std::string Name() const override;
  apollo::common::Status Init() override;
  apollo::common::Status Start() override;
  void Stop() override;

  virtual ~Velodyne() = default;

 private:
  typedef common::util::BlockingQueue<velodyne_msgs::VelodyneScanUnifiedPtr>
      RawDataCache;
  typedef common::util::BlockingQueue<sensor_msgs::PointCloud2Ptr>
      PointCloudCache;
  void Packet(RawDataCache* output);
  void Convert(RawDataCache* input, PointCloudCache* output);
  void Compensate(PointCloudCache* input);
  bool SetNpackets(VelodyneConf* conf);
  inline int64_t GetTime() {
    return apollo::common::time::AsInt64<common::time::micros>(
        apollo::common::time::Clock::Now());
  }
  void Notice();

 private:
  VelodyneConf conf_;
  common::monitor::MonitorLogger monitor_logger_;
  std::shared_ptr<RawDataCache> packet_cache_;
  std::shared_ptr<PointCloudCache> pointcloud_cache_;

  bool running_ = true;
  std::vector<std::shared_ptr<std::thread> > threads_;
};

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_VELODYNE_VELODYNE_H_
