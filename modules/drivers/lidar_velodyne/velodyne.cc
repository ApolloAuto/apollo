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

#include "modules/drivers/lidar_velodyne/velodyne.h"

#include <memory>
#include <string>

#include "ros/include/std_msgs/String.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/drivers/lidar_velodyne/common/velodyne_gflags.h"
#include "modules/drivers/lidar_velodyne/driver/driver.h"
#include "modules/drivers/lidar_velodyne/pointcloud/compensator.h"
#include "modules/drivers/lidar_velodyne/pointcloud/converter.h"
#include "modules/drivers/lidar_velodyne/pointcloud/velodyne_adapter.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

using apollo::common::adapter::AdapterManager;
using apollo::common::ErrorCode;
using apollo::common::time::Clock;
using apollo::common::Status;
using apollo::common::util::GetProtoFromFile;
using apollo::common::util::BlockingQueue;
using velodyne_msgs::VelodyneScanUnifiedPtr;

static const int TAKE_TIME_OUT = 10;
static const int NOTICE_LOG_INTERVAL = 1000;
static const int POLL_LOG_INTERVAL = 100000;

std::string Velodyne::Name() const { return FLAGS_velodyne_module_name; }

Status Velodyne::Init() {
  AINFO << "Velodyne init, starting ...";

  if (!GetProtoFromFile(FLAGS_velodyne_conf_file, &conf_)) {
    std::string msg = "Load velodyne conf fail:" + FLAGS_velodyne_conf_file;
    AERROR << msg;
    return Status(ErrorCode::DRIVER_ERROR_VELODYNE, msg);
  }
  AINFO << "Conf file: " << FLAGS_velodyne_conf_file << " is loaded.";
  if (!SetNpackets(&conf_)) {
    AERROR << "calc npackets fail.";
    return Status(ErrorCode::DRIVER_ERROR_VELODYNE, "init npackets fail.");
  }

  AdapterManager::Init(FLAGS_velodyne_adapter_config_filename);
  common::monitor::MonitorLogBuffer buffer(&monitor_logger_);

  if (!VelodyneAdapter::CheckVelodyne()) {
    return Status(ErrorCode::DRIVER_ERROR_VELODYNE, "Adapter not initialized");
  }

  Notice();

  int size = conf_.cache_size();
  if (FLAGS_pipeline_mode) {
    size += conf_.npackets();
  }

  packet_cache_.reset(new RawDataCache);
  packet_cache_->init(size, "packet_cache");
  pointcloud_cache_.reset(new PointCloudCache);
  pointcloud_cache_->init(conf_.cache_size(), "pointcloud_cache");

  return Status::OK();
}

void Velodyne::Packet(RawDataCache* output) {
  AINFO << "start packet thread.";
  VelodyneDriver* dvr = VelodyneDriverFactory::create_driver(conf_);
  if (nullptr == dvr || !dvr->init()) {
    AERROR << "Create or init driver fail.";
    return;
  }

  velodyne_msgs::VelodyneScanUnifiedPtr full_scan(
      new velodyne_msgs::VelodyneScanUnified());
  full_scan->packets.resize(conf_.npackets());
  int scan_idx = 0;

  while (running_) {
    velodyne_msgs::VelodyneScanUnifiedPtr scan(
        new velodyne_msgs::VelodyneScanUnified());
    if (!dvr->poll(scan)) {
      AINFO_EVERY(POLL_LOG_INTERVAL) << "poll data fail.";
      continue;
    }

    int64_t t0 = GetTime();
    AINFO_EVERY(NOTICE_LOG_INTERVAL) << "packet raw data ...";
    output->put(scan);

    velodyne_msgs::VelodyneScanUnifiedPtr pub = scan;
    if (FLAGS_pipeline_mode) {
      pub = full_scan;
      memcpy(&(full_scan->packets[scan_idx].data[0]),
             &(scan->packets[0].data[0]), FIRING_DATA_PACKET_SIZE);
      full_scan->packets[scan_idx].stamp = scan->packets[0].stamp;
      ++scan_idx;

      if (scan_idx != conf_.npackets()) {
        continue;
      }
      full_scan->header = scan->header;
      full_scan->basetime = scan->basetime;
    }

    if (FLAGS_publish_raw_data) {
      VelodyneAdapter::PublishVelodyneScanByIndex(conf_.index(), pub);
    }

    if (FLAGS_pipeline_mode) {
      scan_idx = 0;
      full_scan.reset(new velodyne_msgs::VelodyneScanUnified());
      full_scan->packets.resize(conf_.npackets());
    }
    AINFO << "CALC raw packet done.";

    int64_t t1 = GetTime();
    ADEBUG << "packet total time: " << t1 - t0;
  }

  AINFO << "end of packet thread func.";
  return;
}

void Velodyne::Convert(RawDataCache* input, PointCloudCache* output) {
  AINFO << "start convert thread.";
  Converter converter;
  if (!converter.init(conf_)) {
    AERROR << "init convert fail.";
    return;
  }

  while (running_) {
    int64_t t0 = GetTime();
    ADEBUG << "input cache size: " << input->size();
    velodyne_msgs::VelodyneScanUnifiedPtr raw = input->take();

    sensor_msgs::PointCloud2Ptr pointcloud(new sensor_msgs::PointCloud2());
    if (!pointcloud) {
      AERROR << "new PointCloud2 fail.";
      continue;
    }
    int64_t t1 = GetTime();

    if (FLAGS_pipeline_mode) {
      if (!converter.append(raw)) {
        AERROR << "converter append fail.";
        continue;
      }
      if (!converter.ready()) {
        continue;
      }
      if (!converter.pack(pointcloud)) {
        AERROR << "converter pack fail.";
        continue;
      }
    } else if (!converter.convert_packets_to_pointcloud(raw, pointcloud)) {
      AERROR << "convert packet to pointcloud fail.";
      continue;
    }
    int64_t t2 = GetTime();

    output->put(pointcloud);
    if (FLAGS_publish_raw_pointcloud) {
      VelodyneAdapter::PublishPointCloudRawByIndex(conf_.index(), pointcloud);
    }
    AINFO << "CALC convert done.";
    int64_t t3 = GetTime();

    ADEBUG << "convert total time: " << t3 - t0;
    ADEBUG << "convert take time: " << t1 - t0;
    ADEBUG << "convert point cloud time: " << t2 - t1;
  }

  AINFO << "end of convert thread func.";
  return;
}

void Velodyne::Compensate(PointCloudCache* input) {
  AINFO << "start compensate thread.";
  Compensator compensator(conf_);
  while (running_) {
    int64_t t0 = GetTime();
    sensor_msgs::PointCloud2Ptr pointcloud = input->take();

    int64_t t1 = GetTime();
    sensor_msgs::PointCloud2Ptr com_pointcloud(new sensor_msgs::PointCloud2());
    if (!compensator.pointcloud_compensate(pointcloud, com_pointcloud)) {
      AWARN << "pointcloud compensate fail.";
      continue;
    }
    uint32_t index = conf_.index();
    if (FLAGS_publish_compensator_pointcloud) {
      VelodyneAdapter::PublishPointCloudByIndex(index, com_pointcloud);
    }
    AINFO << "CALC compensate done.";
    int64_t t2 = GetTime();
    ADEBUG << "compensate total time: " << t2 - t0;
    ADEBUG << "compensate take time: " << t1 - t0;
    ADEBUG << "compensate real time: " << t2 - t1;
  }

  AINFO << "end of compensator thread func.";
  return;
}

Status Velodyne::Start() {
  std::shared_ptr<std::thread> packet_thread(
      new std::thread(std::bind(&Velodyne::Packet, this, packet_cache_.get())));
  std::shared_ptr<std::thread> convert_therad(new std::thread(std::bind(
      &Velodyne::Convert, this, packet_cache_.get(), pointcloud_cache_.get())));
  std::shared_ptr<std::thread> compensate_thread(new std::thread(
      std::bind(&Velodyne::Compensate, this, pointcloud_cache_.get())));

  threads_.push_back(packet_thread);
  threads_.push_back(convert_therad);
  threads_.push_back(compensate_thread);

  ADEBUG << "Velodyne start done!";
  common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  buffer.INFO("velodyne started");

  return Status::OK();
}

void Velodyne::Stop() {
  AINFO << "Velodyne Stopping ...";
  running_ = false;

  for (size_t i = 0; i < threads_.size(); ++i) {
    if (threads_[i]->joinable()) {
      threads_[i]->join();
    }
  }
  threads_.clear();
  AINFO << "Velodyne Stopped.";
}

void Velodyne::Notice() {
  AINFO_IF(!FLAGS_publish_raw_data) << "publish_raw_data flag closed,"
                                    << "will not publish raw data";
  AINFO_IF(!FLAGS_publish_raw_pointcloud)
      << "publish_raw_pointcloud flag closed,"
      << "will not publish pointcloud2 raw data";
  AINFO_IF(!FLAGS_publish_compensator_pointcloud)
      << "publish_compensator_pointcloud flag closed,"
      << "will not publish compensator pointcloud2";
}

bool Velodyne::SetNpackets(VelodyneConf* conf) {
  const double frequency = (conf->rpm() / 60.0);
  conf->set_npackets(static_cast<int>(ceil(conf->packet_rate() / frequency)));
  AINFO << "lidar " << conf->index() << " npackets is " << conf->npackets();
  return true;
}

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo
