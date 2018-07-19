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
#include "modules/drivers/lidar_velodyne/pointcloud/fusion.h"
#include "modules/drivers/lidar_velodyne/pointcloud/multiple_velodyne_adapter.h"

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

  if (!GetProtoFromFile(FLAGS_velodyne_conf_file, &conf_unit_)) {
    std::string msg = "Load velodyne conf fail:" + FLAGS_velodyne_conf_file;
    AERROR << msg;
    return Status(ErrorCode::DRIVER_ERROR_VELODYNE, msg);
  }
  AINFO << "Conf file: " << FLAGS_velodyne_conf_file << " is loaded.";
  if (!CalcNpackets(&conf_unit_)) {
    AERROR << "calc npackets fail.";
    return Status(ErrorCode::DRIVER_ERROR_VELODYNE, "init npackets fail.");
  }

  AdapterManager::Init(FLAGS_velodyne_adapter_config_filename);
  common::monitor::MonitorLogBuffer buffer(&monitor_logger_);

  if (!MultipleVelodyneAdapter::CheckMultipleVelodyne()) {
    return Status(ErrorCode::DRIVER_ERROR_VELODYNE, "Adapter not initialized");
  }

  Notice();

  is_fusion_ = conf_unit_.is_fusion();

  auto conf_size = conf_unit_.conf_size();
  std::map<uint32_t, uint8_t> velodyne_index_map;

  for (auto i = 0; i < conf_size; i++) {
    int size = conf_unit_.conf(i).cache_size();
    if (FLAGS_pipeline_mode) {
      size += conf_unit_.conf(i).npackets();
    }
    RawDataCache* pack_cache = new RawDataCache;
    pack_cache->init(size, "packet_cache_" + std::to_string(i));
    packet_cache_vec_.push_back(pack_cache);
    PointCloudCache* pointcloud_cache = new PointCloudCache;
    pointcloud_cache->init(conf_unit_.conf(i).cache_size(),
                           "pointcloud_cache_" + std::to_string(i));
    pointcloud_cache_vec_.push_back(pointcloud_cache);
    velodyne_index_map[conf_unit_.conf(i).index()] = 1;
  }

  if (is_fusion_ && !FusionCheckInit(velodyne_index_map)) {
    return Status(ErrorCode::DRIVER_ERROR_VELODYNE, "fusion init error!");
  }

  return Status::OK();
}

void Velodyne::Packet(RawDataCache* output, const VelodyneConf& conf) {
  AINFO << "start packet thread.";
  VelodyneDriver* dvr = VelodyneDriverFactory::create_driver(conf);
  if (nullptr == dvr || !dvr->init()) {
    AERROR << "Create or init driver fail.";
    return;
  }

  velodyne_msgs::VelodyneScanUnifiedPtr full_scan(
      new velodyne_msgs::VelodyneScanUnified());
  full_scan->packets.resize(conf.npackets());
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
      scan_idx++;

      if (scan_idx != conf.npackets()) {
        continue;
      }
      full_scan->header = scan->header;
      full_scan->basetime = scan->basetime;
    }

    if (FLAGS_publish_raw_data) {
      MultipleVelodyneAdapter::PublishVelodyneRawByIndex(conf.index(), pub);
    }

    if (FLAGS_pipeline_mode) {
      scan_idx = 0;
      full_scan.reset(new velodyne_msgs::VelodyneScanUnified());
      full_scan->packets.resize(conf.npackets());
    }
    AINFO << "CALC raw packet done.";

    int64_t t1 = GetTime();
    ADEBUG << "packet total time: " << t1 - t0;
  }

  AINFO << "end of packet thread func.";
  return;
}

void Velodyne::Convert(RawDataCache* input, PointCloudCache* output,
                       const VelodyneConf& conf) {
  AINFO << "start convert thread.";
  Converter converter;
  VelodyneConf conf_var = conf;
  if (!converter.init(&conf_var)) {
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
      MultipleVelodyneAdapter::PublishPointCloudRawByIndex(conf.index(),
                                                           pointcloud);
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

void Velodyne::Compensate(PointCloudCache* input, const VelodyneConf& conf) {
  AINFO << "start compensate thread.";
  Compensator compensator(conf);
  while (running_) {
    int64_t t0 = GetTime();
    sensor_msgs::PointCloud2Ptr pointcloud = input->take();

    int64_t t1 = GetTime();
    sensor_msgs::PointCloud2Ptr com_pointcloud(new sensor_msgs::PointCloud2());
    if (!compensator.pointcloud_compensate(pointcloud, com_pointcloud)) {
      AWARN << "pointcloud compensate fail.";
      continue;
    }
    uint32_t index = conf.index();
    if (is_fusion_ && (fusion_cache_.count(index) > 0)) {
      fusion_cache_[index]->put(com_pointcloud);
    }
    if (FLAGS_publish_compensator_pointcloud) {
      MultipleVelodyneAdapter::PublishPointCloudByIndex(index, com_pointcloud);
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
void Velodyne::PointCloudFusion(
    std::map<uint32_t, PointCloudCache*> fusion_cache) {
  Fusion velodyne_fusion;
  while (running_) {
    int64_t t0 = GetTime();
    int size = fusion_cache.size();
    if (size == 0) {
      AERROR << "no velodyne cache,fusion error!";
      return;
    }
    int64_t t1 = GetTime();
    sensor_msgs::PointCloud2Ptr point_cloud_fusion(
        new sensor_msgs::PointCloud2());
    uint32_t major_index = conf_unit_.fusion_conf().major_index();
    auto it = fusion_cache.find(major_index);
    if (it != fusion_cache.end()) {
      if (size == 1) {
        // only major
        AERROR << "only one major velodyne,fusion error!";
      } else {
        // fusion
        auto major_point_cloud_ptr = fusion_cache[major_index]->take();
        auto it = fusion_cache.begin();  // upper_bound(major_index);
        std::vector<sensor_msgs::PointCloud2Ptr> slave_point_cloud_vec;
        for (; it != fusion_cache.end(); it++) {
          if (it->first == major_index) {
            continue;
          }
          slave_point_cloud_vec.push_back(it->second->take());
        }
        t1 = GetTime();
        bool fusion_ret = velodyne_fusion.fusion(
            major_point_cloud_ptr, slave_point_cloud_vec, point_cloud_fusion);
        if (!fusion_ret) {
          AERROR << "fusion error!";
        }
        // TODO(all): enable fusion
        // AdapterManager::PublishPointCloudFusion(*point_cloud_fusion);
      }
    } else {
      // no major
      AERROR << "no major velodyne,fusion error!";
    }
    AINFO << "CALC fusion done.";
    int64_t t2 = GetTime();
    ADEBUG << "fusion total time: " << t2 - t0;
    ADEBUG << "fusion take time: " << t1 - t0;
    ADEBUG << "fusion real time: " << t2 - t1;
  }
  AINFO << "end of fusion thread func.";
  return;
}

Status Velodyne::Start() {
  auto conf_size = conf_unit_.conf_size();
  for (auto i = 0; i < conf_size; i++) {
    std::shared_ptr<std::thread> packet_thread(new std::thread(std::bind(
        &Velodyne::Packet, this, packet_cache_vec_[i], conf_unit_.conf(i))));
    std::shared_ptr<std::thread> convert_therad(new std::thread(
        std::bind(&Velodyne::Convert, this, packet_cache_vec_[i],
                  pointcloud_cache_vec_[i], conf_unit_.conf(i))));
    std::shared_ptr<std::thread> compensate_thread(new std::thread(
        std::bind(&Velodyne::Compensate, this, pointcloud_cache_vec_[i],
                  conf_unit_.conf(i))));

    threads_.push_back(packet_thread);
    threads_.push_back(convert_therad);
    threads_.push_back(compensate_thread);
  }

  if (is_fusion_) {
    std::shared_ptr<std::thread> fusion_thread(new std::thread(
        std::bind(&Velodyne::PointCloudFusion, this, fusion_cache_)));
    threads_.push_back(fusion_thread);
  }

  AINFO << "Velodyne start done!";
  common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  buffer.INFO("velodyne started");

  return Status::OK();
}

void Velodyne::Stop() {
  AINFO << "Velodyne Stopping ...";
  running_ = false;

  for (uint i = 0; i < threads_.size(); ++i) {
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

bool Velodyne::CalcNpackets(VelodyneConfUnit* unit) {
  for (int i = 0; i < unit->conf_size(); ++i) {
    VelodyneConf* conf = unit->mutable_conf(i);
    double frequency = (conf->rpm() / 60.0);
    conf->set_npackets(static_cast<int>(ceil(conf->packet_rate() / frequency)));
    AINFO << "lidar " << conf->index() << " npackets is " << conf->npackets();
  }
  return true;
}
bool Velodyne::FusionCheckInit(
    const std::map<uint32_t, uint8_t>& velodyne_index_map) {
  if (is_fusion_) {
    uint32_t major_index = conf_unit_.fusion_conf().major_index();
    fusion_index_map_[major_index] = 1;
    auto fusion_size = conf_unit_.fusion_conf().slave_index_size();
    for (int i = 0; i < fusion_size; i++) {
      fusion_index_map_[conf_unit_.fusion_conf().slave_index(i)] = 1;
    }
    if (fusion_index_map_.size() == 1) {
      AERROR << "only one fusion index, not need fusion.";
      return false;
    }
    auto it = fusion_index_map_.begin();
    for (; it != fusion_index_map_.end(); it++) {
      PointCloudCache* fusion_pointcloud_cache = new PointCloudCache;
      uint32_t index = it->first;
      if (velodyne_index_map.count(index) == 0) {
        return false;
      }
      fusion_pointcloud_cache->init(
          conf_unit_.conf(index).cache_size(),
          "fusion_pointcloud_cache_" + std::to_string(index));
      // uint32_t index = conf_unit_.conf(i).index();
      fusion_cache_[index] = fusion_pointcloud_cache;
    }
  }
  return true;
}

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo
