/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
#ifndef APOLLO_LIDAR_COMPONENT_BASE_IMPL_H
#define APOLLO_LIDAR_COMPONENT_BASE_IMPL_H

#include <memory>
#include <string>

#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/drivers/lidar/common/proto/lidar_config_base.pb.h"

#include "cyber/base/arena_queue.h"
#include "cyber/cyber.h"
#include "cyber/transport/shm/protobuf_arena_manager.h"
#include "modules/drivers/lidar/common/sync_buffering.h"

namespace apollo {
namespace drivers {
namespace lidar {

template <typename ScanType, typename ComponentType = apollo::cyber::NullType>
class LidarComponentBaseImpl : public apollo::cyber::Component<ComponentType> {
 public:
  virtual ~LidarComponentBaseImpl() = default;

  bool Init() override = 0;
  virtual bool InitBase(const LidarConfigBase& lidar_config_base) = 0;
  virtual void ReadScanCallback(
      const std::shared_ptr<ScanType>& scan_message) = 0;

  std::shared_ptr<ScanType> AcquireScanMessage();

 protected:
  bool InitConverter(const LidarConfigBase& lidar_config_base);

  bool InitPacket(const LidarConfigBase& lidar_config_base);

  virtual bool WriteScan(const std::shared_ptr<ScanType>& scan_message);

  virtual std::shared_ptr<PointCloud> AllocatePointCloud();

  virtual bool WritePointCloud(const std::shared_ptr<PointCloud>& point_cloud);

  virtual std::shared_ptr<apollo::cyber::base::ArenaQueue<PointXYZIT>>
  GetPointQueue();
  // static std::shared_ptr<PointCloud> PcdDefaultAllocator(
  //         std::shared_ptr<cyber::Writer<PointCloud>>& pcd_writer);

  static void PcdDefaultCleaner(std::shared_ptr<PointCloud>& unused_pcd_frame);

  std::string frame_id_;
  bool write_scan_;
  uint32_t default_reserved_points_num_;

 private:
  std::shared_ptr<cyber::Writer<ScanType>> scan_writer_ = nullptr;
  std::shared_ptr<cyber::Reader<ScanType>> scan_reader_ = nullptr;
  std::shared_ptr<cyber::Writer<PointCloud>> pcd_writer_ = nullptr;
  // std::shared_ptr<SyncBuffering<PointCloud>> pcd_buffer_ = nullptr;

  std::atomic<int> pcd_sequence_num_{0};
};

template <typename ScanType, typename ComponentType>
bool LidarComponentBaseImpl<ScanType, ComponentType>::InitConverter(
    const LidarConfigBase& lidar_config_base) {
  pcd_writer_ = this->node_->template CreateWriter<PointCloud>(
      lidar_config_base.point_cloud_channel());
  RETURN_VAL_IF(pcd_writer_ == nullptr, false);

  if (lidar_config_base.source_type() ==
      LidarConfigBase_SourceType_RAW_PACKET) {
    scan_reader_ = this->node_->template CreateReader<ScanType>(
        lidar_config_base.scan_channel(),
        std::bind(&LidarComponentBaseImpl::ReadScanCallback, this,
                  std::placeholders::_1));
    RETURN_VAL_IF(scan_reader_ == nullptr, false);
  }

  frame_id_ = lidar_config_base.frame_id();
  write_scan_ = lidar_config_base.write_scan();
  default_reserved_points_num_ =
      lidar_config_base.default_reserved_points_num();

  // pcd_buffer_ = std::make_shared<SyncBuffering<PointCloud>>(
  //         [this](){
  //             return LidarComponentBaseImpl::PcdDefaultAllocator(
  //                     this->pcd_writer_);
  //         },
  //         LidarComponentBaseImpl::PcdDefaultCleaner);
  // pcd_buffer_->SetBufferSize(lidar_config_base.buffer_size());
  // pcd_buffer_->Init();
  return true;
}

template <typename ScanType, typename ComponentType>
bool LidarComponentBaseImpl<ScanType, ComponentType>::InitPacket(
    const LidarConfigBase& lidar_config_base) {
  if (lidar_config_base.source_type() ==
      LidarConfigBase_SourceType_ONLINE_LIDAR) {
    scan_writer_ = this->node_->template CreateWriter<ScanType>(
        lidar_config_base.scan_channel());
    RETURN_VAL_IF(scan_writer_ == nullptr, false);
  }
  return true;
}

template <typename ScanType, typename ComponentType>
std::shared_ptr<ScanType>
LidarComponentBaseImpl<ScanType, ComponentType>::AcquireScanMessage() {
  if (scan_writer_ == nullptr) {
    return std::make_shared<ScanType>();
  }
  return scan_writer_->AcquireMessage();
}

template <typename ScanType, typename ComponentType>
bool LidarComponentBaseImpl<ScanType, ComponentType>::WriteScan(
    const std::shared_ptr<ScanType>& scan_message) {
  if (write_scan_) {
    return scan_writer_->Write(scan_message);
  }
  return true;
}

template <typename ScanType, typename ComponentType>
std::shared_ptr<apollo::cyber::base::ArenaQueue<PointXYZIT>>
LidarComponentBaseImpl<ScanType, ComponentType>::GetPointQueue() {
  std::shared_ptr<apollo::cyber::base::ArenaQueue<PointXYZIT>> ret;
  auto channel_id = pcd_writer_->GetChannelId();

  auto arena_manager =
      apollo::cyber::transport::ProtobufArenaManager::Instance();
  if (!arena_manager->Enable()) {
    ADEBUG << "arena manager enable failed.";
  } else {
    apollo::cyber::base::ArenaQueue<PointXYZIT>* p;
    arena_manager->RegisterQueue<apollo::drivers::PointXYZIT>(channel_id,
                                                              170000);
    ret = std::shared_ptr<apollo::cyber::base::ArenaQueue<PointXYZIT>>(
        reinterpret_cast<
            apollo::cyber::base::ArenaQueue<apollo::drivers::PointXYZIT>*>(
            arena_manager->GetAvailableBuffer(channel_id)),
        [](apollo::cyber::base::ArenaQueue<PointXYZIT>* ptr) {});
  }
  return ret;
}

template <typename ScanType, typename ComponentType>
std::shared_ptr<PointCloud>
LidarComponentBaseImpl<ScanType, ComponentType>::AllocatePointCloud() {
  std::shared_ptr<PointCloud> new_pcd_object = pcd_writer_->AcquireMessage();
  new_pcd_object->Clear();
  new_pcd_object->mutable_point()->Reserve(default_reserved_points_num_);
  return new_pcd_object;
}

template <typename ScanType, typename ComponentType>
bool LidarComponentBaseImpl<ScanType, ComponentType>::WritePointCloud(
    const std::shared_ptr<PointCloud>& point_cloud) {
  point_cloud->mutable_header()->set_frame_id(frame_id_);
  point_cloud->mutable_header()->set_sequence_num(
      pcd_sequence_num_.fetch_add(1));
  point_cloud->mutable_header()->set_timestamp_sec(
      cyber::Time().Now().ToSecond());
  RETURN_VAL_IF(!pcd_writer_->Write(point_cloud), false);
  return true;
}

// template <typename ScanType, typename ComponentType>
// std::shared_ptr<PointCloud>
// LidarComponentBaseImpl<ScanType, ComponentType>::PcdDefaultAllocator(
//                     std::shared_ptr<cyber::Writer<PointCloud>>& pcd_writer) {
//     constexpr int default_point_cloud_reserve = 170000;
//     std::shared_ptr<PointCloud> new_pcd_object =
//     pcd_writer->AcquireMessage();
//     new_pcd_object->mutable_point()->Reserve(default_point_cloud_reserve);
//     AINFO << "new pcd frame memory allocated, reserve point size = "
//           << default_point_cloud_reserve;
//     return new_pcd_object;
// }

template <typename ScanType, typename ComponentType>
void LidarComponentBaseImpl<ScanType, ComponentType>::PcdDefaultCleaner(
    std::shared_ptr<PointCloud>& unused_pcd_frame) {
  unused_pcd_frame->clear_point();
}
}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
#endif  // APOLLO_LIDAR_COMPONENT_BASE_IMPL_H
