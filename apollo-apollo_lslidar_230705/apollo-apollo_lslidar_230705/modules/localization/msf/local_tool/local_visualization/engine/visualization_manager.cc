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

#include "modules/localization/msf/local_tool/local_visualization/engine/visualization_manager.h"

#include <algorithm>
#include <thread>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>

#include "cyber/common/log.h"

namespace apollo {
namespace localization {
namespace msf {

// ===================MessageBuffer=======================
template <class MessageType>
MessageBuffer<MessageType>::MessageBuffer(int capacity) : capacity_(capacity) {
  pthread_mutex_init(&buffer_mutex_, nullptr);
}

template <class MessageType>
MessageBuffer<MessageType>::~MessageBuffer() {
  pthread_mutex_destroy(&buffer_mutex_);
}

template <class MessageType>
bool MessageBuffer<MessageType>::PushNewMessage(const double timestamp,
                                                const MessageType &msg) {
  if (capacity_ == 0) {
    AERROR << "The buffer capacity is 0.";
    return false;
  }

  pthread_mutex_lock(&buffer_mutex_);
  auto found_iter = msg_map_.find(timestamp);
  if (found_iter != msg_map_.end()) {
    AERROR << "The msg has existed in buffer.";
    pthread_mutex_unlock(&buffer_mutex_);
    return false;
  }
  pthread_mutex_unlock(&buffer_mutex_);

  std::pair<double, MessageType> msg_pair = std::make_pair(timestamp, msg);
  pthread_mutex_lock(&buffer_mutex_);
  if (msg_list_.size() < capacity_) {
    msg_list_.push_back(msg_pair);
  } else {
    msg_map_.erase(msg_list_.begin()->first);
    msg_list_.pop_front();
    msg_list_.push_back(msg_pair);
  }
  ListIterator iter = msg_list_.end();
  msg_map_[timestamp] = (--iter);
  pthread_mutex_unlock(&buffer_mutex_);

  return true;
}

template <class MessageType>
bool MessageBuffer<MessageType>::PopOldestMessage(MessageType *msg) {
  if (IsEmpty()) {
    AERROR << "The buffer is empty.";
    return false;
  }

  pthread_mutex_lock(&buffer_mutex_);
  *msg = msg_list_.begin()->second;
  msg_map_.erase(msg_list_.begin()->first);
  msg_list_.pop_front();
  pthread_mutex_unlock(&buffer_mutex_);

  return true;
}

template <class MessageType>
bool MessageBuffer<MessageType>::GetMessageBefore(const double timestamp,
                                                  MessageType *msg) {
  if (IsEmpty()) {
    AERROR << "The buffer is empty.";
    return false;
  }

  std::list<std::pair<double, MessageType>> msg_list;
  pthread_mutex_lock(&buffer_mutex_);
  std::copy(this->msg_list_.begin(), this->msg_list_.end(),
            std::back_inserter(msg_list));
  pthread_mutex_unlock(&buffer_mutex_);

  for (ListIterator iter = msg_list.end(); iter != msg_list.begin();) {
    --iter;
    if (iter->first <= timestamp) {
      *msg = iter->second;
      return true;
    }
  }

  return false;
}

template <class MessageType>
bool MessageBuffer<MessageType>::GetMessage(const double timestamp,
                                            MessageType *msg) {
  pthread_mutex_lock(&buffer_mutex_);
  auto found_iter = msg_map_.find(timestamp);
  if (found_iter != msg_map_.end()) {
    *msg = found_iter->second->second;
    pthread_mutex_unlock(&buffer_mutex_);
    return true;
  }
  pthread_mutex_unlock(&buffer_mutex_);
  return false;
}

template <class MessageType>
void MessageBuffer<MessageType>::Clear() {
  pthread_mutex_lock(&buffer_mutex_);
  msg_list_.clear();
  msg_map_.clear();
  pthread_mutex_unlock(&buffer_mutex_);
}

template <class MessageType>
void MessageBuffer<MessageType>::SetCapacity(const unsigned int capacity) {
  capacity_ = capacity;
}

template <class MessageType>
void MessageBuffer<MessageType>::GetAllMessages(
    std::list<std::pair<double, MessageType>> *msg_list) {
  pthread_mutex_lock(&buffer_mutex_);
  msg_list->clear();
  std::copy(msg_list_.begin(), msg_list_.end(), std::back_inserter(*msg_list));
  pthread_mutex_unlock(&buffer_mutex_);
}

template <class MessageType>
bool MessageBuffer<MessageType>::IsEmpty() {
  bool flag = true;
  pthread_mutex_lock(&buffer_mutex_);
  flag = msg_list_.empty();
  pthread_mutex_unlock(&buffer_mutex_);
  return flag;
}

template <class MessageType>
unsigned int MessageBuffer<MessageType>::BufferSize() {
  unsigned int size = 0;
  pthread_mutex_lock(&buffer_mutex_);
  size = static_cast<unsigned int>(msg_list_.size());
  pthread_mutex_unlock(&buffer_mutex_);

  return size;
}

// ==============IntepolationMessageBuffer==================
template <class MessageType>
IntepolationMessageBuffer<MessageType>::IntepolationMessageBuffer(int capacity)
    : MessageBuffer<MessageType>(capacity) {}

template <class MessageType>
IntepolationMessageBuffer<MessageType>::~IntepolationMessageBuffer() {}

template <class MessageType>
bool IntepolationMessageBuffer<MessageType>::QueryMessage(
    const double timestamp, MessageType *msg, double timeout_s) {
  std::map<double, ListIterator> msg_map_tem;
  std::list<std::pair<double, MessageType>> msg_list_tem;

  if (!WaitMessageBufferOk(timestamp, &msg_map_tem, &msg_list_tem,
                           timeout_s * 100)) {
    return false;
  }

  auto found_iter = msg_map_tem.find(timestamp);
  if (found_iter != msg_map_tem.end()) {
    *msg = found_iter->second->second;
    return true;
  }

  ListIterator begin_iter = msg_list_tem.begin();
  if (begin_iter->first > timestamp) {
    return false;
  }

  ListIterator after_iter;
  ListIterator before_iter;
  for (ListIterator iter = msg_list_tem.end(); iter != msg_list_tem.begin();) {
    --iter;
    if (iter->first > timestamp) {
      after_iter = iter;
    } else {
      before_iter = iter;

      double delta_time = after_iter->first - before_iter->first;
      // if (delete_time > 0.1) {
      //     return false;
      // }
      if (std::abs(delta_time) < 1e-9) {
        AERROR << "Delta_time is too small.";
        return false;
      }
      double scale = (timestamp - before_iter->first) / delta_time;
      *msg = before_iter->second.interpolate(scale, after_iter->second);
      msg->timestamp = timestamp;
      break;
    }
  }
  return true;
}

template <class MessageType>
bool IntepolationMessageBuffer<MessageType>::WaitMessageBufferOk(
    const double timestamp, std::map<double, ListIterator> *msg_map,
    std::list<std::pair<double, MessageType>> *msg_list, double timeout_ms) {
  boost::posix_time::ptime start_time =
      boost::posix_time::microsec_clock::local_time();
  pthread_mutex_lock(&(this->buffer_mutex_));
  msg_list->clear();
  std::copy(this->msg_list_.begin(), this->msg_list_.end(),
            std::back_inserter(*msg_list));
  *msg_map = this->msg_map_;
  pthread_mutex_unlock(&(this->buffer_mutex_));

  if (msg_list->empty()) {
    AERROR << "The queried buffer is empty.";
    return false;
  }

  ListIterator last_iter = msg_list->end();
  --last_iter;

  if (last_iter->first < timestamp && timeout_ms < 5000) {
    return false;
  }

  while (last_iter->first < timestamp) {
    AINFO << "Waiting new message!";
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    pthread_mutex_lock(&(this->buffer_mutex_));
    msg_list->clear();
    std::copy(this->msg_list_.begin(), this->msg_list_.end(),
              std::back_inserter(*msg_list));
    *msg_map = this->msg_map_;
    pthread_mutex_unlock(&(this->buffer_mutex_));
    last_iter = msg_list->end();
    --last_iter;

    boost::posix_time::ptime end_time =
        boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration during_time = end_time - start_time;

    if (during_time.total_milliseconds() >= timeout_ms) {
      AERROR << "Waiting time is out";
      return false;
    }
  }

  return true;
}

// ==================VisualizationManager=================
VisualizationManager::VisualizationManager()
    : visual_engine_(),
      stop_visualization_(false),
      lidar_frame_buffer_(10),
      gnss_loc_info_buffer_(20),
      lidar_loc_info_buffer_(40),
      fusion_loc_info_buffer_(400) {}

VisualizationManager::~VisualizationManager() {
  if (!(stop_visualization_.load())) {
    stop_visualization_ = true;
    visual_thread_.join();
  }
}

bool VisualizationManager::Init(const std::string &map_folder,
                                const std::string &map_visual_folder,
                                const Eigen::Affine3d &velodyne_extrinsic,
                                const VisualMapParam &map_param) {
  AINFO << "Get zone id.";
  unsigned int resolution_id = 0;
  int zone_id = 0;

  bool success = GetZoneIdFromMapFolder(map_folder, resolution_id, &zone_id);
  if (!success) {
    AERROR << "Get zone id failed.";
    return false;
  }
  AINFO << "Get zone id succeed.";

  AINFO << "Init visualization engine.";
  success = visual_engine_.Init(map_folder, map_visual_folder, map_param,
                                resolution_id, zone_id, velodyne_extrinsic,
                                LOC_INFO_NUM);
  if (!success) {
    AERROR << "Visualization engine init failed.";
    return false;
  }
  AINFO << "Visualization engine init succeed.";

  visual_engine_.SetAutoPlay(true);

  return true;
}

bool VisualizationManager::Init(const VisualizationManagerParams &params) {
  lidar_frame_buffer_.SetCapacity(params.lidar_frame_buffer_capacity);
  gnss_loc_info_buffer_.SetCapacity(params.gnss_loc_info_buffer_capacity);
  lidar_loc_info_buffer_.SetCapacity(params.lidar_loc_info_buffer_capacity);
  fusion_loc_info_buffer_.SetCapacity(params.fusion_loc_info_buffer_capacity);

  return Init(params.map_folder, params.map_visual_folder,
              params.velodyne_extrinsic, params.map_param);
}

void VisualizationManager::AddLidarFrame(const LidarVisFrame &lidar_frame) {
  AINFO << "AddLidarFrame.";
  static int id = 0;
  AINFO << "id." << id;
  lidar_frame_buffer_.PushNewMessage(lidar_frame.timestamp, lidar_frame);
  id++;
}

void VisualizationManager::AddGNSSLocMessage(
    const LocalizationMsg &gnss_loc_msg) {
  AINFO << "AddGNSSLocMessage.";
  gnss_loc_info_buffer_.PushNewMessage(gnss_loc_msg.timestamp, gnss_loc_msg);
}

void VisualizationManager::AddLidarLocMessage(
    const LocalizationMsg &lidar_loc_msg) {
  AINFO << "AddLidarLocMessage.";
  lidar_loc_info_buffer_.PushNewMessage(lidar_loc_msg.timestamp, lidar_loc_msg);
}

void VisualizationManager::AddFusionLocMessage(
    const LocalizationMsg &fusion_loc_msg) {
  AINFO << "AddFusionLocMessage.";
  fusion_loc_info_buffer_.PushNewMessage(fusion_loc_msg.timestamp,
                                         fusion_loc_msg);
}

void VisualizationManager::StartVisualization() {
  visual_thread_ = std::thread(&VisualizationManager::DoVisualize, this);
}

void VisualizationManager::StopVisualization() {
  stop_visualization_ = true;
  visual_thread_.join();
}

void VisualizationManager::DoVisualize() {
  while (!(stop_visualization_.load())) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // if (!lidar_frame_buffer_.IsEmpty()) {
    if (lidar_frame_buffer_.BufferSize() > 5) {
      LidarVisFrame lidar_frame;
      bool pop_success = lidar_frame_buffer_.PopOldestMessage(&lidar_frame);
      if (!pop_success) {
        continue;
      }

      LocalizationMsg lidar_loc;
      LocalizationMsg fusion_loc;
      bool lidar_query_success = lidar_loc_info_buffer_.QueryMessage(
          lidar_frame.timestamp, &lidar_loc, 0);
      // bool lidar_query_success = lidar_loc_info_buffer_.GetMessage(
      //     lidar_frame.timestamp, lidar_loc);

      bool fusion_query_success = fusion_loc_info_buffer_.QueryMessage(
          lidar_frame.timestamp, &fusion_loc, 0);

      if (!lidar_query_success && !fusion_query_success) {
        continue;
      }

      LocalizatonInfo lidar_loc_info;
      LocalizatonInfo fusion_loc_info;
      LocalizatonInfo gnss_loc_info;

      if (lidar_query_success) {
        Eigen::Translation3d trans(
            Eigen::Vector3d(lidar_loc.x, lidar_loc.y, lidar_loc.z));
        Eigen::Quaterniond quat(lidar_loc.qw, lidar_loc.qx, lidar_loc.qy,
                                lidar_loc.qz);

        const Eigen::Vector3d lidar_std(lidar_loc.std_x, lidar_loc.std_y,
                                        lidar_loc.std_z);

        lidar_loc_info.set(trans, quat, lidar_std, "Lidar.",
                           lidar_frame.timestamp, lidar_frame.frame_id);
      }

      if (fusion_query_success) {
        Eigen::Translation3d trans(
            Eigen::Vector3d(fusion_loc.x, fusion_loc.y, fusion_loc.z));
        Eigen::Quaterniond quat(fusion_loc.qw, fusion_loc.qx, fusion_loc.qy,
                                fusion_loc.qz);

        const Eigen::Vector3d fusion_std(fusion_loc.std_x, fusion_loc.std_y,
                                         fusion_loc.std_z);

        fusion_loc_info.set(trans, quat, fusion_std, "Fusion.",
                            lidar_frame.timestamp, lidar_frame.frame_id);
      }

      LocalizationMsg gnss_loc;
      bool success = gnss_loc_info_buffer_.GetMessageBefore(
          lidar_frame.timestamp, &gnss_loc);
      if (success) {
        Eigen::Translation3d trans(
            Eigen::Vector3d(gnss_loc.x, gnss_loc.y, gnss_loc.z));

        const Eigen::Vector3d gnss_std(gnss_loc.std_x, gnss_loc.std_y,
                                       gnss_loc.std_z);

        gnss_loc_info.set(trans, gnss_std, "GNSS.", lidar_frame.timestamp,
                          lidar_frame.frame_id);
      }

      ::apollo::common::EigenVector<LocalizatonInfo> loc_infos{
          lidar_loc_info, fusion_loc_info, gnss_loc_info};
      visual_engine_.Visualize(std::move(loc_infos), lidar_frame.pt3ds);
    }
  }
}

bool VisualizationManager::GetZoneIdFromMapFolder(
    const std::string &map_folder, const unsigned int resolution_id,
    int *zone_id) {
  char buf[256];
  snprintf(buf, sizeof(buf), "/%03u", resolution_id);
  std::string folder_north = map_folder + "/map" + buf + "/north";
  std::string folder_south = map_folder + "/map" + buf + "/south";
  boost::filesystem::directory_iterator directory_end;
  boost::filesystem::directory_iterator iter_north(folder_north);
  if (iter_north == directory_end) {
    boost::filesystem::directory_iterator iter_south(folder_south);
    if (iter_south == directory_end) {
      return false;
    } else {
      std::string zone_id_full_path = (*iter_south).path().string();
      std::size_t pos = zone_id_full_path.find_last_of("/");
      std::string zone_id_str =
          zone_id_full_path.substr(pos + 1, zone_id_full_path.length());

      *zone_id = -(std::stoi(zone_id_str));
      AINFO << "Find zone id: " << *zone_id;
      return true;
    }
  }
  std::string zone_id_full_path = (*iter_north).path().string();
  std::size_t pos = zone_id_full_path.find_last_of("/");
  std::string zone_id_str =
      zone_id_full_path.substr(pos + 1, zone_id_full_path.length());

  *zone_id = (std::stoi(zone_id_str));
  AINFO << "Find zone id: " << *zone_id;
  return true;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
