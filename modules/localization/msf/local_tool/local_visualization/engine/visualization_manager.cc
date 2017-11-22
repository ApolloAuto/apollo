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

#include "visualization_manager.h"
#include <boost/filesystem.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "modules/localization/msf/common/io/velodyne_utility.h"

namespace apollo {
namespace localization {
namespace msf {

//===================MessageBuffer=======================
template <class MessageType>
MessageBuffer<MessageType>::MessageBuffer(int capacity) : capacity_(capacity) {
  pthread_mutex_init(&buffer_mutex_, NULL);
}

template <class MessageType>
MessageBuffer<MessageType>::~MessageBuffer() {
  pthread_mutex_destroy(&buffer_mutex_);
}

template <class MessageType>
bool MessageBuffer<MessageType>::PushNewMessage(const double timestamp,
                                                const MessageType &msg) {
  if (capacity_ == 0) {
    std::cerr << "The buffer capacity is 0." << std::endl;
    return false;
  }

  auto found_iter = msg_map_.find(timestamp);
  if (found_iter != msg_map_.end()) {
    std::cerr << "The msg has existed in buffer." << std::endl;
    return false;
  }

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
bool MessageBuffer<MessageType>::PopOldestMessage(MessageType &msg) {
  if (IsEmpty()) {
    std::cerr << "The buffer is empty." << std::endl;
    return false;
  }

  pthread_mutex_lock(&buffer_mutex_);
  msg = msg_list_.begin()->second;
  msg_map_.erase(msg_list_.begin()->first);
  msg_list_.pop_front();
  pthread_mutex_unlock(&buffer_mutex_);

  return true;
}

template <class MessageType>
bool MessageBuffer<MessageType>::GetMessage(const double timestamp,
                                            MessageType &msg) {
  pthread_mutex_lock(&buffer_mutex_);
  auto found_iter = msg_map_.find(timestamp);
  if (found_iter != msg_map_.end()) {
    msg = found_iter->second;
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
    std::list<std::pair<double, MessageType>> &msg_list) {
  pthread_mutex_lock(&buffer_mutex_);
  msg_list.clear();
  std::copy(msg_list_.begin(), msg_list_.end(), std::back_inserter(msg_list));
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

//==============IntepolationMessageBuffer==================
template <class MessageType>
IntepolationMessageBuffer<MessageType>::IntepolationMessageBuffer(int capacity)
    : MessageBuffer<MessageType>(capacity) {}

template <class MessageType>
IntepolationMessageBuffer<MessageType>::~IntepolationMessageBuffer() {}

template <class MessageType>
bool IntepolationMessageBuffer<MessageType>::QueryMessage(
    const double timestamp, MessageType &msg, double timeout_s) {
  std::map<double, ListIterator> msg_map_tem;
  std::list<std::pair<double, MessageType>> msg_list_tem;

  if (!WaitMessageBufferOk(timestamp, msg_map_tem, msg_list_tem,
                           timeout_s * 100)) {
    return false;
  }

  auto found_iter = msg_map_tem.find(timestamp);
  if (found_iter != msg_map_tem.end()) {
    msg = found_iter->second->second;
    return true;
  }

  ListIterator begin_iter = msg_list_tem.begin();
  if (begin_iter->first > timestamp) {
    return false;
  }

  ListIterator after_iter;
  ListIterator before_iter;
  for (ListIterator iter = msg_list_tem.end(); iter != msg_list_tem.begin();
       --iter) {
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
        std::cerr << "Delta_time is too small." << std::endl;
        return false;
      }
      double scale = (timestamp - before_iter->first) / delta_time;
      msg = before_iter->second.interpolate(scale, after_iter->second);
      msg.timestamp = timestamp;
      break;
    }
  }

  return true;
}

template <class MessageType>
bool IntepolationMessageBuffer<MessageType>::WaitMessageBufferOk(
    const double timestamp, std::map<double, ListIterator> &msg_map,
    std::list<std::pair<double, MessageType>> &msg_list, double timeout_ms) {
  boost::posix_time::ptime start_time =
      boost::posix_time::microsec_clock::local_time();
  pthread_mutex_lock(&(this->buffer_mutex_));
  msg_list.clear();
  msg_map.clear();
  std::copy(this->msg_list_.begin(), this->msg_list_.end(),
            std::back_inserter(msg_list));
  pthread_mutex_unlock(&(this->buffer_mutex_));

  if (msg_list.empty()) {
    std::cerr << "The queried buffer is empty." << std::endl;
    return false;
  }

  ListIterator last_iter = msg_list.end();
  --last_iter;
  while (last_iter->first < timestamp) {
    std::cout << "Waiting new message!" << std::endl;
    usleep(5000);
    pthread_mutex_lock(&(this->buffer_mutex_));
    msg_list.clear();
    msg_map.clear();
    std::copy(this->msg_list_.begin(), this->msg_list_.end(),
              std::back_inserter(msg_list));
    pthread_mutex_unlock(&(this->buffer_mutex_));
    last_iter = msg_list.end();
    --last_iter;

    boost::posix_time::ptime end_time =
        boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration during_time = end_time - start_time;

    if (during_time.total_milliseconds() >= timeout_ms) {
      std::cout << "Waiting time is out" << std::endl;
      return false;
    }
  }

  return true;
}

// ==================VisualizationManager=================
VisualizationManager::VisualizationManager()
    : visual_engine_(),
      stop_visualization_(false),
      lidar_frame_buffer_(2),
      gnss_loc_info_buffer_(2),
      lidar_loc_info_buffer_(20),
      fusion_loc_info_buffer_(200) {}

VisualizationManager::~VisualizationManager() {
  stop_visualization_ = true;
  visual_thread_.join();
}

bool VisualizationManager::Init(const std::string &map_folder,
                                const Eigen::Affine3d &velodyne_extrinsic) {
  BaseMapConfig map_config;
  unsigned int resolution_id = 0;
  int zone_id = 0;

  std::string config_file = map_folder + "/config.xml";
  map_config.map_version_ = "lossy_map";
  bool success = map_config.Load(config_file);
  if (!success) {
    std::cerr << "Load map config failed." << std::endl;
    return false;
  }
  std::cout << "Load map config succeed." << std::endl;

  resolution_id = 0;
  success = GetZoneIdFromMapFolder(map_folder, resolution_id, zone_id);
  if (!success) {
    std::cerr << "Get zone id failed." << std::endl;
    return false;
  }
  std::cout << "Get zone id succeed." << std::endl;

  success = visual_engine_.Init(map_folder, map_config, resolution_id, zone_id,
                                velodyne_extrinsic, LOC_INFO_NUM);
  if (!success) {
    std::cerr << "Visualization engine init failed." << std::endl;
    return false;
  }
  std::cout << "Visualization engine init succeed." << std::endl;

  return true;
}

void VisualizationManager::AddLidarFrame(const LidarFrame &lidar_frame) {
  lidar_frame_buffer_.PushNewMessage(lidar_frame.timestamp, lidar_frame);
}

void VisualizationManager::AddGNSSLocMessage(
    const LocalizationMsg &gnss_loc_msg) {
  gnss_loc_info_buffer_.PushNewMessage(gnss_loc_msg.timestamp, gnss_loc_msg);
}

void VisualizationManager::AddLidarLocMessage(
    const LocalizationMsg &lidar_loc_msg) {
  lidar_loc_info_buffer_.PushNewMessage(lidar_loc_msg.timestamp, lidar_loc_msg);
}

void VisualizationManager::AddFusionLocMessage(
    const LocalizationMsg &fusion_loc_msg) {
  fusion_loc_info_buffer_.PushNewMessage(fusion_loc_msg.timestamp,
                                         fusion_loc_msg);
}

void VisualizationManager::StartVisualization() {
  visual_thread_ = std::thread(&VisualizationManager::DoVisualize, this);
}

void VisualizationManager::DoVisualize() {
  while (!stop_visualization_) {
    usleep(50000);
    if (!lidar_frame_buffer_.IsEmpty()) {
      LidarFrame lidar_frame;
      bool pop_success = lidar_frame_buffer_.PopOldestMessage(lidar_frame);
      if (!pop_success) {
        continue;
      }

      LocalizationMsg lidar_loc;
      LocalizationMsg fusion_loc;
      bool lidar_query_success = fusion_loc_info_buffer_.QueryMessage(
          lidar_frame.timestamp, lidar_loc, 0.1);
      bool fusion_query_success = fusion_loc_info_buffer_.QueryMessage(
          lidar_frame.timestamp, fusion_loc, 0.1);

      if (!lidar_query_success && !fusion_query_success) {
        continue;
      }

      LocalizatonInfo lidar_loc_info;
      LocalizatonInfo fusion_loc_info;

      if (lidar_query_success) {
        Eigen::Translation3d trans(
            Eigen::Vector3d(lidar_loc.x, lidar_loc.y, lidar_loc.z));
        Eigen::Quaterniond quat(lidar_loc.qw, lidar_loc.qx, lidar_loc.qy,
                                lidar_loc.qz);

        const Eigen::Affine3d lidar_pose = trans * quat;
        const Eigen::Vector3d lidar_std(lidar_loc.std_x, lidar_loc.std_y,
                                        lidar_loc.std_z);

        lidar_loc_info.set(lidar_pose, lidar_std, "Lidar.",
                           lidar_frame.timestamp, lidar_frame.frame_id);
      }

      if (fusion_query_success) {
        Eigen::Translation3d trans(
            Eigen::Vector3d(fusion_loc.x, fusion_loc.y, fusion_loc.z));
        Eigen::Quaterniond quat(fusion_loc.qw, fusion_loc.qx, fusion_loc.qy,
                                fusion_loc.qz);

        const Eigen::Affine3d fusion_pose = trans * quat;
        const Eigen::Vector3d fusion_std(fusion_loc.std_x, fusion_loc.std_y,
                                         fusion_loc.std_z);

        fusion_loc_info.set(fusion_pose, fusion_std, "Fusion.",
                            lidar_frame.timestamp, lidar_frame.frame_id);
      }

      std::vector<LocalizatonInfo> loc_infos;
      loc_infos.push_back(lidar_loc_info);
      loc_infos.push_back(fusion_loc_info);
      visual_engine_.Visualize(loc_infos, lidar_frame.pt3ds);
    }
  }
}

bool VisualizationManager::GetZoneIdFromMapFolder(
    const std::string &map_folder, const unsigned int &resolution_id,
    int &zone_id) {
  char buf[256];
  snprintf(buf, 256, "/%03u", resolution_id);
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

      zone_id = -(std::stoi(zone_id_str));
      std::cout << "Find zone id: " << zone_id << std::endl;
      return true;
    }
  }
  std::string zone_id_full_path = (*iter_north).path().string();
  std::size_t pos = zone_id_full_path.find_last_of("/");
  std::string zone_id_str =
      zone_id_full_path.substr(pos + 1, zone_id_full_path.length());

  zone_id = (std::stoi(zone_id_str));
  std::cout << "Find zone id: " << zone_id << std::endl;
  return true;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
