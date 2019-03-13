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

/**
 * @file visualization_manager.h
 * @brief
 */
#pragma once

#include <atomic>
#include <list>
#include <map>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "modules/localization/msf/local_tool/local_visualization/engine/visualization_engine.h"

namespace apollo {
namespace localization {
namespace msf {

struct LidarVisFrame {
  /**@brief The frame index. */
  unsigned int frame_id;
  /**@brief The time stamp. */
  double timestamp;
  /**@brief The 3D point cloud in this frame. */
  std::vector<Eigen::Vector3d> pt3ds;
};

struct LocalizationMsg {
  double timestamp;
  double x;
  double y;
  double z;

  double qx;
  double qy;
  double qz;
  double qw;

  double std_x = 0;
  double std_y = 0;
  double std_z = 0;

  LocalizationMsg interpolate(const double scale,
                              const LocalizationMsg &loc_msg) {
    LocalizationMsg res;
    res.x = this->x * (1 - scale) + loc_msg.x * scale;
    res.y = this->y * (1 - scale) + loc_msg.y * scale;
    res.z = this->z * (1 - scale) + loc_msg.z * scale;

    Eigen::Quaterniond quatd1(this->qw, this->qx, this->qy, this->qz);
    Eigen::Quaterniond quatd2(loc_msg.qw, loc_msg.qx, loc_msg.qy, loc_msg.qz);
    Eigen::Quaterniond res_quatd = quatd1.slerp(scale, quatd2);
    res.qx = res_quatd.x();
    res.qy = res_quatd.y();
    res.qz = res_quatd.z();
    res.qw = res_quatd.w();

    res.std_x = this->std_x * (1 - scale) + loc_msg.std_x * scale;
    res.std_y = this->std_y * (1 - scale) + loc_msg.std_y * scale;
    res.std_z = this->std_z * (1 - scale) + loc_msg.std_z * scale;

    return res;
  }
};

template <class MessageType>
class MessageBuffer {
 public:
  typedef
      typename std::list<std::pair<double, MessageType>>::iterator ListIterator;

 public:
  explicit MessageBuffer(int capacity);
  ~MessageBuffer();

  bool PushNewMessage(const double timestamp, const MessageType &msg);
  bool PopOldestMessage(MessageType *msg);
  bool GetMessageBefore(const double timestamp, MessageType *msg);
  bool GetMessage(const double timestamp, MessageType *msg);

  void Clear();

  void SetCapacity(const unsigned int capacity);
  void GetAllMessages(std::list<std::pair<double, MessageType>> *msg_list);

  bool IsEmpty();
  unsigned int BufferSize();

 protected:
  std::map<double, ListIterator> msg_map_;
  std::list<std::pair<double, MessageType>> msg_list_;

 protected:
  pthread_mutex_t buffer_mutex_;
  unsigned int capacity_;
};

template <class MessageType>
class IntepolationMessageBuffer : public MessageBuffer<MessageType> {
 public:
  typedef
      typename std::list<std::pair<double, MessageType>>::iterator ListIterator;

 public:
  explicit IntepolationMessageBuffer(int capacity);
  ~IntepolationMessageBuffer();

  bool QueryMessage(const double timestamp, MessageType *msg,
                    double timeout_s = 0.01);

 private:
  bool WaitMessageBufferOk(const double timestamp,
                           std::map<double, ListIterator> *msg_map,
                           std::list<std::pair<double, MessageType>> *msg_list,
                           double timeout_ms);
};

struct VisualizationManagerParams {
  std::string map_folder;
  std::string map_visual_folder;
  Eigen::Affine3d velodyne_extrinsic;
  VisualMapParam map_param;
  unsigned int lidar_frame_buffer_capacity;
  unsigned int gnss_loc_info_buffer_capacity;
  unsigned int lidar_loc_info_buffer_capacity;
  unsigned int fusion_loc_info_buffer_capacity;
};

class VisualizationManager {
#define LOC_INFO_NUM 3

 public:
  VisualizationManager();
  ~VisualizationManager();

  static VisualizationManager &GetInstance() {
    static VisualizationManager visual_manage;
    return visual_manage;
  }

  bool Init(const std::string &map_folder, const std::string &map_visual_folder,
            const Eigen::Affine3d &velodyne_extrinsic,
            const VisualMapParam &map_param);
  bool Init(const VisualizationManagerParams &params);

  void AddLidarFrame(const LidarVisFrame &lidar_frame);
  void AddGNSSLocMessage(const LocalizationMsg &gnss_loc_msg);
  void AddLidarLocMessage(const LocalizationMsg &lidar_loc_msg);
  void AddFusionLocMessage(const LocalizationMsg &fusion_loc_msg);
  void StartVisualization();
  void StopVisualization();

 private:
  void DoVisualize();
  bool GetZoneIdFromMapFolder(const std::string &map_folder,
                              const unsigned int resolution_id, int *zone_id);

 private:
  VisualizationEngine visual_engine_;
  // Visualization Thread
  std::thread visual_thread_;
  std::atomic<bool> stop_visualization_;

  MessageBuffer<LidarVisFrame> lidar_frame_buffer_;
  IntepolationMessageBuffer<LocalizationMsg> gnss_loc_info_buffer_;
  IntepolationMessageBuffer<LocalizationMsg> lidar_loc_info_buffer_;
  IntepolationMessageBuffer<LocalizationMsg> fusion_loc_info_buffer_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
