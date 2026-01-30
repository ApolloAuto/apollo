/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/dreamview_plus/backend/perception_camera_updater/perception_camera_updater.h"

#include <limits>
#include <string>
#include <vector>

#include "opencv2/opencv.hpp"

#include "modules/common_msgs/basic_msgs/geometry.pb.h"
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/dreamview/proto/camera_update.pb.h"

#include "cyber/common/file.h"
#include "modules/common/adapters/adapter_gflags.h"
namespace apollo {
namespace dreamview {

using apollo::common::Quaternion;
using apollo::drivers::CompressedImage;
using apollo::localization::LocalizationEstimate;
using apollo::localization::Pose;
using apollo::transform::TransformStamped;

namespace {
void ConvertMatrixToArray(const Eigen::Matrix4d &matrix,
                          std::vector<double> *array) {
  const double *pointer = matrix.data();
  for (int i = 0; i < matrix.size(); ++i) {
    array->push_back(pointer[i]);
  }
}

template <typename Point>
void ConstructTransformationMatrix(const Quaternion &quaternion,
                                   const Point &translation,
                                   Eigen::Matrix4d *matrix) {
  matrix->setConstant(0);
  Eigen::Quaterniond q;
  q.x() = quaternion.qx();
  q.y() = quaternion.qy();
  q.z() = quaternion.qz();
  q.w() = quaternion.qw();
  matrix->block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  (*matrix)(0, 3) = translation.x();
  (*matrix)(1, 3) = translation.y();
  (*matrix)(2, 3) = translation.z();
  (*matrix)(3, 3) = 1;
}
}  // namespace

PerceptionCameraUpdater::PerceptionCameraUpdater(WebSocketHandler *websocket)
    : websocket_(websocket),
      node_(cyber::CreateNode("perception_camera_updater")) {
  InitReaders();
}

void PerceptionCameraUpdater::StartStream(const double &time_interval_ms,
                                          const std::string &channel_name,
                                          nlohmann::json *subscribe_param) {
  if (channel_name.empty()) {
    AERROR << "Failed to subscribe channel for channel is empty!";
    return;
  }
  if (std::find(channels_.begin(), channels_.end(), channel_name) ==
      channels_.end()) {
    AERROR << "Failed to subscribe channel: " << channel_name
           << " for channels_ not registered!";
    return;
  }
  if (time_interval_ms > 0) {
    CameraChannelUpdater *updater = GetCameraChannelUpdater(channel_name);
    updater->enabled_ = true;
    updater->timer_.reset(new cyber::Timer(
        time_interval_ms,
        [channel_name, this]() { this->OnTimer(channel_name); }, false));
    updater->timer_->Start();
  } else {
    this->OnTimer(channel_name);
  }
}

CameraChannelUpdater *PerceptionCameraUpdater::GetCameraChannelUpdater(
    const std::string &channel_name) {
  std::lock_guard<std::mutex> lck(channel_updater_map_mutex_);
  if (channel_updaters_.find(channel_name) == channel_updaters_.end()) {
    channel_updaters_[channel_name] = new CameraChannelUpdater(channel_name);
    channel_updaters_[channel_name]->perception_camera_reader_ =
        node_->CreateReader<drivers::Image>(
            channel_name,
            [channel_name, this](const std::shared_ptr<drivers::Image> &image) {
              OnImage(image, channel_name);
            });
    // get obstacle channel reader
    channel_updaters_[channel_name]->perception_obstacle_reader_ =
        GetObstacleReader(channel_name);
  }
  return channel_updaters_[channel_name];
}

void PerceptionCameraUpdater::Stop() {
  channel_updaters_.clear();
}

void PerceptionCameraUpdater::StopStream(const std::string &channel_name) {
  if (channel_name.empty()) {
    AERROR << "Failed to unsubscribe channel for channel is empty!";
    return;
  }
  CameraChannelUpdater *updater = GetCameraChannelUpdater(channel_name);
  if (updater->enabled_) {
    if (updater->timer_) {
      updater->timer_->Stop();
    }
    updater->localization_queue_.clear();
    updater->image_buffer_.clear();
    updater->current_image_timestamp_ = 0.0;
    updater->camera_update_.Clear();
    updater->enabled_ = false;
  }
}

void PerceptionCameraUpdater::OnTimer(const std::string &channel_name) {
  PublishMessage(channel_name);
}

void PerceptionCameraUpdater::PublishMessage(const std::string &channel_name) {
  std::string to_send;
  // the channel has no data input, clear the sending object.
  if (!channel_updaters_[channel_name]
           ->perception_camera_reader_->HasWriter()) {
    CameraChannelUpdater *updater = GetCameraChannelUpdater(channel_name);
    updater->image_buffer_.clear();
    updater->current_image_timestamp_ = 0.0;
    updater->camera_update_.Clear();
    to_send = "";
  } else {
    GetUpdate(&to_send, channel_name);
  }
  StreamData stream_data;
  std::string stream_data_string;
  stream_data.set_action("stream");
  stream_data.set_data_name("camera");
  stream_data.set_channel_name(channel_name);
  std::vector<uint8_t> byte_data(to_send.begin(), to_send.end());
  stream_data.set_data(&(byte_data[0]), byte_data.size());
  stream_data.set_type("camera");
  stream_data.SerializeToString(&stream_data_string);
  websocket_->BroadcastBinaryData(stream_data_string);
}

void PerceptionCameraUpdater::GetImageLocalization(
    std::vector<double> *localization, const std::string &channel_name) {
  CameraChannelUpdater *updater = GetCameraChannelUpdater(channel_name);
  if (updater->localization_queue_.empty()) {
    // AERROR << "Localization queue is empty, cannot get localization for
    // image,"
    //        << "image_timestamp: " << current_image_timestamp_;
    return;
  }

  double timestamp_diff = std::numeric_limits<double>::max();
  Pose image_pos;
  while (!updater->localization_queue_.empty()) {
    const double tmp_diff =
        updater->localization_queue_.front()->measurement_time() -
        updater->current_image_timestamp_;
    if (tmp_diff < 0) {
      timestamp_diff = tmp_diff;
      image_pos = updater->localization_queue_.front()->pose();
      if (updater->localization_queue_.size() > 1) {
        updater->localization_queue_.pop_front();
      } else {
        // At least keep one pose in queue, in case there's no localization
        // coming between two requests.
        break;
      }
    } else {
      if (tmp_diff < std::fabs(timestamp_diff)) {
        image_pos = updater->localization_queue_.front()->pose();
      }
      break;
    }
  }

  Eigen::Matrix4d localization_matrix;
  ConstructTransformationMatrix(image_pos.orientation(), image_pos.position(),
                                &localization_matrix);
  ConvertMatrixToArray(localization_matrix, localization);
}

bool PerceptionCameraUpdater::QueryStaticTF(const std::string &frame_id,
                                            const std::string &child_frame_id,
                                            Eigen::Matrix4d *matrix) {
  TransformStamped transform;
  if (tf_buffer_->GetLatestStaticTF(frame_id, child_frame_id, &transform)) {
    ConstructTransformationMatrix(transform.transform().rotation(),
                                  transform.transform().translation(), matrix);
    return true;
  }
  return false;
}

void PerceptionCameraUpdater::GetLocalization2CameraTF(
    std::vector<double> *localization2camera_tf) {
  Eigen::Matrix4d localization2camera_mat = Eigen::Matrix4d::Identity();

  // Since "/tf" topic has dynamic updates of world->novatel and
  // world->localization, novatel->localization in tf buffer is being changed
  // and their transformation does not represent for static transform anymore.
  // Thus we query static transform respectively and calculate by ourselves
  Eigen::Matrix4d loc2novatel_mat;
  if (QueryStaticTF("localization", "novatel", &loc2novatel_mat)) {
    localization2camera_mat *= loc2novatel_mat;
  }

  Eigen::Matrix4d novatel2lidar_mat;
  if (QueryStaticTF("novatel", "velodyne128", &novatel2lidar_mat)) {
    localization2camera_mat *= novatel2lidar_mat;
  }

  Eigen::Matrix4d lidar2camera_mat;
  if (QueryStaticTF("velodyne128", "front_6mm", &lidar2camera_mat)) {
    localization2camera_mat *= lidar2camera_mat;
  }

  ConvertMatrixToArray(localization2camera_mat, localization2camera_tf);
}

// void PerceptionCameraUpdater::OnCompressedImage(
//     const std::shared_ptr<CompressedImage> &compressed_image) {
//   if (!enabled_ ||
//       compressed_image->format() == "h265" /* skip video format */) {
//     return;
//   }

//   std::vector<uint8_t> compressed_raw_data(compressed_image->data().begin(),
//                                            compressed_image->data().end());
//   cv::Mat mat_image = cv::imdecode(compressed_raw_data, cv::IMREAD_COLOR);
//   const int width = mat_image.cols;
//   const int height = mat_image.rows;

//   // Scale down image size properly to reduce data transfer latency through
//   // websocket and ensure image quality is acceptable meanwhile
//   cv::resize(mat_image, mat_image,
//              cv::Size(static_cast<int>(mat_image.cols * kImageScale),
//                       static_cast<int>(mat_image.rows * kImageScale)),
//              0, 0, cv::INTER_LINEAR);
//   std::vector<uint8_t> tmp_buffer;
//   cv::imencode(".jpg", mat_image, tmp_buffer, std::vector<int>() /* params
//   */);

//   double next_image_timestamp;
//   if (compressed_image->has_measurement_time()) {
//     next_image_timestamp = compressed_image->measurement_time();
//   } else {
//     next_image_timestamp = compressed_image->header().timestamp_sec();
//   }

//   std::lock_guard<std::mutex> lock(image_mutex_);
//   if (next_image_timestamp < current_image_timestamp_) {
//     // If replay different bags, the timestamp may jump to earlier time and
//     // we need to clear the localization queue
//     localization_queue_.clear();
//   }
//   current_image_timestamp_ = next_image_timestamp;
//   camera_update_.set_image(&(tmp_buffer[0]), tmp_buffer.size());
//   camera_update_.set_image_aspect_ratio(static_cast<double>(width) / height);
// }

void PerceptionCameraUpdater::OnImage(
    const std::shared_ptr<apollo::drivers::Image> &image,
    const std::string& channel_name) {
  CameraChannelUpdater* updater = GetCameraChannelUpdater(channel_name);
  if (!updater->enabled_) {
    return;
  }
  cv::Mat mat(image->height(), image->width(), CV_8UC3,
              const_cast<char *>(image->data().data()), image->step());
  cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
  cv::resize(mat, mat,
             cv::Size(static_cast<int>(image->width() * kImageScale),
                      static_cast<int>(image->height() * kImageScale)),
             0, 0, cv::INTER_LINEAR);
  cv::imencode(".jpg", mat, updater->image_buffer_, std::vector<int>());
  double next_image_timestamp;
  if (image->has_measurement_time()) {
    next_image_timestamp = image->measurement_time();
  } else {
    next_image_timestamp = image->header().timestamp_sec();
  }
  std::lock_guard<std::mutex> lock(updater->image_mutex_);
  if (next_image_timestamp < updater->current_image_timestamp_) {
    updater->localization_queue_.clear();
  }
  updater->current_image_timestamp_ = next_image_timestamp;
  updater->camera_update_.set_image(&(updater->image_buffer_[0]),
                                    updater->image_buffer_.size());
  updater->camera_update_.set_k_image_scale(kImageScale);
}

void PerceptionCameraUpdater::OnLocalization(
    const std::shared_ptr<LocalizationEstimate> &localization) {
  for (auto iter = channel_updaters_.begin(); iter != channel_updaters_.end();
       iter++) {
    if (iter->second->enabled_) {
      std::lock_guard<std::mutex> lock(iter->second->localization_mutex_);
      iter->second->localization_queue_.push_back(localization);
    }
  }
}

void PerceptionCameraUpdater::OnObstacles(
    const std::shared_ptr<apollo::perception::PerceptionObstacles> &obstacles,
    const std::string &channel_name) {
  CameraChannelUpdater *updater = GetCameraChannelUpdater(channel_name);
  perception_obstacle_enable_ = true;
  std::lock_guard<std::mutex> lock(updater->obstacle_mutex_);
  updater->bbox2ds.clear();
  updater->obstacle_id.clear();
  updater->obstacle_sub_type.clear();
  for (const auto &obstacle : obstacles->perception_obstacle()) {
    updater->bbox2ds.push_back(obstacle.bbox2d());
    updater->obstacle_id.push_back(obstacle.id());
    updater->obstacle_sub_type.push_back(obstacle.sub_type());
  }
}

void PerceptionCameraUpdater::InitReaders() {
  node_->CreateReader<LocalizationEstimate>(
      FLAGS_localization_topic,
      [this](const std::shared_ptr<LocalizationEstimate> &localization) {
        OnLocalization(localization);
      });
}

void PerceptionCameraUpdater::GetUpdate(std::string *camera_update,
                                        const std::string &channel_name) {
  {
    std::vector<double> localization;
    GetImageLocalization(&localization, channel_name);
    CameraChannelUpdater *updater = GetCameraChannelUpdater(channel_name);
    std::lock(updater->image_mutex_, updater->localization_mutex_,
              updater->obstacle_mutex_);
    std::lock_guard<std::mutex> lock1(updater->image_mutex_, std::adopt_lock);
    std::lock_guard<std::mutex> lock2(updater->localization_mutex_,
                                      std::adopt_lock);
    std::lock_guard<std::mutex> lock3(updater->obstacle_mutex_,
                                      std::adopt_lock);
    *(updater->camera_update_).mutable_localization() = {localization.begin(),
                                                         localization.end()};
    std::vector<double> localization2camera_tf;
    GetLocalization2CameraTF(&localization2camera_tf);
    *(updater->camera_update_).mutable_localization2camera_tf() = {
        localization2camera_tf.begin(), localization2camera_tf.end()};
    // Concurrently modify protobuf msg can cause ByteSizeConsistencyError
    // when serializing, so we need lock.
    if (perception_obstacle_enable_) {
      *(updater->camera_update_).mutable_bbox2d() = {updater->bbox2ds.begin(),
                                                     updater->bbox2ds.end()};
      *(updater->camera_update_).mutable_obstacles_id() = {
          updater->obstacle_id.begin(), updater->obstacle_id.end()};
      *(updater->camera_update_).mutable_obstacles_sub_type() = {
          updater->obstacle_sub_type.begin(), updater->obstacle_sub_type.end()};
    }
    updater->camera_update_.SerializeToString(camera_update);
  }
}
void PerceptionCameraUpdater::GetChannelMsg(
    std::vector<std::string> *channels) {
  GetChannelMsgWithFilter(channels, "drivers.Image", "/raw/image", true);
}

std::shared_ptr<cyber::Reader<apollo::perception::PerceptionObstacles>>
PerceptionCameraUpdater::GetObstacleReader(const std::string &channel_name) {
  // TODO(fanyueqiao): Temporarily supports the demonstration of the demo,
  // and needs to adapt to the changes of the new perception module in the
  // future.
  size_t camera_string_end_pos = channel_name.find_last_of("/");
  size_t camera_string_start_pos =
      channel_name.find_last_of("/", camera_string_end_pos - 1);
  std::string camera_name =
      channel_name.substr(camera_string_start_pos + 1,
                          camera_string_end_pos - camera_string_start_pos - 1);
  std::string perception_obstacle_channel =
      "/apollo/prediction/perception_obstacles_" + camera_name;
  auto channel_manager =
      apollo::cyber::service_discovery::TopologyManager::Instance()
          ->channel_manager();
  std::shared_ptr<cyber::Reader<apollo::perception::PerceptionObstacles>>
      perception_obstacle_reader;
  // The channel is invalid or there is no data to writer.
  if (!channel_manager->HasWriter(perception_obstacle_channel)) {
    perception_obstacle_reader =
        node_->CreateReader<apollo::perception::PerceptionObstacles>(
            FLAGS_perception_obstacle_topic,
            [channel_name, this](
                const std::shared_ptr<apollo::perception::PerceptionObstacles>
                    &obstacles) { OnObstacles(obstacles, channel_name); });

  } else {
    perception_obstacle_reader =
        node_->CreateReader<apollo::perception::PerceptionObstacles>(
            perception_obstacle_channel,
            [channel_name, this](
                const std::shared_ptr<apollo::perception::PerceptionObstacles>
                    &obstacles) { OnObstacles(obstacles, channel_name); });
  }
  return perception_obstacle_reader;
}

}  // namespace dreamview
}  // namespace apollo
