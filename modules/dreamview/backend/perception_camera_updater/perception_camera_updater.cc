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

#include "modules/dreamview/backend/perception_camera_updater/perception_camera_updater.h"

#include <cfloat>
#include <string>

#include "cyber/common/file.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/proto/geometry.pb.h"
#include "opencv2/opencv.hpp"

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

void PerceptionCameraUpdater::Start() { enabled_ = true; }

void PerceptionCameraUpdater::Stop() {
  if (enabled_) {
    localization_queue_.clear();
    image_buffer_.clear();
    tf_static_.clear();
    current_image_timestamp_ = 0.0;
  }
  enabled_ = false;
}

void PerceptionCameraUpdater::GetImageLocalization(
    std::vector<double> *localization) {
  if (localization_queue_.empty()) {
    AERROR << "Localization queue is empty, cannot get localization for image,"
           << "image_timestamp: " << current_image_timestamp_;
    return;
  }

  double timestamp_diff = DBL_MAX;
  Pose image_pos;
  while (!localization_queue_.empty()) {
    const double tmp_diff = localization_queue_.front()->measurement_time() -
                            current_image_timestamp_;
    if (tmp_diff < 0) {
      timestamp_diff = tmp_diff;
      image_pos = localization_queue_.front()->pose();
      if (localization_queue_.size() > 1) {
        localization_queue_.pop_front();
      } else {
        // At least keep one pose in queue, in case there's no localization
        // coming between two requests.
        break;
      }
    } else {
      if (tmp_diff < std::fabs(timestamp_diff)) {
        image_pos = localization_queue_.front()->pose();
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

void PerceptionCameraUpdater::OnImage(
    const std::shared_ptr<CompressedImage> &compressed_image) {
  if (!enabled_ ||
      compressed_image->format() == "h265" /* skip video format */) {
    return;
  }

  std::vector<uint8_t> compressed_raw_data(compressed_image->data().begin(),
                                           compressed_image->data().end());
  cv::Mat mat_image = cv::imdecode(compressed_raw_data, CV_LOAD_IMAGE_COLOR);
  const int width = mat_image.cols;
  const int height = mat_image.rows;

  // Scale down image size properly to reduce data transfer latency through
  // websocket and ensure image quality is acceptable meanwhile
  cv::resize(mat_image, mat_image,
             cv::Size(static_cast<int>(mat_image.cols * kImageScale),
                      static_cast<int>(mat_image.rows * kImageScale)),
             0, 0, CV_INTER_LINEAR);
  std::vector<uint8_t> tmp_buffer;
  cv::imencode(".jpg", mat_image, tmp_buffer, std::vector<int>() /* params */);

  double next_image_timestamp;
  if (compressed_image->has_measurement_time()) {
    next_image_timestamp = compressed_image->measurement_time();
  } else {
    next_image_timestamp = compressed_image->header().timestamp_sec();
  }

  std::lock_guard<std::mutex> lock(image_mutex_);
  if (next_image_timestamp < current_image_timestamp_) {
    // If replay different bags, the timestamp may jump to earlier time and
    // we need to clear the localization queue
    localization_queue_.clear();
  }
  current_image_timestamp_ = next_image_timestamp;
  camera_update_.set_image(&(tmp_buffer[0]), tmp_buffer.size());
  camera_update_.set_image_aspect_ratio(static_cast<double>(width) / height);
}

void PerceptionCameraUpdater::OnLocalization(
    const std::shared_ptr<LocalizationEstimate> &localization) {
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(localization_mutex_);
  localization_queue_.push_back(localization);
}

void PerceptionCameraUpdater::InitReaders() {
  node_->CreateReader<CompressedImage>(
      FLAGS_image_short_topic,
      [this](const std::shared_ptr<CompressedImage> &image) {
        OnImage(image);
      });

  node_->CreateReader<LocalizationEstimate>(
      FLAGS_localization_topic,
      [this](const std::shared_ptr<LocalizationEstimate> &localization) {
        OnLocalization(localization);
      });
}

void PerceptionCameraUpdater::GetUpdate(std::string *camera_update) {
  {
    std::lock(image_mutex_, localization_mutex_);
    std::lock_guard<std::mutex> lock1(image_mutex_, std::adopt_lock);
    std::lock_guard<std::mutex> lock2(localization_mutex_, std::adopt_lock);

    std::vector<double> localization;
    GetImageLocalization(&localization);
    *camera_update_.mutable_localization() = {localization.begin(),
                                              localization.end()};
    std::vector<double> localization2camera_tf;
    GetLocalization2CameraTF(&localization2camera_tf);
    *camera_update_.mutable_localization2camera_tf() = {
        localization2camera_tf.begin(), localization2camera_tf.end()};
    // Concurrently modify protobuf msg can cause ByteSizeConsistencyError
    // when serializing, so we need lock.
    camera_update_.SerializeToString(camera_update);
  }
}

}  // namespace dreamview
}  // namespace apollo
