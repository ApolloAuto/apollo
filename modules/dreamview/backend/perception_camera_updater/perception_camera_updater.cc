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

#include <float.h>
#include <map>
#include <string>
#include "cyber/common/file.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/proto/geometry.pb.h"
#include "opencv2/opencv.hpp"

namespace apollo {
namespace dreamview {

using apollo::drivers::CompressedImage;
using apollo::localization::LocalizationEstimate;
using apollo::localization::Pose;
using apollo::transform::TransformStampeds;
using apollo::transform::Transform;
using apollo::common::Quaternion;

namespace {
  void ConvertMatrixToArray(
      const Eigen::Matrix4d &matrix, std::vector<double> *array) {
    const double* pointer = matrix.data();
    for (int i = 0; i < matrix.size(); ++i) {
      array->push_back(pointer[i]);
    }
  }

  template <typename Point>
  void ConstructTransformationMatrix(const Quaternion &quaternion,
      const Point &translation, Eigen::Matrix4d &matrix) {
    matrix.setConstant(0);
    Eigen::Quaterniond q;
    q.x() = quaternion.qx();
    q.y() = quaternion.qy();
    q.z() = quaternion.qz();
    q.w() = quaternion.qw();
    matrix.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    matrix(0, 3) = translation.x();
    matrix(1, 3) = translation.y();
    matrix(2, 3) = translation.z();
    matrix(3, 3) = 1;
  }
}  // namespace

PerceptionCameraUpdater::PerceptionCameraUpdater(WebSocketHandler* websocket)
    : websocket_(websocket),
      node_(cyber::CreateNode("perception_camera_updater")) {
  InitReaders();
}

void PerceptionCameraUpdater::Start() {
  enabled_ = true;
}

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
  double timestamp_diff = DBL_MAX;
  auto iter = localization_queue_.rbegin();
  Pose image_pos;
  for (; iter != localization_queue_.rend(); ++iter) {
    const double tmp_diff = (*iter)->measurement_time()
        - current_image_timestamp_;
    if (tmp_diff > 0) {
      timestamp_diff = tmp_diff;
      image_pos = (*iter)->pose();
    } else {
      if (std::fabs(tmp_diff) < timestamp_diff) {
        image_pos = (*iter)->pose();
      }
      break;
    }
  }
  // clean useless localization
  while (localization_queue_.front() != (*iter)) {
    localization_queue_.pop_front();
  }

  Eigen::Matrix4d localization_matrix;
  ConstructTransformationMatrix(image_pos.orientation(), image_pos.position(),
      localization_matrix);
  ConvertMatrixToArray(localization_matrix, localization);
}

void PerceptionCameraUpdater::OnImage(
    const std::shared_ptr<CompressedImage> &compressed_image) {
  if (!enabled_
      || compressed_image->format() == "h265" /* skip video format */) {
    return;
  }

  std::vector<uint8_t> compressed_raw_data(compressed_image->data().begin(),
                                           compressed_image->data().end());
  cv::Mat mat_image = cv::imdecode(compressed_raw_data, CV_LOAD_IMAGE_COLOR);
  // Scale down image size properly to reduce data transfer latency through
  // websocket and ensure image quality is acceptable meanwhile
  cv::resize(
      mat_image, mat_image,
      cv::Size(static_cast<int>(mat_image.cols * kImageScale),
               static_cast<int>(mat_image.rows * kImageScale)),
      0, 0, CV_INTER_LINEAR);
  std::vector<uint8_t> tmp_buffer;
  cv::imencode(".jpg", mat_image, tmp_buffer, std::vector<int>() /* params */);

  std::lock_guard<std::mutex> lock(image_mutex_);
  if (compressed_image->has_measurement_time()) {
    current_image_timestamp_ = compressed_image->measurement_time();
  } else {
    current_image_timestamp_ = compressed_image->header().timestamp_sec();
  }
  camera_update_.set_data(&(tmp_buffer[0]), tmp_buffer.size());
}

void PerceptionCameraUpdater::OnLocalization(
    const std::shared_ptr<LocalizationEstimate> &localization) {
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(localization_mutex_);
  localization_queue_.push_back(localization);
}

void PerceptionCameraUpdater::OnStaticTransform(
    const std::shared_ptr<TransformStampeds> &static_tf) {
  if (static_tf->transforms().size() > 0) {
    std::map<std::string, Eigen::Matrix4d> transforms;
    for (const auto &tf : static_tf->transforms()) {
      const std::string frame_id = tf.header().frame_id();
      const std::string child_frame_id = tf.child_frame_id();
      Transform transform = tf.transform();
      Eigen::Matrix4d matrix;
      ConstructTransformationMatrix(transform.rotation(),
          transform.translation(), matrix);
      std::string key = frame_id + "_" + child_frame_id;
      transforms[key] = matrix;
    }
    // Camera-to-localization transformation matrix can be calculated by
    // camera->lidar->imu->localization extrinsic parameters, refer to
    // modules/perception/camera/tools/Visualizer::Init_all_info_single_camera
    Eigen::Matrix4d localization2camera_mat = Eigen::Matrix4d::Identity();
    if (transforms.find("localization_novatel") != transforms.end()) {
      localization2camera_mat *= transforms["localization_novatel"];
    }
    if (transforms.find("novatel_velodyne128") != transforms.end()) {
      localization2camera_mat *= transforms["novatel_velodyne128"];
    }
    if (transforms.find("velodyne128_front_6mm") != transforms.end()) {
      localization2camera_mat *= transforms["velodyne128_front_6mm"];
    }
    std::vector<double> localization2camera_arr;
    ConvertMatrixToArray(localization2camera_mat, &localization2camera_arr);
    *camera_update_.mutable_tf_static() = {localization2camera_arr.begin(),
                                           localization2camera_arr.end()};
  }
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

  node_->CreateReader<TransformStampeds>(
      FLAGS_tf_static_topic,
      [this](const std::shared_ptr<TransformStampeds> &static_transform) {
        OnStaticTransform(static_transform);
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

    camera_update_.SerializeToString(camera_update);
  }
}

}  // namespace dreamview
}  // namespace apollo
