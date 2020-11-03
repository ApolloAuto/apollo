/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/onboard/transform_wrapper/transform_wrapper.h"

#include <limits>

#include "cyber/common/log.h"
#include "modules/common/util/string_util.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"

namespace apollo {
namespace perception {
namespace onboard {

DEFINE_string(obs_sensor2novatel_tf2_frame_id, "novatel",
              "sensor to novatel frame id");
DEFINE_string(obs_novatel2world_tf2_frame_id, "world",
              "novatel to world frame id");
DEFINE_string(obs_novatel2world_tf2_child_frame_id, "novatel",
              "novatel to world child frame id");
DEFINE_double(obs_tf2_buff_size, 0.01, "query Cyber TF buffer size in second");
DEFINE_double(obs_transform_cache_size, 1.0, "transform cache size in second");
DEFINE_double(obs_max_local_pose_extrapolation_latency, 0.15,
              "max local pose extrapolation period in second");
DEFINE_bool(obs_enable_local_pose_extrapolation, true,
            "use local pose extrapolation");

void TransformCache::AddTransform(const StampedTransform& transform) {
  if (transforms_.empty()) {
    transforms_.push_back(transform);
    return;
  }
  double delt = transform.timestamp - transforms_.back().timestamp;
  if (delt < 0.0) {
    AINFO << "ERROR: add earlier transform to transform cache";
    return;
  }

  do {
    delt = transform.timestamp - transforms_.front().timestamp;
    if (delt < cache_duration_) {
      break;
    }
    transforms_.pop_front();
  } while (!transforms_.empty());

  transforms_.push_back(transform);
}

Eigen::Quaterniond Slerp(const Eigen::Quaterniond& source, const double& t,
                         const Eigen::Quaterniond& other) {
  const double one = 1.0 - std::numeric_limits<double>::epsilon();
  double d = source.x() * other.x() + source.y() * other.y() +
             source.z() * other.z() + source.w() * other.w();
  double abs_d = std::abs(d);

  double scale0;
  double scale1;

  if (abs_d >= one) {
    scale0 = 1.0 - t;
    scale1 = t;
  } else {
    // theta is the angle between the 2 quaternions
    double theta = std::acos(abs_d);
    double sin_theta = std::sin(theta);

    scale0 = std::sin((1.0 - t) * theta) / sin_theta;
    scale1 = std::sin((t * theta)) / sin_theta;
  }
  if (d < 0) scale1 = -scale1;

  return Eigen::Quaterniond(scale0 * source.w() + scale1 * other.w(),
                            scale0 * source.x() + scale1 * other.x(),
                            scale0 * source.y() + scale1 * other.y(),
                            scale0 * source.z() + scale1 * other.z());
}

bool TransformCache::QueryTransform(double timestamp,
                                    StampedTransform* transform,
                                    double max_duration) {
  if (transforms_.empty() || transform == nullptr) {
    return false;
  }

  double delt = timestamp - transforms_.back().timestamp;
  if (delt > max_duration) {
    AINFO << "ERROR: query timestamp is " << delt
          << "s later than cached timestamp";
    return false;
  } else if (delt < 0.0) {
    AINFO << "ERROR: query earlier timestamp than transform cache";
    return false;
  }

  int size = static_cast<int>(transforms_.size());
  if (size == 1) {
    (*transform) = transforms_.back();
    transform->timestamp = timestamp;
    AINFO << "use transform at " << transforms_.back().timestamp << " for "
          << timestamp;
  } else {
    double ratio =
        (timestamp - transforms_[size - 2].timestamp) /
        (transforms_[size - 1].timestamp - transforms_[size - 2].timestamp);

    transform->rotation = Slerp(transforms_[size - 2].rotation, ratio,
                                transforms_[size - 1].rotation);

    transform->translation.x() =
        transforms_[size - 2].translation.x() * (1 - ratio) +
        transforms_[size - 1].translation.x() * ratio;
    transform->translation.y() =
        transforms_[size - 2].translation.y() * (1 - ratio) +
        transforms_[size - 1].translation.y() * ratio;
    transform->translation.z() =
        transforms_[size - 2].translation.z() * (1 - ratio) +
        transforms_[size - 1].translation.z() * ratio;

    AINFO << "estimate pose at " << timestamp << " from poses at "
          << transforms_[size - 2].timestamp << " and "
          << transforms_[size - 1].timestamp;
  }
  return true;
}

void TransformWrapper::Init(
    const std::string& sensor2novatel_tf2_child_frame_id) {
  sensor2novatel_tf2_frame_id_ = FLAGS_obs_sensor2novatel_tf2_frame_id;
  sensor2novatel_tf2_child_frame_id_ = sensor2novatel_tf2_child_frame_id;
  novatel2world_tf2_frame_id_ = FLAGS_obs_novatel2world_tf2_frame_id;
  novatel2world_tf2_child_frame_id_ =
      FLAGS_obs_novatel2world_tf2_child_frame_id;
  transform_cache_.SetCacheDuration(FLAGS_obs_transform_cache_size);
  inited_ = true;
}

void TransformWrapper::Init(
    const std::string& sensor2novatel_tf2_frame_id,
    const std::string& sensor2novatel_tf2_child_frame_id,
    const std::string& novatel2world_tf2_frame_id,
    const std::string& novatel2world_tf2_child_frame_id) {
  sensor2novatel_tf2_frame_id_ = sensor2novatel_tf2_frame_id;
  sensor2novatel_tf2_child_frame_id_ = sensor2novatel_tf2_child_frame_id;
  novatel2world_tf2_frame_id_ = novatel2world_tf2_frame_id;
  novatel2world_tf2_child_frame_id_ = novatel2world_tf2_child_frame_id;
  transform_cache_.SetCacheDuration(FLAGS_obs_transform_cache_size);
  inited_ = true;
}

bool TransformWrapper::GetSensor2worldTrans(
    double timestamp, Eigen::Affine3d* sensor2world_trans,
    Eigen::Affine3d* novatel2world_trans) {
  if (!inited_) {
    AERROR << "TransformWrapper not Initialized,"
           << " unable to call GetSensor2worldTrans.";
    return false;
  }

  if (sensor2novatel_extrinsics_ == nullptr) {
    StampedTransform trans_sensor2novatel;
    if (!QueryTrans(timestamp, &trans_sensor2novatel,
                    sensor2novatel_tf2_frame_id_,
                    sensor2novatel_tf2_child_frame_id_)) {
      return false;
    }
    sensor2novatel_extrinsics_.reset(new Eigen::Affine3d);
    *sensor2novatel_extrinsics_ =
        trans_sensor2novatel.translation * trans_sensor2novatel.rotation;
    AINFO << "Get sensor2novatel extrinsics successfully.";
  }

  StampedTransform trans_novatel2world;
  trans_novatel2world.timestamp = timestamp;
  Eigen::Affine3d novatel2world;

  if (!QueryTrans(timestamp, &trans_novatel2world, novatel2world_tf2_frame_id_,
                  novatel2world_tf2_child_frame_id_)) {
    if (FLAGS_obs_enable_local_pose_extrapolation) {
      if (!transform_cache_.QueryTransform(
              timestamp, &trans_novatel2world,
              FLAGS_obs_max_local_pose_extrapolation_latency)) {
        return false;
      }
    } else {
      return false;
    }
  } else if (FLAGS_obs_enable_local_pose_extrapolation) {
    transform_cache_.AddTransform(trans_novatel2world);
  }

  novatel2world =
      trans_novatel2world.translation * trans_novatel2world.rotation;
  *sensor2world_trans = novatel2world * (*sensor2novatel_extrinsics_);
  if (novatel2world_trans != nullptr) {
    *novatel2world_trans = novatel2world;
  }
  AINFO << "Get pose timestamp: " << timestamp << ", pose: \n"
        << (*sensor2world_trans).matrix();
  return true;
}

bool TransformWrapper::GetExtrinsics(Eigen::Affine3d* trans) {
  if (!inited_ || trans == nullptr || sensor2novatel_extrinsics_ == nullptr) {
    AERROR << "TransformWrapper get extrinsics failed";
    return false;
  }
  *trans = *sensor2novatel_extrinsics_;
  return true;
}

bool TransformWrapper::GetTrans(double timestamp, Eigen::Affine3d* trans,
                                const std::string& frame_id,
                                const std::string& child_frame_id) {
  StampedTransform transform;
  if (!QueryTrans(timestamp, &transform, frame_id, child_frame_id)) {
    if (!FLAGS_obs_enable_local_pose_extrapolation ||
        !transform_cache_.QueryTransform(
            timestamp, &transform,
            FLAGS_obs_max_local_pose_extrapolation_latency)) {
      return false;
    }
  }

  if (FLAGS_obs_enable_local_pose_extrapolation) {
    transform_cache_.AddTransform(transform);
  }

  *trans = transform.translation * transform.rotation;
  return true;
}

bool TransformWrapper::QueryTrans(double timestamp, StampedTransform* trans,
                                  const std::string& frame_id,
                                  const std::string& child_frame_id) {
  cyber::Time query_time(timestamp);
  std::string err_string;
  if (!tf2_buffer_->canTransform(frame_id, child_frame_id, query_time,
                                 static_cast<float>(FLAGS_obs_tf2_buff_size),
                                 &err_string)) {
    AERROR << "Can not find transform. " << FORMAT_TIMESTAMP(timestamp)
           << " frame_id: " << frame_id << " child_frame_id: " << child_frame_id
           << " Error info: " << err_string;
    return false;
  }

  apollo::transform::TransformStamped stamped_transform;
  try {
    stamped_transform =
        tf2_buffer_->lookupTransform(frame_id, child_frame_id, query_time);

    trans->translation =
        Eigen::Translation3d(stamped_transform.transform().translation().x(),
                             stamped_transform.transform().translation().y(),
                             stamped_transform.transform().translation().z());
    trans->rotation =
        Eigen::Quaterniond(stamped_transform.transform().rotation().qw(),
                           stamped_transform.transform().rotation().qx(),
                           stamped_transform.transform().rotation().qy(),
                           stamped_transform.transform().rotation().qz());
  } catch (tf2::TransformException& ex) {
    AERROR << ex.what();
    return false;
  }
  return true;
}

bool TransformWrapper::GetExtrinsicsBySensorId(
    const std::string& from_sensor_id, const std::string& to_sensor_id,
    Eigen::Affine3d* trans) {
  if (trans == nullptr) {
    AERROR << "TransformWrapper get extrinsics failed";
    return false;
  }

  common::SensorManager* sensor_manager = common::SensorManager::Instance();
  std::string frame_id = sensor_manager->GetFrameId(to_sensor_id);
  std::string child_frame_id = sensor_manager->GetFrameId(from_sensor_id);

  StampedTransform transform;
  bool status = QueryTrans(0.0, &transform, frame_id, child_frame_id);
  if (status) {
    *trans = transform.translation * transform.rotation;
  }
  return status;
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
