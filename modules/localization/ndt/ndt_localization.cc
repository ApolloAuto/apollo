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

#include "modules/localization/ndt/ndt_localization.h"

#include "Eigen/Geometry"
#include "yaml-cpp/yaml.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {
namespace ndt {

void NDTLocalization::Init() {
  tf_buffer_ = apollo::transform::Buffer::Instance();
  tf_buffer_->Init();

  resolution_id_ = 0;
  zone_id_ = FLAGS_local_utm_zone_id;
  online_resolution_ = FLAGS_online_resolution;
  ndt_debug_log_flag_ = FLAGS_ndt_debug_log_flag;
  tf_source_frame_id_ = FLAGS_broadcast_tf_frame_id;
  tf_target_frame_id_ = FLAGS_broadcast_tf_child_frame_id;
  std::string lidar_height_file = FLAGS_lidar_height_file;
  std::string lidar_extrinsics_file = FLAGS_lidar_extrinsics_file;
  bad_score_count_threshold_ = FLAGS_ndt_bad_score_count_threshold;
  warnning_ndt_score_ = FLAGS_ndt_warnning_ndt_score;
  error_ndt_score_ = FLAGS_ndt_error_ndt_score;

  std::string map_path_ =
      FLAGS_map_dir + "/" + FLAGS_ndt_map_dir + "/" + FLAGS_local_map_name;
  AINFO << "map folder: " << map_path_;
  velodyne_extrinsic_ = Eigen::Affine3d::Identity();
  bool success =
      LoadLidarExtrinsic(lidar_extrinsics_file, &velodyne_extrinsic_);
  if (!success) {
    AERROR << "LocalizationLidar: Fail to access the lidar"
              " extrinsic file: "
           << lidar_extrinsics_file;
    return;
  }
  Eigen::Quaterniond ext_quat(velodyne_extrinsic_.linear());
  AINFO << "lidar extrinsics: " << velodyne_extrinsic_.translation().x() << ", "
        << velodyne_extrinsic_.translation().y() << ", "
        << velodyne_extrinsic_.translation().z() << ", " << ext_quat.x() << ", "
        << ext_quat.y() << ", " << ext_quat.z() << ", " << ext_quat.w();

  success = LoadLidarHeight(lidar_height_file, &lidar_height_);
  if (!success) {
    AWARN << "LocalizationLidar: Fail to load the lidar"
             " height file: "
          << lidar_height_file << " Will use default value!";
    lidar_height_.height = FLAGS_lidar_height_default;
  }

  // try load zone id from local_map folder
  if (FLAGS_if_utm_zone_id_from_folder) {
    bool success = LoadZoneIdFromFolder(map_path_, &zone_id_);
    if (!success) {
      AWARN << "Can't load utm zone id from map folder, use default value.";
    }
  }
  AINFO << "utm zone id: " << zone_id_;

  lidar_locator_.SetMapFolderPath(map_path_);
  lidar_locator_.SetVelodyneExtrinsic(velodyne_extrinsic_);
  lidar_locator_.SetLidarHeight(lidar_height_.height);
  lidar_locator_.SetOnlineCloudResolution(
      static_cast<float>(online_resolution_));

  odometry_buffer_.clear();
  odometry_buffer_size_ = 0;

  is_service_started_ = false;
}
// receive odometry message
void NDTLocalization::OdometryCallback(
    const std::shared_ptr<localization::Gps>& odometry_msg) {
  double odometry_time = odometry_msg->header().timestamp_sec();
  static double pre_odometry_time = odometry_time;
  double time_delay = odometry_time - pre_odometry_time;
  if (time_delay > 0.1) {
    AINFO << "Odometry message loss more than 10ms, the pre time and cur time: "
          << pre_odometry_time << ", " << odometry_time;
  } else if (time_delay < 0.0) {
    AINFO << "Odometry message's time is earlier than last one, "
          << "the pre time and cur time: " << pre_odometry_time << ", "
          << odometry_time;
  }
  pre_odometry_time = odometry_time;
  Eigen::Affine3d odometry_pose = Eigen::Affine3d::Identity();
  if (odometry_msg->has_localization()) {
    odometry_pose.translation()[0] =
        odometry_msg->localization().position().x();
    odometry_pose.translation()[1] =
        odometry_msg->localization().position().y();
    odometry_pose.translation()[2] =
        odometry_msg->localization().position().z();
    Eigen::Quaterniond tmp_quat(
        odometry_msg->localization().orientation().qw(),
        odometry_msg->localization().orientation().qx(),
        odometry_msg->localization().orientation().qy(),
        odometry_msg->localization().orientation().qz());
    odometry_pose.linear() = tmp_quat.toRotationMatrix();
  }
  if (ZeroOdometry(odometry_pose)) {
    AINFO << "Detect Zero Odometry";
    return;
  }

  TimeStampPose timestamp_pose;
  timestamp_pose.timestamp = odometry_time;
  timestamp_pose.pose = odometry_pose;
  {
    std::lock_guard<std::mutex> lock(odometry_buffer_mutex_);
    if (odometry_buffer_size_ < max_odometry_buffer_size_) {
      odometry_buffer_.push_back(timestamp_pose);
      odometry_buffer_size_++;
    } else {
      odometry_buffer_.pop_front();
      odometry_buffer_.push_back(timestamp_pose);
    }
  }

  if (ndt_debug_log_flag_) {
    AINFO << "NDTLocalization Debug Log: odometry msg: "
          << std::setprecision(15) << "time: " << odometry_time << ", "
          << "x: " << odometry_msg->localization().position().x() << ", "
          << "y: " << odometry_msg->localization().position().y() << ", "
          << "z: " << odometry_msg->localization().position().z() << ", "
          << "qx: " << odometry_msg->localization().orientation().qx() << ", "
          << "qy: " << odometry_msg->localization().orientation().qy() << ", "
          << "qz: " << odometry_msg->localization().orientation().qz() << ", "
          << "qw: " << odometry_msg->localization().orientation().qw();
  }

  Eigen::Affine3d localization_pose = Eigen::Affine3d::Identity();
  if (lidar_locator_.IsInitialized()) {
    localization_pose =
        pose_buffer_.UpdateOdometryPose(odometry_time, odometry_pose);
  }
  ComposeLocalizationEstimate(localization_pose, odometry_msg,
                              &localization_result_);
  drivers::gnss::InsStat odometry_status;
  FindNearestOdometryStatus(odometry_time, &odometry_status);
  ComposeLocalizationStatus(odometry_status, &localization_status_);
  is_service_started_ = true;
}

// receive lidar pointcloud message
void NDTLocalization::LidarCallback(
    const std::shared_ptr<drivers::PointCloud>& lidar_msg) {
  static unsigned int frame_idx = 0;
  LidarFrame lidar_frame;
  LidarMsgTransfer(lidar_msg, &lidar_frame);

  double time_stamp = lidar_frame.measurement_time;
  Eigen::Affine3d odometry_pose = Eigen::Affine3d::Identity();
  if (!QueryPoseFromBuffer(time_stamp, &odometry_pose)) {
    if (!QueryPoseFromTF(time_stamp, &odometry_pose)) {
      AERROR << "Can not query forecast pose";
      return;
    }
    AINFO << "Query pose from TF";
  } else {
    AINFO << "Query pose from buffer";
  }
  if (!lidar_locator_.IsInitialized()) {
    lidar_locator_.Init(odometry_pose, resolution_id_, zone_id_);
    return;
  }
  lidar_locator_.Update(frame_idx++, odometry_pose, lidar_frame);
  lidar_pose_ = lidar_locator_.GetPose();
  pose_buffer_.UpdateLidarPose(time_stamp, lidar_pose_, odometry_pose);
  ComposeLidarResult(time_stamp, lidar_pose_, &lidar_localization_result_);
  ndt_score_ = lidar_locator_.GetFitnessScore();
  if (ndt_score_ > warnning_ndt_score_) {
    bad_score_count_++;
  } else {
    bad_score_count_ = 0;
  }
  if (ndt_debug_log_flag_) {
    Eigen::Quaterniond tmp_quat(lidar_pose_.linear());
    AINFO << "NDTLocalization Debug Log: lidar pose: " << std::setprecision(15)
          << "time: " << time_stamp << ", "
          << "x: " << lidar_pose_.translation()[0] << ", "
          << "y: " << lidar_pose_.translation()[1] << ", "
          << "z: " << lidar_pose_.translation()[2] << ", "
          << "qx: " << tmp_quat.x() << ", "
          << "qy: " << tmp_quat.y() << ", "
          << "qz: " << tmp_quat.z() << ", "
          << "qw: " << tmp_quat.w();

    Eigen::Quaterniond qbn(odometry_pose.linear());
    AINFO << "NDTLocalization Debug Log: odometry for lidar pose: "
          << std::setprecision(15) << "time: " << time_stamp << ", "
          << "x: " << odometry_pose.translation()[0] << ", "
          << "y: " << odometry_pose.translation()[1] << ", "
          << "z: " << odometry_pose.translation()[2] << ", "
          << "qx: " << qbn.x() << ", "
          << "qy: " << qbn.y() << ", "
          << "qz: " << qbn.z() << ", "
          << "qw: " << qbn.w();
  }
}

void NDTLocalization::OdometryStatusCallback(
    const std::shared_ptr<drivers::gnss::InsStat>& status_msg) {
  std::unique_lock<std::mutex> lock(odometry_status_list_mutex_);
  if (odometry_status_list_.size() < odometry_status_list_max_size_) {
    odometry_status_list_.push_back(*status_msg);
  } else {
    odometry_status_list_.pop_front();
    odometry_status_list_.push_back(*status_msg);
  }
}
// output localization result
void NDTLocalization::GetLocalization(
    LocalizationEstimate* localization) const {
  *localization = localization_result_;
}

void NDTLocalization::GetLidarLocalization(
    LocalizationEstimate* localization) const {
  *localization = lidar_localization_result_;
}

void NDTLocalization::GetLocalizationStatus(
    LocalizationStatus* localization_status) const {
  *localization_status = localization_status_;
}

bool NDTLocalization::IsServiceStarted() { return is_service_started_; }

void NDTLocalization::FillLocalizationMsgHeader(
    LocalizationEstimate* localization) {
  auto* header = localization->mutable_header();
  double timestamp = apollo::common::time::Clock::NowInSeconds();
  header->set_module_name(module_name_);
  header->set_timestamp_sec(timestamp);
  header->set_sequence_num(++localization_seq_num_);
}

void NDTLocalization::ComposeLocalizationEstimate(
    const Eigen::Affine3d& pose,
    const std::shared_ptr<localization::Gps>& odometry_msg,
    LocalizationEstimate* localization) {
  localization->Clear();
  FillLocalizationMsgHeader(localization);

  localization->set_measurement_time(odometry_msg->header().timestamp_sec());
  auto mutable_pose = localization->mutable_pose();
  mutable_pose->mutable_position()->set_x(pose.translation().x());
  mutable_pose->mutable_position()->set_y(pose.translation().y());
  mutable_pose->mutable_position()->set_z(pose.translation().z());

  Eigen::Quaterniond quat(pose.linear());
  mutable_pose->mutable_orientation()->set_qw(quat.w());
  mutable_pose->mutable_orientation()->set_qx(quat.x());
  mutable_pose->mutable_orientation()->set_qy(quat.y());
  mutable_pose->mutable_orientation()->set_qz(quat.z());
  double heading =
      common::math::QuaternionToHeading(quat.w(), quat.x(), quat.y(), quat.z());
  mutable_pose->set_heading(heading);

  common::math::EulerAnglesZXYd euler(quat.w(), quat.x(), quat.y(), quat.z());
  mutable_pose->mutable_euler_angles()->set_x(euler.pitch());
  mutable_pose->mutable_euler_angles()->set_y(euler.roll());
  mutable_pose->mutable_euler_angles()->set_z(euler.yaw());

  const auto& odometry_pose = odometry_msg->localization();
  if (odometry_pose.has_linear_velocity()) {
    mutable_pose->mutable_linear_velocity()->CopyFrom(
        odometry_pose.linear_velocity());
  }
  if (odometry_pose.has_linear_acceleration()) {
    mutable_pose->mutable_linear_acceleration()->CopyFrom(
        odometry_pose.linear_acceleration());
  }
  if (odometry_pose.has_angular_velocity()) {
    mutable_pose->mutable_angular_velocity()->CopyFrom(
        odometry_pose.angular_velocity());
  }
  if (odometry_pose.has_linear_acceleration_vrf()) {
    mutable_pose->mutable_linear_acceleration_vrf()->CopyFrom(
        odometry_pose.linear_acceleration_vrf());
  }
  if (odometry_pose.has_angular_velocity_vrf()) {
    mutable_pose->mutable_angular_velocity_vrf()->CopyFrom(
        odometry_pose.angular_velocity_vrf());
  }
}

void NDTLocalization::ComposeLidarResult(double time_stamp,
                                         const Eigen::Affine3d& pose,
                                         LocalizationEstimate* localization) {
  localization->Clear();
  FillLocalizationMsgHeader(localization);

  localization->set_measurement_time(time_stamp);
  auto mutable_pose = localization->mutable_pose();
  mutable_pose->mutable_position()->set_x(pose.translation().x());
  mutable_pose->mutable_position()->set_y(pose.translation().y());
  mutable_pose->mutable_position()->set_z(pose.translation().z());

  Eigen::Quaterniond quat(pose.linear());
  mutable_pose->mutable_orientation()->set_qw(quat.w());
  mutable_pose->mutable_orientation()->set_qx(quat.x());
  mutable_pose->mutable_orientation()->set_qy(quat.y());
  mutable_pose->mutable_orientation()->set_qz(quat.z());
  double heading =
      common::math::QuaternionToHeading(quat.w(), quat.x(), quat.y(), quat.z());
  mutable_pose->set_heading(heading);

  common::math::EulerAnglesZXYd euler(quat.w(), quat.x(), quat.y(), quat.z());
  mutable_pose->mutable_euler_angles()->set_x(euler.pitch());
  mutable_pose->mutable_euler_angles()->set_y(euler.roll());
  mutable_pose->mutable_euler_angles()->set_z(euler.yaw());
}

bool NDTLocalization::QueryPoseFromTF(double time, Eigen::Affine3d* pose) {
  cyber::Time query_time(time);
  const float time_out = 0.01f;
  std::string err_msg = "";
  bool can_transform = tf_buffer_->canTransform(
      tf_target_frame_id_, tf_source_frame_id_, query_time, time_out, &err_msg);
  if (!can_transform) {
    AERROR << "Can not transform: " << err_msg;
    return false;
  }
  apollo::transform::TransformStamped query_transform;
  try {
    query_transform = tf_buffer_->lookupTransform(
        tf_target_frame_id_, tf_source_frame_id_, query_time);
  } catch (tf2::TransformException ex) {
    AERROR << ex.what();
    return false;
  }
  pose->translation()[0] = query_transform.transform().translation().x();
  pose->translation()[1] = query_transform.transform().translation().y();
  pose->translation()[2] = query_transform.transform().translation().z();
  Eigen::Quaterniond tmp_quat(query_transform.transform().rotation().qw(),
                              query_transform.transform().rotation().qx(),
                              query_transform.transform().rotation().qy(),
                              query_transform.transform().rotation().qz());
  pose->linear() = tmp_quat.toRotationMatrix();
  return true;
}

void NDTLocalization::ComposeLocalizationStatus(
    const drivers::gnss::InsStat& status,
    LocalizationStatus* localization_status) {
  apollo::common::Header* header = localization_status->mutable_header();
  double timestamp = apollo::common::time::Clock::NowInSeconds();
  header->set_timestamp_sec(timestamp);
  localization_status->set_measurement_time(status.header().timestamp_sec());

  if (!status.has_pos_type()) {
    localization_status->set_fusion_status(MeasureState::ERROR);
    localization_status->set_state_message(
        "Error: Current Localization Status Is Missing.");
    return;
  }

  auto pos_type = static_cast<drivers::gnss::SolutionType>(status.pos_type());
  if (ndt_score_ < warnning_ndt_score_ &&
      pos_type == drivers::gnss::SolutionType::INS_RTKFIXED) {
    localization_status->set_fusion_status(MeasureState::OK);
    localization_status->set_state_message("");
  } else if (bad_score_count_ > bad_score_count_threshold_ ||
             ndt_score_ > error_ndt_score_ ||
             (pos_type != drivers::gnss::SolutionType::INS_RTKFIXED &&
              pos_type != drivers::gnss::SolutionType::INS_RTKFLOAT)) {
    localization_status->set_fusion_status(MeasureState::ERROR);
    localization_status->set_state_message(
        "Error: Current Localization Is Very Unstable.");
  } else {
    localization_status->set_fusion_status(MeasureState::WARNNING);
    localization_status->set_state_message(
        "Warning: Current Localization Is Unstable.");
  }
}

bool NDTLocalization::QueryPoseFromBuffer(double time, Eigen::Affine3d* pose) {
  CHECK_NOTNULL(pose);

  TimeStampPose pre_pose;
  TimeStampPose next_pose;
  {
    std::lock_guard<std::mutex> lock(odometry_buffer_mutex_);
    TimeStampPose timestamp_pose = odometry_buffer_.back();
    // check abnormal timestamp
    if (time > timestamp_pose.timestamp) {
      AERROR << "query time is newer than latest odometry time, it doesn't "
                "make sense!";
      return false;
    }
    // search from reverse direction
    auto iter = odometry_buffer_.crbegin();
    for (; iter != odometry_buffer_.crend(); ++iter) {
      if (iter->timestamp < time) {
        break;
      }
    }
    if (iter == odometry_buffer_.crend()) {
      AINFO << "Cannot find matching pose from odometry buffer";
      return false;
    }
    pre_pose = *iter;
    next_pose = *(--iter);
  }
  // interpolation
  double v1 =
      (next_pose.timestamp - time) / (next_pose.timestamp - pre_pose.timestamp);
  double v2 =
      (time - pre_pose.timestamp) / (next_pose.timestamp - pre_pose.timestamp);
  pose->translation() =
      pre_pose.pose.translation() * v1 + next_pose.pose.translation() * v2;

  Eigen::Quaterniond pre_quat(pre_pose.pose.linear());

  common::math::EulerAnglesZXYd pre_euler(pre_quat.w(), pre_quat.x(),
                                          pre_quat.y(), pre_quat.z());

  Eigen::Quaterniond next_quat(next_pose.pose.linear());
  common::math::EulerAnglesZXYd next_euler(next_quat.w(), next_quat.x(),
                                           next_quat.y(), next_quat.z());

  double tmp_euler[3] = {};
  tmp_euler[0] = pre_euler.pitch() * v1 + next_euler.pitch() * v2;
  tmp_euler[1] = pre_euler.roll() * v1 + next_euler.roll() * v2;
  tmp_euler[2] = pre_euler.yaw() * v1 + next_euler.yaw() * v2;
  common::math::EulerAnglesZXYd euler(tmp_euler[1], tmp_euler[0], tmp_euler[2]);
  Eigen::Quaterniond quat = euler.ToQuaternion();
  quat.normalize();
  pose->linear() = quat.toRotationMatrix();
  return true;
}

bool NDTLocalization::ZeroOdometry(const Eigen::Affine3d& pose) {
  double x = pose.translation().x();
  double y = pose.translation().y();
  double z = pose.translation().z();
  double norm = std::sqrt(x * x + y * y + z * z);
  if (norm <= 0.01) {
    return true;
  }
  return false;
}

void NDTLocalization::LidarMsgTransfer(
    const std::shared_ptr<drivers::PointCloud>& msg, LidarFrame* lidar_frame) {
  CHECK_NOTNULL(lidar_frame);

  if (msg->height() > 1 && msg->width() > 1) {
    for (unsigned int i = 0; i < msg->height(); ++i) {
      for (unsigned int j = 0; j < msg->width(); ++j) {
        Eigen::Vector3f pt3d;
        pt3d[0] = msg->point(i * msg->width() + j).x();
        pt3d[1] = msg->point(i * msg->width() + j).y();
        pt3d[2] = msg->point(i * msg->width() + j).z();
        if (!std::isnan(pt3d[0])) {
          Eigen::Vector3f pt3d_tem = pt3d;

          if (pt3d_tem[2] > max_height_) {
            continue;
          }
          unsigned char intensity = static_cast<unsigned char>(
              msg->point(i * msg->width() + j).intensity());
          lidar_frame->pt_xs.push_back(pt3d[0]);
          lidar_frame->pt_ys.push_back(pt3d[1]);
          lidar_frame->pt_zs.push_back(pt3d[2]);
          lidar_frame->intensities.push_back(intensity);
        }
      }
    }
  } else {
    AINFO << "Receiving un-organized-point-cloud, width " << msg->width()
          << " height " << msg->height() << "size " << msg->point_size();
    for (int i = 0; i < msg->point_size(); ++i) {
      Eigen::Vector3f pt3d;
      pt3d[0] = msg->point(i).x();
      pt3d[1] = msg->point(i).y();
      pt3d[2] = msg->point(i).z();
      if (!std::isnan(pt3d[0])) {
        Eigen::Vector3f pt3d_tem = pt3d;

        if (pt3d_tem[2] > max_height_) {
          continue;
        }
        unsigned char intensity =
            static_cast<unsigned char>(msg->point(i).intensity());
        lidar_frame->pt_xs.push_back(pt3d[0]);
        lidar_frame->pt_ys.push_back(pt3d[1]);
        lidar_frame->pt_zs.push_back(pt3d[2]);
        lidar_frame->intensities.push_back(intensity);
      }
    }
  }

  lidar_frame->measurement_time =
      cyber::Time(msg->measurement_time()).ToSecond();
  if (ndt_debug_log_flag_) {
    AINFO << std::setprecision(15)
          << "NDTLocalization Debug Log: velodyne msg. "
          << "[time:" << lidar_frame->measurement_time
          << "][height:" << msg->height() << "][width:" << msg->width()
          << "][point_cnt:" << msg->point_size() << "]";
  }
}

bool NDTLocalization::LoadLidarExtrinsic(const std::string& file_path,
                                         Eigen::Affine3d* lidar_extrinsic) {
  CHECK_NOTNULL(lidar_extrinsic);

  YAML::Node config = YAML::LoadFile(file_path);
  if (config["transform"]) {
    if (config["transform"]["translation"]) {
      lidar_extrinsic->translation()(0) =
          config["transform"]["translation"]["x"].as<double>();
      lidar_extrinsic->translation()(1) =
          config["transform"]["translation"]["y"].as<double>();
      lidar_extrinsic->translation()(2) =
          config["transform"]["translation"]["z"].as<double>();
      if (config["transform"]["rotation"]) {
        double qx = config["transform"]["rotation"]["x"].as<double>();
        double qy = config["transform"]["rotation"]["y"].as<double>();
        double qz = config["transform"]["rotation"]["z"].as<double>();
        double qw = config["transform"]["rotation"]["w"].as<double>();
        lidar_extrinsic->linear() =
            Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
        return true;
      }
    }
  }
  return false;
}

bool NDTLocalization::LoadLidarHeight(const std::string& file_path,
                                      LidarHeight* height) {
  CHECK_NOTNULL(height);

  if (!cyber::common::PathExists(file_path)) {
    return false;
  }

  YAML::Node config = YAML::LoadFile(file_path);
  if (config["vehicle"]) {
    if (config["vehicle"]["parameters"]) {
      height->height = config["vehicle"]["parameters"]["height"].as<double>();
      height->height_var =
          config["vehicle"]["parameters"]["height_var"].as<double>();
      return true;
    }
  }
  return false;
}

bool NDTLocalization::LoadZoneIdFromFolder(const std::string& folder_path,
                                           int* zone_id) {
  std::string map_zone_id_folder;
  if (cyber::common::DirectoryExists(folder_path + "/map/000/north")) {
    map_zone_id_folder = folder_path + "/map/000/north";
  } else if (cyber::common::DirectoryExists(folder_path + "/map/000/south")) {
    map_zone_id_folder = folder_path + "/map/000/south";
  } else {
    return false;
  }

  auto folder_list = cyber::common::ListSubPaths(map_zone_id_folder);
  for (auto itr = folder_list.begin(); itr != folder_list.end(); ++itr) {
    *zone_id = std::stoi(*itr);
    return true;
  }
  return false;
}

bool NDTLocalization::FindNearestOdometryStatus(
    const double odometry_timestamp, drivers::gnss::InsStat* status) {
  CHECK_NOTNULL(status);

  std::unique_lock<std::mutex> lock(odometry_status_list_mutex_);
  auto odometry_status_list = odometry_status_list_;
  lock.unlock();

  double timestamp_diff_sec = 1e8;
  auto nearest_itr = odometry_status_list.end();
  for (auto itr = odometry_status_list.begin();
       itr != odometry_status_list.end(); ++itr) {
    double diff = std::abs(itr->header().timestamp_sec() - odometry_timestamp);
    if (diff < timestamp_diff_sec) {
      timestamp_diff_sec = diff;
      nearest_itr = itr;
    }
  }

  if (nearest_itr == odometry_status_list.end()) {
    return false;
  }

  if (timestamp_diff_sec > odometry_status_time_diff_threshold_) {
    return false;
  }

  *status = *nearest_itr;
  return true;
}

}  // namespace ndt
}  // namespace localization
}  // namespace apollo
