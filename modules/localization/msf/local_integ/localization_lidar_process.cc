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

#include "modules/localization/msf/local_integ/localization_lidar_process.h"

#include "yaml-cpp/yaml.h"

#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/time/timer.h"
#include "modules/common/util/file.h"

namespace apollo {
namespace localization {
namespace msf {

using apollo::common::Status;
using apollo::common::time::Timer;

LocalizationLidarProcess::LocalizationLidarProcess()
    : locator_(new LocalizationLidar()),
      pose_forcastor_(new PoseForcast()),
      map_path_(""),
      lidar_extrinsic_file_(""),
      lidar_height_file_(""),
      debug_log_flag_(true),
      localization_mode_(2),
      yaw_align_mode_(2),
      lidar_filter_size_(17),
      delta_yaw_limit_(0.25 * 3.14159 / 180.0),
      init_delta_yaw_limit_(1.5 * 3.14159 / 180.0),
      compensate_pitch_roll_limit_(0.035),
      utm_zone_id_(50),
      map_coverage_theshold_(0.8),
      lidar_extrinsic_(TransformD::Identity()),
      lidar_height_(),
      is_get_first_lidar_msg_(false),
      cur_predict_location_(TransformD::Identity()),
      pre_predict_location_(TransformD::Identity()),
      velocity_(Vector3D::Zero()),
      pre_location_(TransformD::Identity()),
      location_(TransformD::Identity()),
      pre_location_time_(0.0),
      location_covariance_(Matrix3D::Zero()),
      lidar_status_(LidarState::NOT_VALID),
      reinit_flag_(false),
      imu_lidar_max_delay_time_(0.5),
      is_unstable_reset_(true),
      unstable_count_(0),
      unstable_threshold_(0.08),
      out_map_count_(0),
      forcast_integ_state_(ForcastState::NOT_VALID),
      forcast_timer_(-1) {}

LocalizationLidarProcess::~LocalizationLidarProcess() {
  delete locator_;
  locator_ = nullptr;

  delete pose_forcastor_;
  pose_forcastor_ = nullptr;
}

Status LocalizationLidarProcess::Init(const LocalizationIntegParam& params) {
  // initial_success_ = false;
  map_path_ = params.map_path;
  lidar_extrinsic_file_ = params.lidar_extrinsic_file;
  lidar_height_file_ = params.lidar_height_file;
  debug_log_flag_ = params.lidar_debug_log_flag;
  localization_mode_ = params.localization_mode;
  yaw_align_mode_ = params.lidar_yaw_align_mode;
  utm_zone_id_ = params.utm_zone_id;
  map_coverage_theshold_ = params.map_coverage_theshold;
  imu_lidar_max_delay_time_ = params.imu_lidar_max_delay_time;

  lidar_filter_size_ = params.lidar_filter_size;

  is_unstable_reset_ = params.is_lidar_unstable_reset;
  unstable_threshold_ = params.unstable_reset_threshold;

  lidar_status_ = LidarState::NOT_VALID;

  // reload_map_flag_ = false;
  reinit_flag_ = false;

  // buffer
  out_map_count_ = 0;

  is_get_first_lidar_msg_ = false;
  cur_predict_location_ = TransformD::Identity();
  pre_predict_location_ = TransformD::Identity();
  pre_location_ = TransformD::Identity();
  velocity_ = Vector3D::Zero();
  location_ = TransformD::Identity();
  location_covariance_ = Matrix3D::Zero();

  bool sucess = LoadLidarExtrinsic(lidar_extrinsic_file_, &lidar_extrinsic_);
  if (!sucess) {
    AERROR << "LocalizationLidar: Fail to access the lidar"
              " extrinsic file: "
           << lidar_extrinsic_file_;
    return Status(common::LOCALIZATION_ERROR_LIDAR,
                  "Fail to access the lidar extrinsic file");
  }

  sucess = LoadLidarHeight(lidar_height_file_, &lidar_height_);
  if (!sucess) {
    AWARN << "LocalizationLidar: Fail to load the lidar"
             " height file: "
          << lidar_height_file_ << " Will use default value!";
    lidar_height_.height = params.lidar_height_default;
  }

  if (!locator_->Init(map_path_, lidar_filter_size_, lidar_filter_size_,
                      utm_zone_id_)) {
    return Status(common::LOCALIZATION_ERROR_LIDAR,
                  "Fail to load localization map!");
  }

  locator_->SetVelodyneExtrinsic(lidar_extrinsic_);
  locator_->SetLocalizationMode(localization_mode_);
  locator_->SetImageAlignMode(yaw_align_mode_);
  locator_->SetValidThreshold(map_coverage_theshold_);
  locator_->SetVehicleHeight(lidar_height_.height);
  locator_->SetDeltaPitchRollLimit(compensate_pitch_roll_limit_);

  const double deg_to_rad = 0.017453292519943;
  const double max_gyro_input = 200 * deg_to_rad;  // 200 degree
  const double max_acc_input = 5.0;                // 5.0 m/s^2
  pose_forcastor_->SetMaxListNum(400);
  pose_forcastor_->SetMaxAccelInput(max_acc_input);
  pose_forcastor_->SetMaxGyroInput(max_gyro_input);
  pose_forcastor_->SetZoneId(utm_zone_id_);

  return Status::OK();
}

double LocalizationLidarProcess::ComputeDeltaYawLimit(
    const int64_t index_cur, const int64_t index_stable, const double limit_min,
    const double limit_max) {
  if (index_cur > index_stable) {
    return limit_min;
  }

  double ratio = static_cast<double>(index_cur);
  ratio /= static_cast<double>(index_stable);
  return limit_min * ratio + limit_max * (1.0 - ratio);
}

void LocalizationLidarProcess::PcdProcess(const LidarFrame& lidar_frame) {
  if (!CheckState()) {
    AERROR << "PcdProcess: Receive an invalid lidar msg!";
    return;
  }

  // pcd process cost time
  Timer timer;
  timer.Start();

  static unsigned int pcd_index = 0;

  if (!GetPredictPose(lidar_frame.measurement_time, &cur_predict_location_,
                      &forcast_integ_state_)) {
    AINFO << "PcdProcess: Discard a lidar msg because can't get predict pose. "
          << "More info see log in function GetPredictPose.";
    return;
  }

  if (forcast_integ_state_ != ForcastState::INCREMENT) {
    forcast_timer_ = -1;
  }
  ++forcast_timer_;

  locator_->SetDeltaYawLimit(ComputeDeltaYawLimit(
      forcast_timer_, 10, delta_yaw_limit_, init_delta_yaw_limit_));

  if (!is_get_first_lidar_msg_) {
    pre_predict_location_ = cur_predict_location_;
    pre_location_ = cur_predict_location_;
    velocity_ = Vector3D::Zero();
    is_get_first_lidar_msg_ = true;
  }

  velocity_ = cur_predict_location_.translation() - pre_location_.translation();

  int ret = locator_->Update(pcd_index++, cur_predict_location_, velocity_,
                             lidar_frame);

  UpdateState(ret, lidar_frame.measurement_time);

  timer.End("Lidar process");
}

void LocalizationLidarProcess::GetResult(int* lidar_status,
                                         TransformD* location,
                                         Matrix3D* covariance) const {
  CHECK_NOTNULL(lidar_status);
  CHECK_NOTNULL(location);
  CHECK_NOTNULL(covariance);

  *lidar_status = static_cast<int>(lidar_status_);
  *location = location_;
  *covariance = location_covariance_;
  return;
}

int LocalizationLidarProcess::GetResult(LocalizationEstimate* lidar_local_msg) {
  if (lidar_local_msg == nullptr) {
    return static_cast<int>(LidarState::NOT_VALID);
  }

  lidar_local_msg->set_measurement_time(pre_location_time_);

  apollo::common::Header* headerpb = lidar_local_msg->mutable_header();
  headerpb->set_timestamp_sec(pre_location_time_);

  apollo::localization::Pose* posepb = lidar_local_msg->mutable_pose();
  apollo::common::PointENU* position = posepb->mutable_position();
  position->set_x(location_.translation()(0));
  position->set_y(location_.translation()(1));
  position->set_z(location_.translation()(2));

  apollo::common::Quaternion* quaternion = posepb->mutable_orientation();
  Eigen::Quaterniond quat(location_.linear());
  quaternion->set_qx(quat.x());
  quaternion->set_qy(quat.y());
  quaternion->set_qz(quat.z());
  quaternion->set_qw(quat.w());

  apollo::localization::Uncertainty* uncertainty =
      lidar_local_msg->mutable_uncertainty();

  apollo::common::Point3D* position_std_dev =
      uncertainty->mutable_position_std_dev();
  position_std_dev->set_x(location_covariance_(0, 0));
  position_std_dev->set_y(location_covariance_(1, 1));
  position_std_dev->set_z(0.0);

  constexpr double yaw_covariance = 0.15 * 0.15 * DEG_TO_RAD2;
  apollo::common::Point3D* orientation_std_dev =
      uncertainty->mutable_orientation_std_dev();
  orientation_std_dev->set_x(0.0);
  orientation_std_dev->set_y(0.0);
  orientation_std_dev->set_z(yaw_covariance);

  return static_cast<int>(lidar_status_);
}

void LocalizationLidarProcess::IntegPvaProcess(const InsPva& sins_pva_msg) {
  pose_forcastor_->PushInspvaData(sins_pva_msg);
  return;
}

void LocalizationLidarProcess::RawImuProcess(const ImuData& imu_msg) {
  pose_forcastor_->PushImuData(imu_msg);
  return;
}

bool LocalizationLidarProcess::GetPredictPose(const double lidar_time,
                                              TransformD* predict_pose,
                                              ForcastState* forcast_state) {
  CHECK_NOTNULL(predict_pose);
  CHECK_NOTNULL(forcast_state);

  double latest_imu_time = pose_forcastor_->GetLastestImuTime();
  if (latest_imu_time - lidar_time > imu_lidar_max_delay_time_) {
    AERROR << std::setprecision(16) << "LocalizationLidar GetPredictPose: "
           << "Lidar msg too old! "
           << "lidar time: " << lidar_time
           << "delay time: " << latest_imu_time - lidar_time;
    return false;
  }

  Pose forcast_pose;
  int state = -1;
  if (lidar_status_ != LidarState::OK) {
    Pose init_pose;
    state = pose_forcastor_->GetBestForcastPose(lidar_time, -1, init_pose,
                                                &forcast_pose);
  } else {
    Pose init_pose;
    init_pose.x = pre_location_.translation()(0);
    init_pose.y = pre_location_.translation()(1);
    init_pose.z = pre_location_.translation()(2);
    Eigen::Quaterniond quatd(pre_location_.linear());
    init_pose.qx = quatd.x();
    init_pose.qy = quatd.y();
    init_pose.qz = quatd.z();
    init_pose.qw = quatd.w();

    state = pose_forcastor_->GetBestForcastPose(lidar_time, pre_location_time_,
                                                init_pose, &forcast_pose);
  }

  if (state < 0) {
    AINFO << "LocalizationLidar GetPredictPose: "
          << "Recive a lidar msg, but can't query predict pose.";
    *forcast_state = ForcastState::NOT_VALID;
    return false;
  }

  if (std::abs(forcast_pose.x) < 10.0 || std::abs(forcast_pose.y) < 10.0) {
    AERROR << "LocalizationLidar Fatal Error: invalid pose!";
    return false;
  }

  Eigen::Quaterniond quatd(forcast_pose.qw, forcast_pose.qx, forcast_pose.qy,
                           forcast_pose.qz);
  Eigen::Translation3d transd(
      Eigen::Vector3d(forcast_pose.x, forcast_pose.y, forcast_pose.z));
  *predict_pose = transd * quatd;

  if (state == 0) {
    *forcast_state = ForcastState::INITIAL;
  } else {
    *forcast_state = ForcastState::INCREMENT;
    AINFO << "The delta translation input lidar localization: " << lidar_time
          << " " << forcast_pose.x - pre_location_.translation()(0) << " "
          << forcast_pose.y - pre_location_.translation()(1) << " "
          << forcast_pose.z - pre_location_.translation()(2);
  }

  return true;
}

bool LocalizationLidarProcess::CheckState() { return true; }

void LocalizationLidarProcess::UpdateState(const int ret, const double time) {
  if (ret == 0) {  // OK
    locator_->GetResult(&location_, &location_covariance_);
    lidar_status_ = LidarState::OK;

    // check covariance
    double cur_location_std_area = std::sqrt(location_covariance_(0, 0)) *
                                   std::sqrt(location_covariance_(1, 1));
    if (cur_location_std_area > unstable_threshold_) {
      ++unstable_count_;
    } else {
      unstable_count_ = 0;
    }

    // check if lidar need reset
    if (unstable_count_ >= 2 && is_unstable_reset_) {
      unstable_count_ = 2;
      reinit_flag_ = true;
      AWARN << "Reinit lidar localization due to big covariance";
      lidar_status_ = LidarState::NOT_STABLE;
    } else {
      lidar_status_ = LidarState::OK;
      if (out_map_count_ > 0) {
        --out_map_count_;
      }
    }
    pre_location_ = location_;
    pre_location_time_ = time;

  } else if (ret == -2) {  // out of map
    locator_->GetResult(&location_, &location_covariance_);
    lidar_status_ = LidarState::NOT_STABLE;
    pre_location_ = location_;
    pre_location_time_ = time;
    if (out_map_count_ < 10) {
      ++out_map_count_;
    } else {
      reinit_flag_ = true;
    }
  } else {  // NOT_VALID
    AERROR << "LocalizationLidar: The reflection map load failed!";
    lidar_status_ = LidarState::NOT_VALID;
  }
}

bool LocalizationLidarProcess::LoadLidarExtrinsic(const std::string& file_path,
                                                  TransformD* lidar_extrinsic) {
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

bool LocalizationLidarProcess::LoadLidarHeight(const std::string& file_path,
                                               LidarHeight* height) {
  CHECK_NOTNULL(height);

  if (!common::util::PathExists(file_path)) {
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

}  // namespace msf
}  // namespace localization
}  // namespace apollo
