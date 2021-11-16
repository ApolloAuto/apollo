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

#include "modules/localization/msf/local_integ/localization_integ_process.h"

#include "yaml-cpp/yaml.h"

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/util/perf_util.h"
#include "modules/localization/msf/common/util/frame_transform.h"

namespace apollo {
namespace localization {
namespace msf {

using apollo::common::Status;
using apollo::cyber::Clock;

LocalizationIntegProcess::LocalizationIntegProcess()
    : sins_(new Sins()),
      gnss_antenna_extrinsic_(TransformD::Identity()),
      integ_state_(IntegState::NOT_INIT),
      ins_pva_(),
      pva_covariance_{0.0},
      corrected_imu_(),
      earth_param_(),
      keep_running_(false),
      measure_data_queue_size_(150),
      delay_output_counter_(0) {}

LocalizationIntegProcess::~LocalizationIntegProcess() {
  StopThreadLoop();

  delete sins_;
  sins_ = nullptr;
}

Status LocalizationIntegProcess::Init(const LocalizationIntegParam &param) {
  // sins init
  sins_->Init(param.is_ins_can_self_align);
  sins_->SetSinsAlignFromVel(param.is_sins_align_with_vel);

  sins_->SetResetSinsPoseStd(param.sins_state_pos_std);
  sins_->SetResetSinsMeasSpanTime(param.sins_state_span_time);

  if (param.is_using_raw_gnsspos) {
    gnss_antenna_extrinsic_.translation()(0) = param.imu_to_ant_offset.offset_x;
    gnss_antenna_extrinsic_.translation()(1) = param.imu_to_ant_offset.offset_y;
    gnss_antenna_extrinsic_.translation()(2) = param.imu_to_ant_offset.offset_z;
  } else {
    gnss_antenna_extrinsic_ = TransformD::Identity();
  }
  AINFO << "gnss and imu lever arm: "
        << gnss_antenna_extrinsic_.translation()(0) << " "
        << gnss_antenna_extrinsic_.translation()(1) << " "
        << gnss_antenna_extrinsic_.translation()(2);

  sins_->SetImuAntennaLeverArm(gnss_antenna_extrinsic_.translation()(0),
                               gnss_antenna_extrinsic_.translation()(1),
                               gnss_antenna_extrinsic_.translation()(2));

  sins_->SetVelThresholdGetYaw(param.vel_threshold_get_yaw);

  StartThreadLoop();

  return Status::OK();
}

void LocalizationIntegProcess::RawImuProcess(const ImuData &imu_msg) {
  integ_state_ = IntegState::NOT_INIT;
  double cur_imu_time = imu_msg.measurement_time;

  if (cur_imu_time < 3000) {
    AERROR << "the imu time is error: " << cur_imu_time;
    return;
  }

  static double pre_imu_time = cur_imu_time;
  double delta_time = cur_imu_time - pre_imu_time;
  if (delta_time > 0.1) {
    AERROR << std::setprecision(16) << "the imu message loss more than 10, "
           << "the pre time and current time: " << pre_imu_time << " "
           << cur_imu_time;
  } else if (delta_time < 0.0) {
    AERROR << std::setprecision(16)
           << "received imu message's time is eary than last imu message, "
           << "the pre time and current time: " << pre_imu_time << " "
           << cur_imu_time;
  }

  double cur_system_time = Clock::NowInSeconds();
  static double pre_system_time = cur_system_time;

  double delta_system_time = cur_system_time - pre_system_time;
  if (delta_system_time > 0.1) {
    AERROR << std::setprecision(16)
           << "the imu message loss more than 10 according to system time, "
           << "the pre system time and current system time: " << pre_system_time
           << " " << cur_system_time;
  } else if (delta_system_time < 0.0) {
    AERROR << std::setprecision(16)
           << "received imu message's time is eary than last imu message "
              "according to system time, "
           << "the pre system time and current system time: " << pre_system_time
           << " " << cur_system_time;
  }

  // add imu msg and get current predict pose
  sins_->AddImu(imu_msg);
  sins_->GetPose(&ins_pva_, pva_covariance_);
  sins_->GetRemoveBiasImu(&corrected_imu_);
  sins_->GetEarthParameter(&earth_param_);

  if (sins_->IsSinsAligned()) {
    integ_state_ = IntegState::NOT_STABLE;
    if (delay_output_counter_ < 3000) {
      ++delay_output_counter_;
    } else {
      integ_state_ = IntegState::OK;
      GetValidFromOK();
    }

    if (cur_imu_time - 0.5 > pre_imu_time) {
      AINFO << "SINS has completed alignment!" << std::endl;
      pre_imu_time = cur_imu_time;
    }
  } else {
    delay_output_counter_ = 0;
    if (cur_imu_time - 0.5 > pre_imu_time) {
      AINFO << "SINS is aligning!" << std::endl;
      pre_imu_time = cur_imu_time;
    }
  }

  pre_imu_time = cur_imu_time;
  pre_system_time = cur_system_time;
}

void LocalizationIntegProcess::GetValidFromOK() {
  if (integ_state_ != IntegState::OK) {
    return;
  }

  // AERROR << pva_covariance_[0][0] << " " << pva_covariance_[1][1]
  //     << " " << pva_covariance_[2][2] << " " << pva_covariance_[8][8];
  if (pva_covariance_[0][0] < 0.3 * 0.3 && pva_covariance_[1][1] < 0.3 * 0.3 &&
      pva_covariance_[2][2] < 0.3 * 0.3 && pva_covariance_[8][8] < 0.1 * 0.1) {
    integ_state_ = IntegState::VALID;
  }
}

void LocalizationIntegProcess::GetState(IntegState *state) {
  CHECK_NOTNULL(state);

  *state = integ_state_;
}

void LocalizationIntegProcess::GetResult(IntegState *state,
                                         LocalizationEstimate *localization) {
  CHECK_NOTNULL(state);
  CHECK_NOTNULL(localization);

  // state
  *state = integ_state_;

  if (*state != IntegState::NOT_INIT) {
    ADEBUG << std::setprecision(16)
           << "IntegratedLocalization Debug Log: integ_pose msg: "
           << "[time:" << ins_pva_.time << "]"
           << "[x:" << ins_pva_.pos.longitude * 57.295779513082323 << "]"
           << "[y:" << ins_pva_.pos.latitude * 57.295779513082323 << "]"
           << "[z:" << ins_pva_.pos.height << "]"
           << "[ve:" << ins_pva_.vel.ve << "]"
           << "[vn:" << ins_pva_.vel.vn << "]"
           << "[vu:" << ins_pva_.vel.vu << "]"
           << "[pitch: " << ins_pva_.att.pitch * 57.295779513082323 << "]"
           << "[roll:" << ins_pva_.att.roll * 57.295779513082323 << "]"
           << "[yaw:" << ins_pva_.att.yaw * 57.295779513082323 << "]";
  }

  // LocalizationEstimation
  apollo::common::Header *headerpb_loc = localization->mutable_header();
  apollo::localization::Pose *posepb_loc = localization->mutable_pose();

  localization->set_measurement_time(ins_pva_.time);
  headerpb_loc->set_timestamp_sec(apollo::cyber::Clock::NowInSeconds());

  apollo::common::PointENU *position_loc = posepb_loc->mutable_position();
  apollo::common::Quaternion *quaternion = posepb_loc->mutable_orientation();
  UTMCoor utm_xy;
  FrameTransform::LatlonToUtmXY(ins_pva_.pos.longitude, ins_pva_.pos.latitude,
                                &utm_xy);
  position_loc->set_x(utm_xy.x);
  position_loc->set_y(utm_xy.y);
  position_loc->set_z(ins_pva_.pos.height);

  quaternion->set_qx(ins_pva_.qbn[1]);
  quaternion->set_qy(ins_pva_.qbn[2]);
  quaternion->set_qz(ins_pva_.qbn[3]);
  quaternion->set_qw(ins_pva_.qbn[0]);

  apollo::common::Point3D *velocitylinear =
      posepb_loc->mutable_linear_velocity();
  velocitylinear->set_x(ins_pva_.vel.ve);
  velocitylinear->set_y(ins_pva_.vel.vn);
  velocitylinear->set_z(ins_pva_.vel.vu);

  apollo::common::Point3D *eulerangles = posepb_loc->mutable_euler_angles();
  eulerangles->set_x(ins_pva_.att.pitch);
  eulerangles->set_y(ins_pva_.att.roll);
  eulerangles->set_z(ins_pva_.att.yaw);

  posepb_loc->set_heading(ins_pva_.att.yaw);

  apollo::localization::Uncertainty *uncertainty =
      localization->mutable_uncertainty();
  apollo::common::Point3D *position_std_dev =
      uncertainty->mutable_position_std_dev();
  position_std_dev->set_x(std::sqrt(pva_covariance_[0][0]));
  position_std_dev->set_y(std::sqrt(pva_covariance_[1][1]));
  position_std_dev->set_z(std::sqrt(pva_covariance_[2][2]));

  apollo::common::Point3D *linear_velocity_std_dev =
      uncertainty->mutable_linear_velocity_std_dev();
  linear_velocity_std_dev->set_x(std::sqrt(pva_covariance_[3][3]));
  linear_velocity_std_dev->set_y(std::sqrt(pva_covariance_[4][4]));
  linear_velocity_std_dev->set_z(std::sqrt(pva_covariance_[5][5]));

  apollo::common::Point3D *orientation_std_dev =
      uncertainty->mutable_orientation_std_dev();
  orientation_std_dev->set_x(std::sqrt(pva_covariance_[6][6]));
  orientation_std_dev->set_y(std::sqrt(pva_covariance_[7][7]));
  orientation_std_dev->set_z(std::sqrt(pva_covariance_[8][8]));
}

void LocalizationIntegProcess::GetResult(IntegState *state, InsPva *sins_pva,
                                         double pva_covariance[9][9]) {
  CHECK_NOTNULL(state);
  CHECK_NOTNULL(sins_pva);
  CHECK_NOTNULL(pva_covariance);

  *state = integ_state_;
  *sins_pva = ins_pva_;
  memcpy(pva_covariance, pva_covariance_, sizeof(double) * 9 * 9);
}

void LocalizationIntegProcess::GetCorrectedImu(ImuData *imu_data) {
  CHECK_NOTNULL(imu_data);

  *imu_data = corrected_imu_;
}

void LocalizationIntegProcess::GetEarthParameter(
    InertialParameter *earth_param) {
  CHECK_NOTNULL(earth_param);

  *earth_param = earth_param_;
}

void LocalizationIntegProcess::MeasureDataProcess(
    const MeasureData &measure_msg) {
  measure_data_queue_mutex_.lock();
  measure_data_queue_.push(measure_msg);
  measure_data_queue_mutex_.unlock();
}

void LocalizationIntegProcess::StartThreadLoop() {
  keep_running_ = true;
  measure_data_queue_size_ = 150;
  cyber::Async(&LocalizationIntegProcess::MeasureDataThreadLoop, this);
}

void LocalizationIntegProcess::StopThreadLoop() {
  if (keep_running_.load()) {
    keep_running_ = false;
  }
}

void LocalizationIntegProcess::MeasureDataThreadLoop() {
  AINFO << "Started measure data process thread";
  while (keep_running_.load()) {
    {
      std::unique_lock<std::mutex> lock(measure_data_queue_mutex_);
      int size = static_cast<int>(measure_data_queue_.size());
      while (size > measure_data_queue_size_) {
        measure_data_queue_.pop();
        --size;
      }
      if (measure_data_queue_.empty()) {
        lock.unlock();
        cyber::Yield();
        continue;
      }
    }

    MeasureData measure;
    int waiting_num = 0;
    {
      std::unique_lock<std::mutex> lock(measure_data_queue_mutex_);
      measure = measure_data_queue_.front();
      measure_data_queue_.pop();
      waiting_num = static_cast<int>(measure_data_queue_.size());
    }

    if (waiting_num > measure_data_queue_size_ / 4) {
      AWARN << waiting_num << " measure are waiting to process.";
    }

    MeasureDataProcessImpl(measure);
  }
  AINFO << "Exited measure data process thread";
}

void LocalizationIntegProcess::MeasureDataProcessImpl(
    const MeasureData &measure_msg) {
  common::util::Timer timer;
  timer.Start();

  if (!CheckIntegMeasureData(measure_msg)) {
    return;
  }

  sins_->AddMeasurement(measure_msg);

  timer.End("time of integrated navigation measure update");
}

bool LocalizationIntegProcess::CheckIntegMeasureData(
    const MeasureData &measure_data) {
  if (measure_data.measure_type == MeasureType::ODOMETER_VEL_ONLY) {
    AERROR << "receive a new odometry measurement!!!\n";
  }

  ADEBUG << std::setprecision(16)
         << "IntegratedLocalization Debug Log: measure data: "
         << "[time:" << measure_data.time << "]"
         << "[x:" << measure_data.gnss_pos.longitude * 57.295779513082323 << "]"
         << "[y:" << measure_data.gnss_pos.latitude * 57.295779513082323 << "]"
         << "[z:" << measure_data.gnss_pos.height << "]"
         << "[ve:" << measure_data.gnss_vel.ve << "]"
         << "[vn:" << measure_data.gnss_vel.vn << "]"
         << "[vu:" << measure_data.gnss_vel.vu << "]"
         << "[pitch:" << measure_data.gnss_att.pitch * 57.295779513082323 << "]"
         << "[roll:" << measure_data.gnss_att.roll * 57.295779513082323 << "]"
         << "[yaw:" << measure_data.gnss_att.yaw * 57.295779513082323 << "]"
         << "[measure type:" << int(measure_data.measure_type) << "]";

  return true;
}

bool LocalizationIntegProcess::LoadGnssAntennaExtrinsic(
    const std::string &file_path, TransformD *extrinsic) const {
  CHECK_NOTNULL(extrinsic);

  YAML::Node confige = YAML::LoadFile(file_path);
  if (confige["leverarm"]) {
    if (confige["leverarm"]["primary"]["offset"]) {
      extrinsic->translation()(0) =
          confige["leverarm"]["primary"]["offset"]["x"].as<double>();
      extrinsic->translation()(1) =
          confige["leverarm"]["primary"]["offset"]["y"].as<double>();
      extrinsic->translation()(2) =
          confige["leverarm"]["primary"]["offset"]["z"].as<double>();
      return true;
    }
  }

  return false;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
