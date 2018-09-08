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

#include "modules/localization/msf/local_integ/localization_integ_impl.h"

#include <list>
#include <queue>

#include "modules/common/log.h"
#include "modules/common/time/timer.h"
#include "modules/common/time/time.h"
#include "modules/localization/msf/common/util/frame_transform.h"

namespace apollo {
namespace localization {
namespace msf {

using common::Status;

LocalizationIntegImpl::LocalizationIntegImpl()
    : republish_process_(new MeasureRepublishProcess()),
      integ_process_(new LocalizationIntegProcess()),
      gnss_process_(new LocalizationGnssProcess()),
      lidar_process_(new LocalizationLidarProcess()),
      lidar_localization_list_max_size_(10),
      integ_localization_list_max_size_(50),
      gnss_localization_list_max_size_(10),
      is_use_gnss_bestpose_(true),
      keep_lidar_running_(false),
      lidar_queue_max_size_(5),
      imu_altitude_from_lidar_localization_(0.0),
      imu_altitude_from_lidar_localization_available_(false),
      keep_imu_running_(false),
      imu_queue_max_size_(200),
      keep_gnss_running_(false),
      gnss_queue_max_size_(100),
      enable_lidar_localization_(true),
      gnss_antenna_extrinsic_(Eigen::Affine3d::Identity()) {}

LocalizationIntegImpl::~LocalizationIntegImpl() {
  StopThreadLoop();

  delete republish_process_;
  delete lidar_process_;
  delete gnss_process_;
  delete integ_process_;
}

Status LocalizationIntegImpl::Init(const LocalizationIntegParam& params) {
  enable_lidar_localization_ = params.enable_lidar_localization;
  if (params.enable_lidar_localization == true) {
    auto state = lidar_process_->Init(params);
    if (!state.ok()) {
      return state;
    }
  }

  auto state = republish_process_->Init(params);
  if (!state.ok()) {
    return state;
  }

  state = integ_process_->Init(params);
  if (!state.ok()) {
    return state;
  }

  if (params.gnss_mode == static_cast<int>(GnssMode::SELF)) {
    state = gnss_process_->Init(params);
    is_use_gnss_bestpose_ = false;
    if (!state.ok()) {
      return state;
    }
  } else {
    is_use_gnss_bestpose_ = true;
  }

  if (params.is_using_raw_gnsspos) {
    gnss_antenna_extrinsic_.translation()(0) =
        params.imu_to_ant_offset.offset_x;
    gnss_antenna_extrinsic_.translation()(1) =
        params.imu_to_ant_offset.offset_y;
    gnss_antenna_extrinsic_.translation()(2) =
        params.imu_to_ant_offset.offset_z;
  } else {
    gnss_antenna_extrinsic_ = Eigen::Affine3d::Identity();
  }
  AINFO << "gnss and imu lever arm: "
        << gnss_antenna_extrinsic_.translation()(0) << " "
        << gnss_antenna_extrinsic_.translation()(1) << " "
        << gnss_antenna_extrinsic_.translation()(2);
  msf_status_.set_local_lidar_consistency(
              apollo::localization::MSF_LOCAL_LIDAR_CONSISTENCY_03);
  msf_status_.set_gnss_consistency(
              apollo::localization::MSF_GNSS_CONSISTENCY_03);
  msf_status_.set_local_lidar_status(
              apollo::localization::MSF_LOCAL_LIDAR_07);
  msf_status_.set_gnsspos_position_type(93);
  msf_status_.set_msf_running_status(
              apollo::localization::MSF_RUNNING_INIT);
  msf_status_.set_local_lidar_quality(
              apollo::localization::MSF_LOCAL_LIDAR_BAD);

  StartThreadLoop();

  return Status::OK();
}

void LocalizationIntegImpl::StartThreadLoop() {
  // run process thread
  keep_lidar_running_ = true;
  lidar_queue_max_size_ = 5;
  const auto& pcd_loop_func = [this] { PcdThreadLoop(); };
  lidar_data_thread_ = std::thread(pcd_loop_func);

  keep_imu_running_ = true;
  imu_queue_max_size_ = 200;
  const auto& imu_loop_func = [this] { ImuThreadLoop(); };
  imu_data_thread_ = std::thread(imu_loop_func);

  keep_gnss_running_ = true;
  gnss_queue_max_size_ = 100;
  const auto gnss_loop_func = [this] { GnssThreadLoop(); };
  gnss_function_thread_ = std::thread(gnss_loop_func);
}

void LocalizationIntegImpl::StopThreadLoop() {
  if (keep_lidar_running_.load()) {
    keep_lidar_running_ = false;
    lidar_data_signal_.notify_one();
    lidar_data_thread_.join();
  }

  if (keep_imu_running_.load()) {
    keep_imu_running_ = false;
    imu_data_signal_.notify_one();
    imu_data_thread_.join();
  }

  if (keep_gnss_running_.load()) {
    keep_gnss_running_ = false;
    gnss_function_signal_.notify_one();
    gnss_function_thread_.join();
  }
}

void LocalizationIntegImpl::PcdProcess(const LidarFrame& lidar_frame) {
  lidar_data_queue_mutex_.lock();
  lidar_data_queue_.push(lidar_frame);
  lidar_data_signal_.notify_one();
  lidar_data_queue_mutex_.unlock();
  return;
}

void LocalizationIntegImpl::PcdThreadLoop() {
  AINFO << "Started pcd data process thread";
  while (keep_lidar_running_.load()) {
    {
      std::unique_lock<std::mutex> lock(lidar_data_queue_mutex_);
      size_t size = lidar_data_queue_.size();
      while (size > lidar_queue_max_size_) {
        lidar_data_queue_.pop();
        --size;
      }
      if (lidar_data_queue_.size() == 0) {
        lidar_data_signal_.wait(lock);
        continue;
      }
    }

    LidarFrame lidar_frame;
    int waiting_num = 0;
    {
      std::unique_lock<std::mutex> lock(lidar_data_queue_mutex_);
      lidar_frame = lidar_data_queue_.front();
      lidar_data_queue_.pop();
      waiting_num = lidar_data_queue_.size();
    }

    if (waiting_num > 2) {
      AWARN << waiting_num << " point cloud msg are waiting to process.";
    }

    PcdProcessImpl(lidar_frame);
  }
  AINFO << "Exited pcd data process thread";
}

void LocalizationIntegImpl::PcdProcessImpl(const LidarFrame& pcd_data) {
  // lidar -> republish -> integ
  lidar_process_->PcdProcess(pcd_data);

  int state = 0;
  LocalizationEstimate lidar_localization;

  state = lidar_process_->GetResult(&lidar_localization);

  msf_status_mutex_.lock();
  msf_status_.set_local_lidar_status(
    lidar_localization.msf_status().local_lidar_status());
  msf_status_.set_local_lidar_quality(
    lidar_localization.msf_status().local_lidar_quality());
  msf_status_mutex_.unlock();

  MeasureData lidar_measure;
  if (state == 2) {  // only state OK republish lidar msg
    republish_process_->LidarLocalProcess(lidar_localization, &lidar_measure);
    integ_process_->MeasureDataProcess(lidar_measure);
    latest_lidar_timestamp_ = lidar_measure.time;

    imu_altitude_from_lidar_localization_ =
        lidar_localization.pose().position().z();
    imu_altitude_from_lidar_localization_available_ = true;
  }

  lidar_localization_mutex_.lock();
  lidar_localization_list_.push_back(
      LocalizationResult(LocalizationMeasureState(state), lidar_localization));
  if (lidar_localization_list_.size() > lidar_localization_list_max_size_) {
    lidar_localization_list_.pop_front();
  }
  lidar_localization_mutex_.unlock();
}

void LocalizationIntegImpl::RawImuProcessRfu(const ImuData& imu_data) {
  // push to imu_data_queue
  imu_data_queue_mutex_.lock();
  imu_data_queue_.push(imu_data);
  imu_data_signal_.notify_one();
  imu_data_queue_mutex_.unlock();
}

void LocalizationIntegImpl::ImuThreadLoop() {
  AINFO << "Started imu data process thread";
  while (keep_imu_running_.load()) {
    {
      std::unique_lock<std::mutex> lock(imu_data_queue_mutex_);
      size_t size = imu_data_queue_.size();
      while (size > imu_queue_max_size_) {
        imu_data_queue_.pop();
        --size;
      }
      if (imu_data_queue_.size() == 0) {
        imu_data_signal_.wait(lock);
        continue;
      }
    }

    ImuData imu_data;
    int waiting_num = 0;
    {
      std::unique_lock<std::mutex> lock(imu_data_queue_mutex_);
      imu_data = imu_data_queue_.front();
      imu_data_queue_.pop();
      waiting_num = imu_data_queue_.size();
    }

    if (waiting_num > 10) {
      AWARN << waiting_num << " imu msg are waiting to process.";
    }

    ImuProcessImpl(imu_data);
  }
  AINFO << "Exited imu data process thread";
}

void LocalizationIntegImpl::ImuProcessImpl(const ImuData& imu_data) {
  // imu -> lidar
  // imu -> integ -> republish -> lidar -> publish

  if (enable_lidar_localization_) {
    lidar_process_->RawImuProcess(imu_data);
  }
  integ_process_->RawImuProcess(imu_data);

  double cur_imu_time = imu_data.measurement_time;
  sensor_status_.set_imu_delay_status(apollo::localization::IMU_DELAY_NORMAL);
  double cur_system_time = apollo::common::time::Clock::NowInSeconds();
  double delta_system_time = cur_system_time - cur_imu_time;
  if (delta_system_time > 0.1) {
    AINFO << std::setprecision(16)
          << "the imu delays " << delta_system_time
          << ", the current system time is "
          << cur_system_time
          << ", the imu timestamp is " << cur_imu_time;
    sensor_status_.set_imu_delay_status(apollo::localization::IMU_DELAY_3);
  } else if (delta_system_time > 0.05) {
    AINFO << std::setprecision(16)
          << "the imu delays " << delta_system_time
          << ", the current system time is "
          << cur_system_time
          << ", the imu timestamp is " << cur_imu_time;
    sensor_status_.set_imu_delay_status(apollo::localization::IMU_DELAY_2);
  } else if (delta_system_time > 0.02) {
    AINFO << std::setprecision(16)
          << "the imu delays " << delta_system_time
          << ", the current system time is "
          << cur_system_time
          << ", the imu timestamp is " << cur_imu_time;
    sensor_status_.set_imu_delay_status(apollo::localization::IMU_DELAY_1);
  }
  sensor_status_.set_imu_missing_status(
                 ImuMsgMissingStatus::IMU_MISSING_NORMAL);
  static double pre_imu_time = cur_imu_time;
  double delta_time = cur_imu_time - pre_imu_time;
  if (delta_time > 0.1) {
    ADEBUG << std::setprecision(16)
           << "The imu message loss more than 100ms, "
           << "the pre time and current time: "
           << pre_imu_time << " "
           << cur_imu_time;
    sensor_status_.set_imu_missing_status(
                   ImuMsgMissingStatus::IMU_MISSING_3);
  } else if (delta_time > 0.05) {
    ADEBUG << std::setprecision(16)
           << "The imu message loss more than 50ms, "
           << "the pre time and current time: "
           << pre_imu_time << " "
           << cur_imu_time;
    sensor_status_.set_imu_missing_status(
                   ImuMsgMissingStatus::IMU_MISSING_2);
  } else if (delta_time > 0.01) {
    ADEBUG << std::setprecision(16)
           << "The imu message loss more than 10ms, "
           << "the pre time and current time: "
           << pre_imu_time << " "
           << cur_imu_time;
    sensor_status_.set_imu_missing_status(
                   ImuMsgMissingStatus::IMU_MISSING_1);
  } else if (delta_time < 0.0) {
    ADEBUG << std::setprecision(16)
           << "The imu message loss more than 10ms, "
           << "the pre time and current time: "
           << pre_imu_time << " "
           << cur_imu_time;
    sensor_status_.set_imu_missing_status(
                   ImuMsgMissingStatus::IMU_MISSING_ABNORMAL);
  }
  pre_imu_time = cur_imu_time;
  msf_status_mutex_.lock();
  if (cur_imu_time - latest_gnsspos_timestamp_ > 2.0) {
    AINFO << std::setprecision(15)
          << "The bestgnsspose msg loses more than 2 seconds: "
          << "imu time and latest gnss time: "
          << cur_imu_time << " "
          << latest_gnsspos_timestamp_;
    msf_status_.set_gnsspos_position_type(91);
  }
  if (cur_imu_time - latest_lidar_timestamp_ > 2.0) {
    AINFO << std::setprecision(15)
          << "The local lidar msg loses more than 2 seconds: "
          << "imu time and latest local lidar time: "
          << cur_imu_time << " "
          << latest_lidar_timestamp_;
    msf_status_.set_local_lidar_status(LocalLidarStatus::MSF_LOCAL_LIDAR_04);
  }
  msf_status_mutex_.unlock();
  // integ
  IntegState state;
  LocalizationEstimate integ_localization;
  integ_process_->GetResult(&state, &integ_localization);

  apollo::common::Point3D position_std =
    integ_localization.uncertainty().position_std_dev();
    msf_status_mutex_.lock();
    if (position_std.x() < 0.15 && position_std.y() < 0.15) {
      if (msf_status_.gnsspos_position_type() == 50 ||
          msf_status_.gnsspos_position_type() == 49 ||
          msf_status_.gnsspos_position_type() == 48) {
          if (msf_status_.local_lidar_status()
              == apollo::localization::MSF_LOCAL_LIDAR_00) {
              msf_status_.set_msf_running_status(
                apollo::localization::MSF_SOL_LIDAR_GNSS);
          } else {
              msf_status_.set_msf_running_status(
                apollo::localization::MSF_SOL_X_GNSS);
          }
      } else if (msf_status_.gnsspos_position_type() == 4 ||
                 msf_status_.gnsspos_position_type() == 5 ||
                 msf_status_.gnsspos_position_type() == 6 ||
                 msf_status_.gnsspos_position_type() == 32 ||
                 msf_status_.gnsspos_position_type() == 33 ||
                 msf_status_.gnsspos_position_type() == 34) {
          if (msf_status_.local_lidar_status()
                  == apollo::localization::MSF_LOCAL_LIDAR_00) {
              msf_status_.set_msf_running_status(
                apollo::localization::MSF_SOL_LIDAR_X);
          } else {
              msf_status_.set_msf_running_status(
                apollo::localization::MSF_SOL_X_X);
          }
      } else if (msf_status_.gnsspos_position_type() == 16 ||
                 msf_status_.gnsspos_position_type() == 17) {
          if (msf_status_.local_lidar_status()
                == apollo::localization::MSF_LOCAL_LIDAR_00) {
              msf_status_.set_msf_running_status(
                apollo::localization::MSF_SOL_LIDAR_XX);
          } else {
              msf_status_.set_msf_running_status(
                apollo::localization::MSF_SOL_X_XX);
          }
      } else {
          if (msf_status_.local_lidar_status()
                  == apollo::localization::MSF_LOCAL_LIDAR_00) {
              msf_status_.set_msf_running_status(
                apollo::localization::MSF_SOL_LIDAR_XXX);
          } else {
              msf_status_.set_msf_running_status(
                apollo::localization::MSF_SOL_X_XXX);
          }
      }
  } else if (position_std.x() < 0.3 * 0.3 && position_std.y() < 0.3 * 0.3) {
      if (msf_status_.gnsspos_position_type() == 50 ||
          msf_status_.gnsspos_position_type() == 49 ||
          msf_status_.gnsspos_position_type() == 48) {
          if (msf_status_.local_lidar_status()
                == apollo::localization::MSF_LOCAL_LIDAR_00) {
              msf_status_.set_msf_running_status(
                apollo::localization::MSF_SSOL_LIDAR_GNSS);
          } else {
              msf_status_.set_msf_running_status(
                apollo::localization::MSF_SSOL_X_GNSS);
          }
      } else if (msf_status_.gnsspos_position_type() == 4 ||
                 msf_status_.gnsspos_position_type() == 5 ||
                 msf_status_.gnsspos_position_type() == 6 ||
                 msf_status_.gnsspos_position_type() == 32 ||
                 msf_status_.gnsspos_position_type() == 33 ||
                 msf_status_.gnsspos_position_type() == 34) {
          if (msf_status_.local_lidar_status()
                == apollo::localization::MSF_LOCAL_LIDAR_00) {
              msf_status_.set_msf_running_status(
                apollo::localization::MSF_SSOL_LIDAR_X);
          } else {
              msf_status_.set_msf_running_status(
                apollo::localization::MSF_SSOL_X_X);
          }
      } else if (msf_status_.gnsspos_position_type() == 16 ||
                 msf_status_.gnsspos_position_type() == 17) {
          if (msf_status_.local_lidar_status()
                == apollo::localization::MSF_LOCAL_LIDAR_00) {
              msf_status_.set_msf_running_status(
                apollo::localization::MSF_SSOL_LIDAR_XX);
          } else {
              msf_status_.set_msf_running_status(
                apollo::localization::MSF_SSOL_X_XX);
          }
      } else {
          if (msf_status_.local_lidar_status()
                == apollo::localization::MSF_LOCAL_LIDAR_00) {
              msf_status_.set_msf_running_status(
                apollo::localization::MSF_SSOL_LIDAR_XXX);
          } else {
              msf_status_.set_msf_running_status(
                apollo::localization::MSF_SSOL_X_XXX);
          }
      }
  } else {
      if (msf_status_.gnsspos_position_type() == 50 ||
          msf_status_.gnsspos_position_type() == 49 ||
          msf_status_.gnsspos_position_type() == 48) {
          if (msf_status_.local_lidar_status()
                == apollo::localization::MSF_LOCAL_LIDAR_00) {
              msf_status_.set_msf_running_status(
                apollo::localization::MSF_NOSOL_LIDAR_GNSS);
          } else {
              msf_status_.set_msf_running_status(
                apollo::localization::MSF_NOSOL_X_GNSS);
          }
      } else if (msf_status_.gnsspos_position_type() == 4 ||
                 msf_status_.gnsspos_position_type() == 5 ||
                 msf_status_.gnsspos_position_type() == 6 ||
                 msf_status_.gnsspos_position_type() == 32 ||
                 msf_status_.gnsspos_position_type() == 33 ||
                 msf_status_.gnsspos_position_type() == 34) {
          if (msf_status_.local_lidar_status()
                == apollo::localization::MSF_LOCAL_LIDAR_00) {
              msf_status_.set_msf_running_status(
                apollo::localization::MSF_NOSOL_LIDAR_X);
          } else {
              msf_status_.set_msf_running_status(
                apollo::localization::MSF_NOSOL_X_X);
          }
      } else if (msf_status_.gnsspos_position_type() == 16 ||
                 msf_status_.gnsspos_position_type() == 17) {
          if (msf_status_.local_lidar_status()
                == apollo::localization::MSF_LOCAL_LIDAR_00) {
              msf_status_.set_msf_running_status(
                apollo::localization::MSF_NOSOL_LIDAR_XX);
          } else {
              msf_status_.set_msf_running_status(
                apollo::localization::MSF_NOSOL_X_XX);
          }
      } else {
          if (msf_status_.local_lidar_status()
                == apollo::localization::MSF_LOCAL_LIDAR_00) {
              msf_status_.set_msf_running_status(
                apollo::localization::MSF_NOSOL_LIDAR_XXX);
          } else {
              msf_status_.set_msf_running_status(
                apollo::localization::MSF_NOSOL_X_XXX);
          }
      }
  }
  MsfStatus *msf_status_ptr = integ_localization.mutable_msf_status();
  msf_status_ptr->set_local_lidar_consistency(
    msf_status_.local_lidar_consistency());
  msf_status_ptr->set_gnss_consistency(msf_status_.gnss_consistency());
  msf_status_ptr->set_local_lidar_status(msf_status_.local_lidar_status());
  msf_status_ptr->set_gnsspos_position_type(
    msf_status_.gnsspos_position_type());
  msf_status_ptr->set_msf_running_status(msf_status_.msf_running_status());
  msf_status_ptr->set_local_lidar_quality(msf_status_.local_lidar_quality());
  msf_status_mutex_.unlock();
  MsfSensorMsgStatus *sensor_status_ptr =
    integ_localization.mutable_sensor_status();
  sensor_status_ptr->set_imu_delay_status(sensor_status_.imu_delay_status());
  sensor_status_ptr->set_imu_missing_status(
    sensor_status_.imu_missing_status());
  sensor_status_ptr->set_imu_data_status(sensor_status_.imu_data_status());

  apollo::localization::Pose* posepb_loc = integ_localization.mutable_pose();

  if (imu_altitude_from_lidar_localization_available_) {
    apollo::common::PointENU* position = posepb_loc->mutable_position();
    position->set_z(imu_altitude_from_lidar_localization_);
  }

  // set linear acceleration
  Eigen::Vector3d orig_acceleration(imu_data.fb[0], imu_data.fb[1],
                                    imu_data.fb[2]);
  const apollo::common::Quaternion& orientation =
      integ_localization.pose().orientation();
  Eigen::Quaternion<double> quaternion(orientation.qw(), orientation.qx(),
                                       orientation.qy(), orientation.qz());
  Eigen::Vector3d vec_acceleration =
      quaternion.toRotationMatrix() * orig_acceleration;

  apollo::common::Point3D* linear_acceleration =
      posepb_loc->mutable_linear_acceleration();
  linear_acceleration->set_x(vec_acceleration(0));
  linear_acceleration->set_y(vec_acceleration(1));
  linear_acceleration->set_z(vec_acceleration(2) - 9.8);

  apollo::common::Point3D* linear_acceleration_vrf =
      posepb_loc->mutable_linear_acceleration_vrf();
  linear_acceleration_vrf->set_x(imu_data.fb[0]);
  linear_acceleration_vrf->set_y(imu_data.fb[1]);
  linear_acceleration_vrf->set_z(imu_data.fb[2]);

  // set angular velocity
  Eigen::Vector3d orig_angular_velocity(imu_data.wibb[0], imu_data.wibb[1],
                                        imu_data.wibb[2]);
  Eigen::Vector3d vec_angular_velocity =
      quaternion.toRotationMatrix() * orig_angular_velocity;
  apollo::common::Point3D* angular_velocity =
      posepb_loc->mutable_angular_velocity();
  angular_velocity->set_x(vec_angular_velocity(0));
  angular_velocity->set_y(vec_angular_velocity(1));
  angular_velocity->set_z(vec_angular_velocity(2));

  apollo::common::Point3D* angular_velocity_vrf =
      posepb_loc->mutable_angular_velocity_vrf();
  angular_velocity_vrf->set_x(imu_data.wibb[0]);
  angular_velocity_vrf->set_y(imu_data.wibb[1]);
  angular_velocity_vrf->set_z(imu_data.wibb[2]);

  integ_localization_mutex_.lock();
  integ_localization_list_.push_back(LocalizationResult(
      LocalizationMeasureState(static_cast<int>(state)), integ_localization));
  if (integ_localization_list_.size() > integ_localization_list_max_size_) {
    integ_localization_list_.pop_front();
  }
  integ_localization_mutex_.unlock();

  InsPva integ_sins_pva;
  double covariance[9][9];
  integ_process_->GetResult(&state, &integ_sins_pva, covariance);

  // update republish
  republish_process_->IntegPvaProcess(integ_sins_pva);

  if (state != IntegState::NOT_INIT) {
    // pass result of integration localization to lidar process module
    if (enable_lidar_localization_ && state != IntegState::NOT_STABLE) {
      lidar_process_->IntegPvaProcess(integ_sins_pva);
    }

    if (!is_use_gnss_bestpose_) {
      // pass localization result to gnss process module
      gnss_process_->IntegSinsPvaProcess(integ_sins_pva, covariance);
    }
  }
  return;
}

void LocalizationIntegImpl::RawObservationProcess(
    const drivers::gnss::EpochObservation& raw_obs_msg) {
  if (is_use_gnss_bestpose_) {
    return;
  }

  // push process function to queue
  gnss_function_queue_mutex_.lock();
  gnss_function_queue_.push(std::function<void()>(std::bind(
      &LocalizationIntegImpl::RawObservationProcessImpl, this, raw_obs_msg)));
  gnss_function_signal_.notify_one();
  gnss_function_queue_mutex_.unlock();

  return;
}

void LocalizationIntegImpl::RawEphemerisProcess(
    const drivers::gnss::GnssEphemeris& gnss_orbit_msg) {
  if (is_use_gnss_bestpose_) {
    return;
  }

  // push process function to queue
  gnss_function_queue_mutex_.lock();
  gnss_function_queue_.push(std::function<void()>(std::bind(
      &LocalizationIntegImpl::RawEphemerisProcessImpl, this, gnss_orbit_msg)));
  gnss_function_signal_.notify_one();
  gnss_function_queue_mutex_.unlock();

  return;
}

void LocalizationIntegImpl::GnssBestPoseProcess(
    const drivers::gnss::GnssBestPose& bestgnsspos_msg) {
  if (!is_use_gnss_bestpose_) {
    return;
  }

  // push process function to queue
  gnss_function_queue_mutex_.lock();
  gnss_function_queue_.push(std::function<void()>(std::bind(
      &LocalizationIntegImpl::GnssBestPoseProcessImpl, this, bestgnsspos_msg)));
  gnss_function_signal_.notify_one();
  gnss_function_queue_mutex_.unlock();

  return;
}

void LocalizationIntegImpl::GnssThreadLoop() {
  AINFO << "Started gnss process thread";
  while (keep_gnss_running_.load()) {
    {
      std::unique_lock<std::mutex> lock(gnss_function_queue_mutex_);
      size_t size = gnss_function_queue_.size();
      while (size > gnss_queue_max_size_) {
        gnss_function_queue_.pop();
        --size;
      }
      if (gnss_function_queue_.size() == 0) {
        gnss_function_signal_.wait(lock);
        continue;
      }
    }

    std::function<void()> gnss_func;
    int waiting_num = 0;
    {
      std::unique_lock<std::mutex> lock(gnss_function_queue_mutex_);
      gnss_func = gnss_function_queue_.front();
      gnss_function_queue_.pop();
      waiting_num = gnss_function_queue_.size();
    }

    if (waiting_num > 2) {
      AWARN << waiting_num << " gnss function are waiting to process.";
    }

    gnss_func();
  }
  AINFO << "Exited gnss process thread";
  return;
}

void LocalizationIntegImpl::RawObservationProcessImpl(
    const drivers::gnss::EpochObservation& raw_obs_msg) {
  gnss_process_->RawObservationProcess(raw_obs_msg);

  MeasureData gnss_measure;
  LocalizationMeasureState state = gnss_process_->GetResult(&gnss_measure);

  MeasureData measure;
  if (state == LocalizationMeasureState::OK ||
      state == LocalizationMeasureState::VALID) {
    republish_process_->GnssLocalProcess(gnss_measure, &measure);
    integ_process_->MeasureDataProcess(measure);
  }

  LocalizationEstimate gnss_localization;
  TransferGnssMeasureToLocalization(measure, &gnss_localization);

  gnss_localization_mutex_.lock();
  gnss_localization_list_.push_back(
      LocalizationResult(state, gnss_localization));
  if (gnss_localization_list_.size() > gnss_localization_list_max_size_) {
    gnss_localization_list_.pop_front();
  }
  gnss_localization_mutex_.unlock();
  return;
}

void LocalizationIntegImpl::RawEphemerisProcessImpl(
    const drivers::gnss::GnssEphemeris& gnss_orbit_msg) {
  gnss_process_->RawEphemerisProcess(gnss_orbit_msg);
  return;
}

void LocalizationIntegImpl::GnssBestPoseProcessImpl(
    const drivers::gnss::GnssBestPose& bestgnsspos_msg) {
  MeasureData measure;
  int gnss_status;
  if (republish_process_->NovatelBestgnssposProcess(
           bestgnsspos_msg, &measure, &gnss_status)) {
    integ_process_->MeasureDataProcess(measure);

    LocalizationEstimate gnss_localization;
    TransferGnssMeasureToLocalization(measure, &gnss_localization);
    MsfStatus *status = gnss_localization.mutable_msf_status();
    status->set_gnsspos_position_type(gnss_status);
    latest_gnsspos_timestamp_ = measure.time;

    gnss_localization_mutex_.lock();
    gnss_localization_list_.push_back(
        LocalizationResult(LocalizationMeasureState::OK, gnss_localization));
    if (gnss_localization_list_.size() > gnss_localization_list_max_size_) {
      gnss_localization_list_.pop_front();
    }
    gnss_localization_mutex_.unlock();
  }
  msf_status_mutex_.lock();
  msf_status_.set_gnsspos_position_type(gnss_status);
  msf_status_mutex_.unlock();
  return;
}

void LocalizationIntegImpl::TransferGnssMeasureToLocalization(
    const MeasureData& measure, LocalizationEstimate* localization) {
  CHECK_NOTNULL(localization);

  apollo::common::Header* headerpb = localization->mutable_header();
  apollo::localization::Pose* posepb = localization->mutable_pose();

  double timestamp = measure.time;
  localization->set_measurement_time(timestamp);
  headerpb->set_timestamp_sec(timestamp);

  UTMCoor utm_xy;
  LatlonToUtmXY(measure.gnss_pos.longitude, measure.gnss_pos.latitude, &utm_xy);

  apollo::common::PointENU* position = posepb->mutable_position();
  position->set_x(utm_xy.x);
  position->set_y(utm_xy.y);
  position->set_z(measure.gnss_pos.height);

  apollo::common::Quaternion* quaternion = posepb->mutable_orientation();
  quaternion->set_qx(0.0);
  quaternion->set_qy(0.0);
  quaternion->set_qz(0.0);
  quaternion->set_qw(1.0);

  apollo::localization::Uncertainty* uncertainty =
      localization->mutable_uncertainty();

  apollo::common::Point3D* position_std_dev =
      uncertainty->mutable_position_std_dev();
  position_std_dev->set_x(-1.0);
  position_std_dev->set_y(-1.0);
  position_std_dev->set_z(-1.0);

  if (measure.is_have_variance) {
    position_std_dev->set_x(measure.variance[0][0]);
    position_std_dev->set_y(measure.variance[1][1]);
    position_std_dev->set_z(measure.variance[2][2]);
  }

  apollo::common::Point3D* orientation_std_dev =
      uncertainty->mutable_orientation_std_dev();
  orientation_std_dev->set_x(-1.0);
  orientation_std_dev->set_y(-1.0);
  orientation_std_dev->set_z(-1.0);

  return;
}

void LocalizationIntegImpl::GetLastestLidarLocalization(
    LocalizationMeasureState* state, LocalizationEstimate* lidar_localization) {
  CHECK_NOTNULL(state);
  CHECK_NOTNULL(lidar_localization);

  lidar_localization_mutex_.lock();
  if (lidar_localization_list_.size()) {
    *state = lidar_localization_list_.front().state();
    *lidar_localization = lidar_localization_list_.front().localization();
    lidar_localization_list_.clear();
  } else {
    *state = LocalizationMeasureState::NOT_VALID;
  }
  lidar_localization_mutex_.unlock();
  return;
}

void LocalizationIntegImpl::GetLastestIntegLocalization(
    LocalizationMeasureState* state, LocalizationEstimate* integ_localization) {
  CHECK_NOTNULL(state);
  CHECK_NOTNULL(integ_localization);

  integ_localization_mutex_.lock();
  if (integ_localization_list_.size()) {
    *state = integ_localization_list_.front().state();
    *integ_localization = integ_localization_list_.front().localization();
    integ_localization_list_.clear();
  } else {
    *state = LocalizationMeasureState::NOT_VALID;
  }
  integ_localization_mutex_.unlock();
  return;
}

void LocalizationIntegImpl::GetLastestGnssLocalization(
    LocalizationMeasureState* state, LocalizationEstimate* gnss_localization) {
  CHECK_NOTNULL(state);
  CHECK_NOTNULL(gnss_localization);

  gnss_localization_mutex_.lock();
  if (gnss_localization_list_.size()) {
    *state = gnss_localization_list_.front().state();
    *gnss_localization = gnss_localization_list_.front().localization();
    gnss_localization_list_.clear();
  } else {
    *state = LocalizationMeasureState::NOT_VALID;
  }
  gnss_localization_mutex_.unlock();
  return;
}

void LocalizationIntegImpl::GetLidarLocalizationList(
    std::list<LocalizationResult>* results) {
  CHECK_NOTNULL(results);

  lidar_localization_mutex_.lock();
  *results = lidar_localization_list_;
  lidar_localization_list_.clear();
  lidar_localization_mutex_.unlock();
}

void LocalizationIntegImpl::GetIntegLocalizationList(
    std::list<LocalizationResult>* results) {
  CHECK_NOTNULL(results);

  integ_localization_mutex_.lock();
  *results = integ_localization_list_;
  integ_localization_list_.clear();
  integ_localization_mutex_.unlock();
}

void LocalizationIntegImpl::GetGnssLocalizationList(
    std::list<LocalizationResult>* results) {
  CHECK_NOTNULL(results);

  gnss_localization_mutex_.lock();
  *results = gnss_localization_list_;
  gnss_localization_list_.clear();
  gnss_localization_mutex_.unlock();
  return;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
