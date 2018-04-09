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

#include "modules/localization/msf/local_integ/localization_integ_impl.h"
#include <list>
#include <queue>
#include "modules/localization/msf/common/util/time_conversion.h"
#include "modules/localization/msf/common/util/timer.h"
#include "modules/common/log.h"
#include "modules/localization/msf/common/util/frame_transform.h"

namespace apollo {
namespace localization {
namespace msf {

using common::Status;

LocalizationIntegImpl::LocalizationIntegImpl() {
  republish_process_ = new MeasureRepublishProcess;
  integ_process_ = new LocalizationIntegProcess;
  gnss_process_ = new LocalizationGnssProcess;
  lidar_process_ = new LocalizationLidarProcess;

  lidar_localization_list_max_size_ = 10;
  integ_localization_list_max_size_ = 50;
  gnss_localization_list_max_size_ = 10;

  is_use_gnss_bestpose_ = true;

  debug_log_flag_ = true;

  // run process thread
  keep_lidar_running_ = true;
  lidar_queue_max_size_ = 5;
  lidar_data_thread_ = std::thread(&LocalizationIntegImpl::PcdThreadLoop, this);

  keep_imu_running_ = true;
  imu_queue_max_size_ = 200;
  imu_data_thread_ = std::thread(&LocalizationIntegImpl::ImuThreadLoop, this);

  keep_gnss_running_ = true;
  gnss_queue_max_size_ = 100;
  gnss_function_thread_ =
      std::thread(&LocalizationIntegImpl::GnssThreadLoop, this);

  imu_altitude_from_lidar_localization_ = 0;
  imu_altitude_from_lidar_localization_available_ = false;
}

LocalizationIntegImpl::~LocalizationIntegImpl() {
  keep_lidar_running_ = false;
  lidar_data_signal_.notify_one();
  lidar_data_thread_.join();

  keep_imu_running_ = false;
  imu_data_signal_.notify_one();
  imu_data_thread_.join();

  keep_gnss_running_ = false;
  gnss_function_signal_.notify_one();
  gnss_function_thread_.join();

  delete republish_process_;
  delete lidar_process_;
  delete gnss_process_;
  delete integ_process_;
}

Status LocalizationIntegImpl::Init(
    const LocalizationIntegParam& params) {
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
  LOG(INFO) << "gnss and imu lever arm: "
            << gnss_antenna_extrinsic_.translation()(0) << " "
            << gnss_antenna_extrinsic_.translation()(1) << " "
            << gnss_antenna_extrinsic_.translation()(2);

  return Status::OK();
}

void LocalizationIntegImpl::PcdProcess(
    const LidarFrame& lidar_frame) {
  // LidarFrame lidar_frame;
  // ParseLidarFrame(message, lidar_frame);

  lidar_data_queue_mutex_.lock();
  lidar_data_queue_.push(lidar_frame);
  lidar_data_signal_.notify_one();
  lidar_data_queue_mutex_.unlock();
  return;
}

void LocalizationIntegImpl::PcdThreadLoop() {
  LOG(INFO) << "Started pcd data process thread";
  while (keep_lidar_running_.load()) {
    {
      std::unique_lock<std::mutex> lock(lidar_data_queue_mutex_);
      int size = lidar_data_queue_.size();
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
      LOG(WARNING) << waiting_num
                   << " point cloud msg are waiting to process.";
    }

    PcdProcessImpl(lidar_frame);
  }
  LOG(INFO) << "Exited pcd data process thread";
}

void LocalizationIntegImpl::PcdProcessImpl(const LidarFrame& pcd_data) {
  // lidar -> republish -> integ
  lidar_process_->PcdProcess(pcd_data);

  int state = 0;
  LocalizationEstimate lidar_localization;

  state = lidar_process_->GetResult(&lidar_localization);
  //   lidar_localization_state_ = LocalizationMeasureState(state);

  MeasureData lidar_measure;
  if (state == 2) {  // only state OK republish lidar msg
    republish_process_->LidarLocalProcess(lidar_localization, &lidar_measure);
    integ_process_->MeasureDataProcess(lidar_measure);

    imu_altitude_from_lidar_localization_ =
        lidar_localization.pose().position().z();
    imu_altitude_from_lidar_localization_available_ = true;

    // LocalOnlineVisualizer::instance()->push_lidar_measure(lidar_measure);
  }

  lidar_localization_mutex_.lock();
  lidar_localization_list_.push_back(
      LocalizationResult(LocalizationMeasureState(state), lidar_localization));
  if (lidar_localization_list_.size() > lidar_localization_list_max_size_) {
    lidar_localization_list_.pop_front();
  }
  // lidar_localization_state_ = LocalizationMeasureState(state);
  // lidar_localization_ = lidar_localization;
  lidar_localization_mutex_.unlock();
}

void LocalizationIntegImpl::RawImuProcessRfu(
    const ImuData& imu_data) {
  // push to imu_data_queue
  imu_data_queue_mutex_.lock();
  imu_data_queue_.push(imu_data);
  imu_data_signal_.notify_one();
  imu_data_queue_mutex_.unlock();
}

void LocalizationIntegImpl::ImuThreadLoop() {
  LOG(INFO) << "Started imu data process thread";
  while (keep_imu_running_.load()) {
    {
      std::unique_lock<std::mutex> lock(imu_data_queue_mutex_);
      int size = imu_data_queue_.size();
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
      LOG(WARNING) << waiting_num << " imu msg are waiting to process.";
    }

    ImuProcessImpl(imu_data);
  }
  LOG(INFO) << "Exited imu data process thread";
}

void LocalizationIntegImpl::ImuProcessImpl(const ImuData& imu_data) {
  // imu -> lidar
  // imu -> integ -> republish -> lidar -> publish

  // start_time = boost::posix_time::microsec_clock::local_time();
  if (enable_lidar_localization_) {
    lidar_process_->RawImuProcess(imu_data);
  }
  integ_process_->RawImuProcess(imu_data);

  // integ
  IntegState state;
  LocalizationEstimate integ_localization;
  integ_process_->GetResult(&state, &integ_localization);

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
  // integ_localization_state_ = LocalizationMeasureState(int(state));
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
    // update lidar
    if (enable_lidar_localization_
        && state != IntegState::NOT_STABLE) {
      lidar_process_->IntegPvaProcess(integ_sins_pva);
    }

    if (!is_use_gnss_bestpose_) {
      // update gnssW
      // MeasureData measure_data = {0.0};
      // integ_process_->GetResult(&measure_data);
      gnss_process_->IntegSinsPvaProcess(integ_sins_pva, covariance);
    }
  }

  // timer.End("imu Process");

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

  // std::cerr << "receive a best gnss pose." << std::endl;

  // push process function to queue
  gnss_function_queue_mutex_.lock();
  gnss_function_queue_.push(std::function<void()>(std::bind(
      &LocalizationIntegImpl::GnssBestPoseProcessImpl, this, bestgnsspos_msg)));
  gnss_function_signal_.notify_one();
  gnss_function_queue_mutex_.unlock();

  return;
}

void LocalizationIntegImpl::GnssThreadLoop() {
  LOG(INFO) << "Started gnss process thread";
  while (keep_gnss_running_.load()) {
    {
      std::unique_lock<std::mutex> lock(gnss_function_queue_mutex_);
      int size = gnss_function_queue_.size();
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
      LOG(WARNING) << waiting_num << " gnss function are waiting to process.";
    }

    gnss_func();
  }
  LOG(INFO) << "Exited gnss process thread";
  return;
}

void LocalizationIntegImpl::RawObservationProcessImpl(
    const drivers::gnss::EpochObservation& raw_obs_msg) {
  gnss_process_->RawObservationProcess(raw_obs_msg);

  MeasureData gnss_measure;
  LocalizationMeasureState state = gnss_process_->GetResult(&gnss_measure);

  MeasureData measure;
  if (state == LocalizationMeasureState::OK
      || state == LocalizationMeasureState::VALID) {
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
  if (republish_process_->NovatelBestgnssposProcess(
      bestgnsspos_msg, &measure)) {
    integ_process_->MeasureDataProcess(measure);

    LocalizationEstimate gnss_localization;
    TransferGnssMeasureToLocalization(measure, &gnss_localization);

    gnss_localization_mutex_.lock();
    gnss_localization_list_.push_back(
        LocalizationResult(LocalizationMeasureState::OK, gnss_localization));
    if (gnss_localization_list_.size() > gnss_localization_list_max_size_) {
      gnss_localization_list_.pop_front();
    }
    gnss_localization_mutex_.unlock();
  }
  return;
}

void LocalizationIntegImpl::TransferGnssMeasureToLocalization(
    const MeasureData& measure, LocalizationEstimate *localization) {
  apollo::common::Header* headerpb = localization->mutable_header();
  apollo::localization::Pose* posepb = localization->mutable_pose();

  double timestamp = measure.time;
  localization->set_measurement_time(timestamp);
  headerpb->set_timestamp_sec(timestamp);

  UTMCoor utm_xy;
  latlon_to_utmxy(measure.gnss_pos.longitude,
                  measure.gnss_pos.latitude,
                  &utm_xy);

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
    LocalizationMeasureState *state,
    LocalizationEstimate *lidar_localization) {
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
    LocalizationMeasureState *state,
    LocalizationEstimate *integ_localization) {
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
    LocalizationMeasureState *state,
    LocalizationEstimate *gnss_localization) {
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
    std::list<LocalizationResult> *results) {
  lidar_localization_mutex_.lock();
  *results = lidar_localization_list_;
  lidar_localization_list_.clear();
  lidar_localization_mutex_.unlock();
}

void LocalizationIntegImpl::GetIntegLocalizationList(
    std::list<LocalizationResult> *results) {
  integ_localization_mutex_.lock();
  *results = integ_localization_list_;
  integ_localization_list_.clear();
  integ_localization_mutex_.unlock();
}

void LocalizationIntegImpl::GetGnssLocalizationList(
    std::list<LocalizationResult> *results) {
  gnss_localization_mutex_.lock();
  *results = gnss_localization_list_;
  gnss_localization_list_.clear();
  gnss_localization_mutex_.unlock();
  return;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
