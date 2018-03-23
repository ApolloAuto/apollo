#include "modules/localization/msf/local_integ/localization_integ_process.h"
#include <yaml-cpp/yaml.h>
#include "modules/common/time/time.h"
#include "modules/common/time/timer.h"
#include "modules/common/log.h"
#include "modules/localization/msf/common/util/sleep.h"
#include "modules/localization/msf/common/util/frame_transform.h"
#include "math_util.h"

namespace apollo {
namespace localization {
namespace msf {

LocalizationIntegProcess::LocalizationIntegProcess() {
  integ_state_ = IntegState::NOT_INIT;
  memset(pva_covariance_, 0, sizeof(double) * 9 * 9);
  // is_sins_state_check_ = true;
  // sins_state_span_time_ = 60.0;
  // sins_state_pos_std_ = 1.0;
  delay_output_counter_ = 0;

  keep_running_ = true;
  measure_data_thread_ =
      std::thread(&LocalizationIntegProcess::MeasureDataThreadLoop, this);
  measure_data_queue_size_ = 150;

  sins_ = new Sins();
}

LocalizationIntegProcess::~LocalizationIntegProcess() {
  keep_running_ = false;
  new_measure_data_signal_.notify_one();
  measure_data_thread_.join();

  // pthread_mutex_destroy(&imu_mutex_);
  // pthread_mutex_destroy(&integ_time_update_mutex_);
  // pthread_mutex_destroy(&measure_callback_mutex_);

  delete sins_;
  sins_ = NULL;
}

LocalizationState LocalizationIntegProcess::Init(
    const LocalizationIntegParam &param) {
  // sins_state_span_time_

  sins_->Init(param.is_ins_can_self_align);
  sins_->SetSinsAlignFromVel(param.is_sins_align_with_vel);

  // sins_->SetSinsStateCheck(param.is_sins_state_check);
  sins_->SetResetSinsPoseStd(param.sins_state_pos_std);
  sins_->SetResetSinsMeasSpanTime(param.sins_state_span_time);

  if (param.is_using_raw_gnsspos) {
    gnss_antenna_extrinsic_.translation()(0) = param.imu_to_ant_offset.offset_x;
    gnss_antenna_extrinsic_.translation()(1) = param.imu_to_ant_offset.offset_y;
    gnss_antenna_extrinsic_.translation()(2) = param.imu_to_ant_offset.offset_z;
  } else {
    gnss_antenna_extrinsic_ = TransformD::Identity();
  }
  LOG(INFO) << "gnss and imu lever arm: "
            << gnss_antenna_extrinsic_.translation()(0) << " "
            << gnss_antenna_extrinsic_.translation()(1) << " "
            << gnss_antenna_extrinsic_.translation()(2);

  sins_->SetImuAntennaLeverArm(gnss_antenna_extrinsic_.translation()(0),
                               gnss_antenna_extrinsic_.translation()(1),
                               gnss_antenna_extrinsic_.translation()(2));
  
  sins_->SetVelThresholdGetYaw(param.vel_threshold_get_yaw);

  imu_rate_ = param.imu_rate;
  debug_log_flag_ = param.integ_debug_log_flag;

  return LocalizationState::OK();
}

void LocalizationIntegProcess::RawImuProcess(const ImuData &imu_msg) {
  integ_state_ = IntegState::NOT_INIT;
  double cur_imu_time = imu_msg.measurement_time;

  if (cur_imu_time < 3000) {
    LOG(INFO) << "the imu time is error: " << cur_imu_time;
    return;
  }

  static double pre_imu_time = cur_imu_time;
  double delta_time = cur_imu_time - pre_imu_time;
  if (delta_time > 0.1) {
    LOG(INFO)
        << std::setprecision(16)
        << "the imu message loss more than 10, the pre time and current time: "
        << pre_imu_time << " " << cur_imu_time;
  } else if (delta_time < 0.0) {
    LOG(INFO) << std::setprecision(16)
              << "received imu message's time is eary than last imu message, "
              << "the pre time and current time: " << pre_imu_time << " "
              << cur_imu_time;
  }

  // add imu msg and get current predict pose
  sins_->AddImu(imu_msg);
  sins_->GetPose(&ins_pva_, pva_covariance_);

  if (sins_->IsSinsAligned()) {
    integ_state_ = IntegState::NOT_STABLE;
    if (delay_output_counter_ < 3000) {
      ++delay_output_counter_;
    } else {
      integ_state_ = IntegState::OK;
      GetValidFromOK();
    }

    if (cur_imu_time - 0.5 > pre_imu_time) {
      std::cout << "SINS has completed alignment!" << std::endl;
      pre_imu_time = cur_imu_time;
    }
  } else {
    delay_output_counter_ = 0;
    if (cur_imu_time - 0.5 > pre_imu_time) {
      std::cout << "SINS is aligning!" << std::endl;
      pre_imu_time = cur_imu_time;
    }
  }

  pre_imu_time = cur_imu_time;

  // if (debug_log_flag_) {
  //   std::cerr << std::setprecision(16)
  //             << "IntegratedLocalization Debug Log: RawImu msg: "
  //             << "[time:" << cur_imu_time << "]" << std::endl;
  //   std::cerr << std::setprecision(6)
  //             << "[acc_x:" << imu_data.fb[0] << "]"
  //             << "[acc_y:" << imu_data.fb[1] << "]"
  //             << "[acc_z:" << imu_data.fb[2] << "]"
  //             << "[w_x:" << imu_data.wibb[0] << "]"
  //             << "[w_y:" << imu_data.wibb[1] << "]"
  //             << "[w_z:" << imu_data.wibb[2] << "]" << std::endl;
  // }

  return;
}

void LocalizationIntegProcess::GetValidFromOK() {
  if (integ_state_ != IntegState::OK) {
    return;
  }

  // LOG(ERROR) << pva_covariance_[0][0] << " " << pva_covariance_[1][1]
  //     << " " << pva_covariance_[2][2] << " " << pva_covariance_[8][8];
  if (pva_covariance_[0][0] < 0.3 * 0.3
      && pva_covariance_[1][1] < 0.3 * 0.3
      && pva_covariance_[2][2] < 0.3 * 0.3
      && pva_covariance_[8][8] < 0.1 * 0.1) {
    integ_state_ = IntegState::VALID;
  }  
  return;
}

void LocalizationIntegProcess::GetState(IntegState &state) {
  state = integ_state_;
}

void LocalizationIntegProcess::GetResult(IntegState &state,
                                         InsPva &sins_pva,
                                         LocalizationEstimate &localization) {
  // state
  state = integ_state_;

  // IntegSinsPva
  sins_pva = ins_pva_;

  if (debug_log_flag_ && state != IntegState::NOT_INIT) {
    LOG(INFO) << std::setprecision(16)
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
  apollo::common::Header *headerpb_loc = localization.mutable_header();
  apollo::localization::Pose *posepb_loc = localization.mutable_pose();

  localization.set_measurement_time(ins_pva_.time);
  headerpb_loc->set_timestamp_sec(apollo::common::time::Clock::NowInSeconds());
  // headerpb_loc->set_module_name(_param.publish_frame_id);

  apollo::common::PointENU *position_loc = posepb_loc->mutable_position();
  apollo::common::Quaternion *quaternion = posepb_loc->mutable_orientation();
  UTMCoor utm_xy;
  latlon_to_utmxy(ins_pva_.pos.longitude, ins_pva_.pos.latitude, &utm_xy);
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
  eulerangles->set_z(-ins_pva_.att.yaw);
  
  posepb_loc->set_heading(-ins_pva_.att.yaw);

  apollo::localization::Uncertainty *uncertainty =
      localization.mutable_uncertainty();
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
  return;
}

void LocalizationIntegProcess::GetResult(MeasureData& measure_data) {
  measure_data.time = ins_pva_.time;
  measure_data.gnss_pos.longitude = ins_pva_.pos.longitude;
  measure_data.gnss_pos.latitude = ins_pva_.pos.latitude;
  measure_data.gnss_pos.height = ins_pva_.pos.height;
  measure_data.gnss_vel.ve = ins_pva_.vel.ve;
  measure_data.gnss_vel.vn = ins_pva_.vel.vn;
  measure_data.gnss_vel.vu = ins_pva_.vel.vu;
  measure_data.gnss_att.pitch = ins_pva_.att.pitch;
  measure_data.gnss_att.roll = ins_pva_.att.roll;
  measure_data.gnss_att.yaw = ins_pva_.att.yaw;

  measure_data.is_have_variance = true;
  
  for (int i = 0; i < 9; ++i) {
    for (int j = 0; j < 9; ++j) {
      measure_data.variance[i][j] = pva_covariance_[i][j];
    }
  }
  
  return;
}

void LocalizationIntegProcess::MeasureDataProcess(
    const MeasureData &measure_msg) {
  measure_data_queue_mutex_.lock();
  measure_data_queue_.push(measure_msg);
  new_measure_data_signal_.notify_one();
  measure_data_queue_mutex_.unlock();
}

void LocalizationIntegProcess::MeasureDataThreadLoop() {
  LOG(INFO) << "Started measure data process thread";
  while (keep_running_.load()) {
    {
      std::unique_lock<std::mutex> lock(measure_data_queue_mutex_);
      int size = measure_data_queue_.size();
      while (size > measure_data_queue_size_) {
        measure_data_queue_.pop();
        --size;
      }
      if (measure_data_queue_.size() == 0) {
        new_measure_data_signal_.wait(lock);
        continue;
      }
    }

    MeasureData measure;
    {
      std::unique_lock<std::mutex> lock(measure_data_queue_mutex_);
      measure = measure_data_queue_.front();
      measure_data_queue_.pop();
    }
    MeasureDataProcessImpl(measure);
  }
  LOG(INFO) << "Exited measure data process thread";
}

void LocalizationIntegProcess::MeasureDataProcessImpl(
    const MeasureData &measure_msg) {
  common::time::Timer timer;
  timer.Start();

  if (!CheckIntegMeasureData(measure_msg)) {
    return;
  }

  sins_->AddMeasurement(measure_msg);

  timer.End("time of integrated navigation measure update");

  // pthread_mutex_lock(&measure_callback_mutex_);
  // if (measure_callback_execute_) {
  //   return;
  // } else {
  //   measure_callback_execute_ = true;
  // }
  // pthread_mutex_unlock(&measure_callback_mutex_);

  // MeasureData measure_data = measure_msg;
  // if (CheckIntegMeasureData(measure_data)) {
  //   pthread_mutex_lock(&imu_mutex_);
  //   double cur_imu_time = rawimu_list_measure_update_.back().time;
  //   pthread_mutex_unlock(&imu_mutex_);

  //   while (cur_imu_time < measure_data.time) {
  //     Sleep::SleepSec(0.003);
  //     // ros::Duration(0.003).sleep();

  //     pthread_mutex_lock(&imu_mutex_);
  //     cur_imu_time = rawimu_list_measure_update_.back().time;
  //     pthread_mutex_unlock(&imu_mutex_);
  //   }

  //   if (measure_data.time > measure_data_list_.back().time ||
  //       measure_data_list_.empty()) {
  //     measure_data_list_.push_back(measure_data);
  //   } else {
  //     for (std::list<MeasureData>::iterator iter = measure_data_list_.begin();
  //          iter != measure_data_list_.end();) {
  //       if ((measure_data.time >= (*iter).time) &&
  //           (measure_data.time < (*(++iter)).time)) {
  //         measure_data_list_.insert(iter, measure_data);
  //         break;
  //       } else {
  //         iter++;
  //       }
  //     }
  //   }
  // } else {
  //   if (measure_data_list_.empty()) {
  //     pthread_mutex_lock(&measure_callback_mutex_);
  //     measure_callback_execute_ = false;
  //     pthread_mutex_unlock(&measure_callback_mutex_);
  //     return;
  //   }
  // }

  // SinsImuData imu_data = {0.0};
  // InsPva ins_pva;
  // double pva_covariance[9][9] = {0.0};

  // pthread_mutex_lock(&integ_time_update_mutex_);
  // bool is_system_init = integ_nav_sins_update_.is_integ_nav_system_init();
  // pthread_mutex_unlock(&integ_time_update_mutex_);
  // // the integrated navigation system init
  // if (!is_system_init) {
  //   measure_data = measure_data_list_.back();
  //   if (measure_data.measure_type ==
  //       adu::localization::integrated_navigation::ODOMETER_VEL_ONLY) {
  //     LOG(ERROR) << "the last measure data is odometry!\n";
  //   }
  //   measure_data_list_.clear();

  //   pthread_mutex_lock(&integ_time_update_mutex_);
  //   integ_nav_sins_update_.integrated_navigation_init(&measure_data);
  //   integ_nav_measure_update_.integrated_navigation_init(&measure_data);
  //   adu::localization::integrated_navigation::Vector3d gnss_level_arm = {
  //       gnss_antenna_extrinsic_.translation()(0),
  //       gnss_antenna_extrinsic_.translation()(1),
  //       gnss_antenna_extrinsic_.translation()(2)};
  //   integ_nav_sins_update_.set_imu_antenna_lever_arm(gnss_level_arm);
  //   integ_nav_measure_update_.set_imu_antenna_lever_arm(gnss_level_arm);

  //   // integ_nav_sins_update_.set_vel_threshold_get_yaw(vel_threshold_get_yaw_);
  //   // integ_nav_measure_update_.set_vel_threshold_get_yaw(vel_threshold_get_yaw_);

  //   if (is_sins_align_with_vel_) {
  //     integ_nav_sins_update_.set_sins_align_from_vel(true);
  //     integ_nav_measure_update_.set_sins_align_from_vel(true);
  //   }

  //   integ_nav_measure_update_ = integ_nav_sins_update_;
  //   pthread_mutex_unlock(&integ_time_update_mutex_);
  //   pre_measure_update_time_ = measure_data.time;

  //   pthread_mutex_lock(&measure_callback_mutex_);
  //   measure_callback_execute_ = false;
  //   pthread_mutex_unlock(&measure_callback_mutex_);

  //   timer.End("time of integrated navigation measure update");
  //   return;
  // }

  // pthread_mutex_lock(&imu_mutex_);
  // // get the imu current imu list for measure update
  // double cur_imu_time = (rawimu_list_measure_update_.back()).time;
  // LOG(INFO) << std::setprecision(16)
  //           << "the current imu time and measure time: " << cur_imu_time << " "
  //           << measure_data.time;
  // pthread_mutex_unlock(&imu_mutex_);

  // double first_measure_time = measure_data_list_.front().time;
  // if ((first_measure_time + 0.00 > cur_imu_time) &&
  //     (int(measure_data.measure_type) != 3)) {
  //   pthread_mutex_lock(&measure_callback_mutex_);
  //   measure_callback_execute_ = false;
  //   pthread_mutex_unlock(&measure_callback_mutex_);
  //   return;
  // }

  // pthread_mutex_lock(&integ_time_update_mutex_);
  // integ_nav_measure_update_ = integ_nav_sins_update_;
  // pthread_mutex_unlock(&integ_time_update_mutex_);

  // int measure_data_counter = 0;
  // double measure_update_time = 0.0;
  // for (std::list<MeasureData>::iterator measure_iter =
  //          measure_data_list_.begin();
  //      measure_iter != measure_data_list_.end();) {
  //   if (measure_iter->time + 0.00 > cur_imu_time) {
  //     break;
  //   }
  //   measure_data = *measure_iter;
  //   measure_update_time = measure_data.time;
  //   integ_nav_measure_update_.receive_new_measure_data(&measure_data);
  //   ++measure_data_counter;
  //   measure_iter = measure_data_list_.erase(measure_iter);
  // }

  // if (measure_data_counter == 0) {
  //   pthread_mutex_lock(&measure_callback_mutex_);
  //   measure_callback_execute_ = false;
  //   pthread_mutex_unlock(&measure_callback_mutex_);

  //   return;
  // }
  // // the main measure update of the system
  // integ_nav_measure_update_.integrated_navigation_main_fun(
  //     &imu_data, &ins_pva, pva_covariance,
  //     adu::localization::integrated_navigation::FILTER_MEASURE_UPDATE);
  // pre_measure_update_time_ = cur_imu_time;

  // bool is_imu_end = false;
  // pthread_mutex_lock(&imu_mutex_);
  // std::list<SinsImuData>::iterator imu_iter =
  //     rawimu_list_measure_update_.begin();
  // is_imu_end = (imu_iter == rawimu_list_measure_update_.end());
  // pthread_mutex_unlock(&imu_mutex_);
  // if (is_imu_end) {
  //   pthread_mutex_lock(&integ_time_update_mutex_);
  //   integ_nav_sins_update_ = integ_nav_measure_update_;
  //   pthread_mutex_unlock(&integ_time_update_mutex_);

  //   pthread_mutex_lock(&measure_callback_mutex_);
  //   measure_callback_execute_ = false;
  //   pthread_mutex_unlock(&measure_callback_mutex_);
  //   return;
  // }
  // do {
  //   if (imu_iter->time <= pre_measure_update_time_) {
  //     if (imu_iter->time <= measure_update_time) {
  //       pthread_mutex_lock(&imu_mutex_);
  //       imu_iter = rawimu_list_measure_update_.erase(imu_iter);
  //       is_imu_end = (imu_iter == rawimu_list_measure_update_.end());
  //       pthread_mutex_unlock(&imu_mutex_);
  //     } else {
  //       ++imu_iter;
  //       pthread_mutex_lock(&imu_mutex_);
  //       is_imu_end = (imu_iter == rawimu_list_measure_update_.end());
  //       pthread_mutex_unlock(&imu_mutex_);
  //     }
  //     continue;
  //   } else {
  //     integ_nav_measure_update_.integrated_navigation_main_fun(
  //         &(*imu_iter), &ins_pva, pva_covariance,
  //         adu::localization::integrated_navigation::SINS_FILTER_TIME_UPDATE);
  //     ++imu_iter;
  //     pthread_mutex_lock(&imu_mutex_);
  //     is_imu_end = (imu_iter == rawimu_list_measure_update_.end());
  //     pthread_mutex_unlock(&imu_mutex_);
  //   }
  // } while (!is_imu_end);

  // pthread_mutex_lock(&integ_time_update_mutex_);
  // integ_nav_sins_update_ = integ_nav_measure_update_;
  // pthread_mutex_unlock(&integ_time_update_mutex_);

  // pthread_mutex_lock(&measure_callback_mutex_);
  // measure_callback_execute_ = false;
  // pthread_mutex_unlock(&measure_callback_mutex_);

  // timer.End("time of integrated navigation measure update");
  return;
}

bool LocalizationIntegProcess::CheckIntegMeasureData(const MeasureData& measure_data) {
  if (measure_data.measure_type == MeasureType::ODOMETER_VEL_ONLY) {
    LOG(ERROR) << "receive a new odometry measurement!!!\n";
  }

  if (debug_log_flag_) {
    LOG(INFO) << std::setprecision(16)
              << "IntegratedLocalization Debug Log: measure data: "
              << "[time:" << measure_data.time << "]"
              << "[x:" << measure_data.gnss_pos.longitude * 57.295779513082323
              << "]"
              << "[y:" << measure_data.gnss_pos.latitude * 57.295779513082323
              << "]"
              << "[z:" << measure_data.gnss_pos.height << "]"
              << "[ve:" << measure_data.gnss_vel.ve << "]"
              << "[vn:" << measure_data.gnss_vel.vn << "]"
              << "[vu:" << measure_data.gnss_vel.vu << "]"
              << "[pitch:" << measure_data.gnss_att.pitch * 57.295779513082323
              << "]"
              << "[roll:" << measure_data.gnss_att.roll * 57.295779513082323
              << "]"
              << "[yaw:" << measure_data.gnss_att.yaw * 57.295779513082323
              << "]"
              << "[measure type:" << int(measure_data.measure_type) << "]";
  }

  return true;
}

// bool LocalizationIntegProcess::GetIntegMeasureData(
//     const MeasureData &measure_msg, MeasureData *measure_data) {
//   measure_data->time = measure_msg.header().timestamp_sec();
//   measure_data->gnss_pos.longitude = measure_msg.position().x();
//   measure_data->gnss_pos.latitude = measure_msg.position().y();
//   measure_data->gnss_pos.height = measure_msg.position().z();

//   measure_data->gnss_vel.ve = measure_msg.velocity().x();
//   measure_data->gnss_vel.vn = measure_msg.velocity().y();
//   measure_data->gnss_vel.vu = measure_msg.velocity().z();

//   measure_data->gnss_att.pitch = 0.0;
//   measure_data->gnss_att.roll = 0.0;
//   measure_data->gnss_att.yaw = measure_msg.yaw();

//   measure_data->measure_type =
//       (adu::localization::integrated_navigation::MeasureType) int(
//           measure_msg.measure_type());
//   if (measure_data->measure_type ==
//       adu::localization::integrated_navigation::ODOMETER_VEL_ONLY) {
//     LOG(ERROR) << "receive a new odometry measurement!!!\n";
//   }

//   measure_data->frame_type =
//       (adu::localization::integrated_navigation::FrameType) int(
//           measure_msg.frame_type());
//   measure_data->is_have_variance = measure_msg.is_have_variance();

//   if (measure_data->is_have_variance) {
//     for (int i = 0; i < 9; ++i) {
//       for (int j = 0; j < 9; ++j) {
//         measure_data->variance[i][j] = measure_msg.measure_covar(i * 9 + j);  
//       }
//     }
//   }

//   if (debug_log_flag_) {
//     LOG(INFO) << std::setprecision(16)
//               << "IntegratedLocalization Debug Log: measure data: "
//               << "[time:" << measure_data->time << "]"
//               << "[x:" << measure_data->gnss_pos.longitude * 57.295779513082323
//               << "]"
//               << "[y:" << measure_data->gnss_pos.latitude * 57.295779513082323
//               << "]"
//               << "[z:" << measure_data->gnss_pos.height << "]"
//               << "[ve:" << measure_data->gnss_vel.ve << "]"
//               << "[vn:" << measure_data->gnss_vel.vn << "]"
//               << "[vu:" << measure_data->gnss_vel.vu << "]"
//               << "[pitch:" << measure_data->gnss_att.pitch * 57.295779513082323
//               << "]"
//               << "[roll:" << measure_data->gnss_att.roll * 57.295779513082323
//               << "]"
//               << "[yaw:" << measure_data->gnss_att.yaw * 57.295779513082323
//               << "]"
//               << "[measure type:" << int(measure_data->measure_type) << "]";
//   }

//   return true;
// }

bool LocalizationIntegProcess::LoadGnssAntennaExtrinsic(
    std::string file_path, TransformD &extrinsic) const {
  YAML::Node confige = YAML::LoadFile(file_path);
  if (confige["leverarm"]) {
    if (confige["leverarm"]["primary"]["offset"]) {
      extrinsic.translation()(0) =
          confige["leverarm"]["primary"]["offset"]["x"].as<double>();
      extrinsic.translation()(1) =
          confige["leverarm"]["primary"]["offset"]["y"].as<double>();
      extrinsic.translation()(2) =
          confige["leverarm"]["primary"]["offset"]["z"].as<double>();
      return true;
    }
  }

  return false;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
