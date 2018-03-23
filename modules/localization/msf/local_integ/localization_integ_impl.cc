#include "localization_integ_impl.h"
#include "modules/localization/msf/common/util/time_conversion.h"
#include "modules/localization/msf/common/util/timer.h"
#include "glog/logging.h"
#include "glog/raw_logging.h"
#include "modules/localization/msf/common/util/frame_transform.h"

namespace apollo {
namespace localization {
namespace msf {

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

LocalizationState LocalizationIntegImpl::Init(
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

  if (params.gnss_mode == int(GnssMode::SELF)) {
    state = gnss_process_->Init(params);
    is_use_gnss_bestpose_ = false;
    if (!state.ok()) {
      return state;
    }
  } else {
    is_use_gnss_bestpose_ = true;
  }

  if (params.is_using_raw_gnsspos) {
    gnss_antenna_extrinsic_.translation()(0) = params.imu_to_ant_offset.offset_x;
    gnss_antenna_extrinsic_.translation()(1) = params.imu_to_ant_offset.offset_y;
    gnss_antenna_extrinsic_.translation()(2) = params.imu_to_ant_offset.offset_z;
  } else {
    gnss_antenna_extrinsic_ = Eigen::Affine3d::Identity();
  }
  LOG(INFO) << "gnss and imu lever arm: "
            << gnss_antenna_extrinsic_.translation()(0) << " "
            << gnss_antenna_extrinsic_.translation()(1) << " "
            << gnss_antenna_extrinsic_.translation()(2);

  // LocalOnlineVisualizer::instance()->Init(params.is_use_visualize);
  // LocalOnlineVisualizer::instance()->SetFilterSize(params.lidar_filter_size,
  //                                                  params.lidar_filter_size);

  return LocalizationState::OK();
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
      LOG(WARNING) << waiting_num << " point cloud msg are waiting to process.";
    } 
    // else {
    //   LOG(INFO) << waiting_num << " point cloud msg are waiting to process.";
    // }

    PcdProcessImpl(lidar_frame);
  }
  LOG(INFO) << "Exited pcd data process thread";
}

void LocalizationIntegImpl::PcdProcessImpl(LidarFrame& pcd_data) {
  // lidar -> republish -> integ
  lidar_process_->PcdProcess(pcd_data);

  int state = 0;
  LocalizationEstimate lidar_localization;

  state = lidar_process_->GetResult(lidar_localization);
  //   lidar_localization_state_ = LocalizationMeasureState(state);

  MeasureData lidar_measure;
  if (state == 2) { // only state OK republish lidar msg
    // TODO republish refactoring

    republish_process_->LidarLocalProcess(lidar_localization, lidar_measure);
    integ_process_->MeasureDataProcess(lidar_measure);

    imu_altitude_from_lidar_localization_ = lidar_localization.pose().position().z();
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
  // ImuData imu_data = {0.0};
  // // double header_time = imu_msg.header().timestamp_sec();
  // double measurement_time = util::GpsToUnixSeconds(imu_msg.measurement_time());
  // // std::cerr << "imu time diff: " << header_time - measurement_time <<
  // // std::endl; std::cerr << std::setprecision(16) << "imu time: " <<
  // // measurement_time << std::endl; imu_data.time =
  // // imu_msg.header().timestamp_sec(); imu_data.time =
  // // util::GpsToUnixSeconds(imu_msg.measurement_time());
  // imu_data.time = measurement_time;
  // imu_data.fb[0] = imu_msg.linear_acceleration().x();
  // imu_data.fb[1] = imu_msg.linear_acceleration().y();
  // imu_data.fb[2] = imu_msg.linear_acceleration().z();

  // imu_data.wibb[0] = imu_msg.angular_velocity().x();  // * imu_rate_;
  // imu_data.wibb[1] = imu_msg.angular_velocity().y();  // * imu_rate_;
  // imu_data.wibb[2] = imu_msg.angular_velocity().z();  // * imu_rate_;

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
    // else {
    //   LOG(INFO) << waiting_num << " imu msg are waiting to process.";
    // }

    ImuProcessImpl(imu_data);
  }
  LOG(INFO) << "Exited imu data process thread";
}

void LocalizationIntegImpl::ImuProcessImpl(const ImuData& imu_data) {
  // imu -> lidar
  // imu -> integ -> republish -> lidar -> publish

  // std::cerr << std::setprecision(16) << "imu time: " << imu_data.time <<
  // std::endl;

  // Timer timer;
  // timer.Start();

  // start_time = boost::posix_time::microsec_clock::local_time();
  if (enable_lidar_localization_) {
    lidar_process_->RawImuProcess(imu_data);
  }
  integ_process_->RawImuProcess(imu_data);

  // integ
  IntegState state;
  LocalizationEstimate integ_localization;
  InsPva integ_sins_pva;

  integ_localization_mutex_.lock();
  integ_process_->GetResult(state, integ_sins_pva, integ_localization);

  // push integ pose to PoseQuery
  auto pose_qurey = integ_localization.pose().position();
  auto orien_qurey = integ_localization.pose().orientation();
  double time_qurey = integ_localization.measurement_time();
  integ_pose_query_.add_pose(time_qurey, 
      Eigen::Vector3d(pose_qurey.x(), pose_qurey.y(), pose_qurey.z()), 
      Eigen::Quaterniond(orien_qurey.qw(), orien_qurey.qx(), orien_qurey.qy(), orien_qurey.qz()));

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

  // integ_localization_state_ = LocalizationMeasureState(int(state));
  integ_localization_list_.push_back(LocalizationResult(
      LocalizationMeasureState(int(state)), integ_localization));
  if (integ_localization_list_.size() > integ_localization_list_max_size_) {
    integ_localization_list_.pop_front();
  }
  integ_localization_mutex_.unlock();

  // update republish
  republish_process_->IntegPvaProcess(integ_sins_pva);
  
  if (state != IntegState::NOT_INIT) {
    // update lidar
    if (enable_lidar_localization_) {
      lidar_process_->IntegPvaProcess(integ_sins_pva);
    }

    // LocalOnlineVisualizer::instance()->push_sins_pva(integ_sins_pva);
    // LocalOnlineVisualizer::instance()->push_imu(imu_data);

    if (!is_use_gnss_bestpose_) {
      // update gnssW
      MeasureData measure_data = {0.0};
      integ_process_->GetResult(measure_data);
      gnss_process_->IntegSinsPvaProcess(integ_sins_pva, measure_data);
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
    // else {
    //   LOG(INFO) << waiting_num << " gnss function are waiting to process.";
    // }

    gnss_func();
  }
  LOG(INFO) << "Exited gnss process thread";
  return;
}

void LocalizationIntegImpl::RawObservationProcessImpl(
    const drivers::gnss::EpochObservation& raw_obs_msg) {
  gnss_process_->RawObservationProcess(raw_obs_msg);

  MeasureData gnss_measure;
  LocalizationMeasureState state = gnss_process_->GetResult(gnss_measure);

  MeasureData measure;
  if (state == LocalizationMeasureState::OK 
      || state == LocalizationMeasureState::VALID) {
    republish_process_->GnssLocalProcess(gnss_measure, measure);
    integ_process_->MeasureDataProcess(measure);
    // LocalOnlineVisualizer::instance()->push_gnss_measure(measure);
  }

  LocalizationEstimate gnss_localization;
  TransferGnssMeasureToLocalization(measure, gnss_localization);

  gnss_localization_mutex_.lock();
  // gnss_localization_state_ = state;
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
  // TODO use GnssInfoType to on/of callback
  MeasureData measure;
  if (republish_process_->NovatelBestgnssposProcess(bestgnsspos_msg, measure)) {
    integ_process_->MeasureDataProcess(measure);
    // LocalOnlineVisualizer::instance()->push_gnss_measure(measure);

    LocalizationEstimate gnss_localization;
    TransferGnssMeasureToLocalization(measure, gnss_localization);

    gnss_localization_mutex_.lock();
    // gnss_localization_state_ = LocalizationMeasureState::OK;
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
    const MeasureData& measure, LocalizationEstimate& localization) {
  apollo::common::Header* headerpb = localization.mutable_header();
  apollo::localization::Pose* posepb = localization.mutable_pose();

  double timestamp = measure.time;
  localization.set_measurement_time(timestamp);
  headerpb->set_timestamp_sec(timestamp);

  // get quat to trans 
  Eigen::Vector3d gnss_antenna_diff(gnss_antenna_extrinsic_.translation()(0),
      gnss_antenna_extrinsic_.translation()(1), gnss_antenna_extrinsic_.translation()(2));
  Eigen::Quaterniond quat(1, 0, 0, 0);
  if (!integ_pose_query_.query_quaternion(timestamp, quat)) {
    LOG(WARNING) << std::setprecision(20) << "Can't query integ pose, time: " << timestamp;
  }
  gnss_antenna_diff = quat * gnss_antenna_diff;

  UTMCoor utm_xy;
  latlon_to_utmxy(measure.gnss_pos.longitude, measure.gnss_pos.latitude, &utm_xy);

  apollo::common::PointENU* position = posepb->mutable_position();
  position->set_x(utm_xy.x - gnss_antenna_diff[0]);
  position->set_y(utm_xy.y - gnss_antenna_diff[1]);
  position->set_z(measure.gnss_pos.height - gnss_antenna_diff[2]);

  apollo::common::Quaternion* quaternion = posepb->mutable_orientation();
  quaternion->set_qx(quat.x());
  quaternion->set_qy(quat.y());
  quaternion->set_qz(quat.z());
  quaternion->set_qw(quat.w());

  apollo::localization::Uncertainty* uncertainty =
      localization.mutable_uncertainty();

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
    LocalizationMeasureState& state, LocalizationEstimate& lidar_localization) {
  lidar_localization_mutex_.lock();

  if (lidar_localization_list_.size()) {
    state = lidar_localization_list_.front().state();
    lidar_localization = lidar_localization_list_.front().localization();
    lidar_localization_list_.clear();
  } else {
    state = LocalizationMeasureState::NOT_VALID;
  }
  // state = lidar_localization_state_;
  // lidar_localization = lidar_localization_;
  lidar_localization_mutex_.unlock();
  return;
}

void LocalizationIntegImpl::GetLastestIntegLocalization(
    LocalizationMeasureState& state, LocalizationEstimate& integ_localization) {
  integ_localization_mutex_.lock();

  if (integ_localization_list_.size()) {
    state = integ_localization_list_.front().state();
    integ_localization = integ_localization_list_.front().localization();
    integ_localization_list_.clear();
  } else {
    state = LocalizationMeasureState::NOT_VALID;
  }
  // state = integ_localization_state_;
  // sins_pva = integ_sins_pva_;  //IntegSinsPva(integ_sins_pva_);
  // integ_localization = integ_localization_;
  // //LocalizationEstimate(integ_localization_);
  integ_localization_mutex_.unlock();
  return;
}

void LocalizationIntegImpl::GetLastestGnssLocalization(
    LocalizationMeasureState& state, LocalizationEstimate& gnss_localization) {
  gnss_localization_mutex_.lock();

  if (gnss_localization_list_.size()) {
    state = gnss_localization_list_.front().state();
    gnss_localization = gnss_localization_list_.front().localization();
    gnss_localization_list_.clear();
  } else {
    state = LocalizationMeasureState::NOT_VALID;
  }
  // state = gnss_localization_state_;
  // gnss_localization = gnss_localization_;
  gnss_localization_mutex_.unlock();
  return;
}

void LocalizationIntegImpl::GetLidarLocalizationList(
    std::list<LocalizationResult>& results) {
  lidar_localization_mutex_.lock();
  results = lidar_localization_list_;
  lidar_localization_list_.clear();
  lidar_localization_mutex_.unlock();
}

void LocalizationIntegImpl::GetIntegLocalizationList(
    std::list<LocalizationResult>& results) {
  integ_localization_mutex_.lock();
  results = integ_localization_list_;
  integ_localization_list_.clear();
  integ_localization_mutex_.unlock();
}

void LocalizationIntegImpl::GetGnssLocalizationList(
    std::list<LocalizationResult>& results) {
  gnss_localization_mutex_.lock();
  results = gnss_localization_list_;
  gnss_localization_list_.clear();
  gnss_localization_mutex_.unlock();
}

// void LocalizationIntegImpl::ParseLidarFrame(
//     const sensor_msgs::PointCloud2& lidar_data, LidarFrame& lidar_frame) const {
//   int total = lidar_data.width * lidar_data.height;
//   int x_offset = -1;
//   int y_offset = -1;
//   int z_offset = -1;
//   int t_offset = -1;
//   int i_offset = -1;
//   int8_t x_datatype;
//   int8_t y_datatype;
//   int8_t z_datatype;
//   int x_count = 0;
//   int y_count = 0;
//   int z_count = 0;
//   for (std::size_t i = 0; i < lidar_data.fields.size(); ++i) {
//     const sensor_msgs::PointField& f = lidar_data.fields[i];
//     if (f.name == "x") {
//       x_offset = f.offset;
//       x_datatype = f.datatype;
//       x_count = f.count;
//     } else if (f.name == "y") {
//       y_offset = f.offset;
//       y_datatype = f.datatype;
//       y_count = f.count;
//     } else if (f.name == "z") {
//       z_offset = f.offset;
//       z_datatype = f.datatype;
//       z_count = f.count;
//     } else if (f.name == "timestamp") {
//       t_offset = f.offset;
//     } else if (f.name == "intensity") {
//       i_offset = f.offset;
//     }
//   }
//   assert(x_offset != -1 && y_offset != -1 && z_offset != -1 && t_offset != -1);
//   assert(x_datatype == y_datatype && y_datatype == z_datatype);
//   assert(x_datatype == 7 || x_datatype == 8);
//   assert(x_count == 1 && y_count == 1 && z_count == 1);

//   int num_cached = 0;
//   std::map<double, TransformD> transform_cache;
//   if (lidar_data.height > 1 && lidar_data.width > 1) {
//     if (x_datatype == sensor_msgs::PointField::FLOAT32) {
//       for (int i = 0; i < lidar_data.height; ++i) {
//         for (int j = 0; j < lidar_data.width; j += 2) {
//           int index = i * lidar_data.width + j;
//           Vector3D pt3d;
//           int offset = index * lidar_data.point_step;
//           pt3d[0] = static_cast<const double>(*reinterpret_cast<const float*>(
//               &lidar_data.data[offset + x_offset]));
//           pt3d[1] = static_cast<const double>(*reinterpret_cast<const float*>(
//               &lidar_data.data[offset + y_offset]));
//           pt3d[2] = static_cast<const double>(*reinterpret_cast<const float*>(
//               &lidar_data.data[offset + z_offset]));
//           if (!std::isnan(pt3d[0])) {
//             unsigned char intensity = *reinterpret_cast<const unsigned char*>(
//                 &lidar_data.data[offset + i_offset]);
//             lidar_frame.pt3ds.push_back(pt3d);
//             lidar_frame.intensities.push_back(intensity);
//             lidar_frame.laser_ids.push_back(j);
//           }
//         }
//       }
//     } else if (x_datatype == sensor_msgs::PointField::FLOAT64) {
//       for (int i = 0; i < lidar_data.height; ++i) {
//         for (int j = 0; j < lidar_data.width; j += 2) {
//           int index = i * lidar_data.width + j;
//           Vector3D pt3d;
//           int offset = index * lidar_data.point_step;
//           pt3d[0] = *reinterpret_cast<const double*>(
//               &lidar_data.data[offset + x_offset]);
//           pt3d[1] = *reinterpret_cast<const double*>(
//               &lidar_data.data[offset + y_offset]);
//           pt3d[2] = *reinterpret_cast<const double*>(
//               &lidar_data.data[offset + z_offset]);
//           if (!std::isnan(pt3d[0])) {
//             unsigned char intensity = *reinterpret_cast<const unsigned char*>(
//                 &lidar_data.data[offset + i_offset]);
//             lidar_frame.pt3ds.push_back(pt3d);
//             lidar_frame.intensities.push_back(intensity);
//             lidar_frame.laser_ids.push_back(j);
//           }
//         }
//       }
//     } else {
//       LOG(ERROR) << "The point cloud data type is not right!";
//     }
//   } else {
//     LOG(INFO) << "Receiving un-origanized-point-cloud, width "
//               << lidar_data.width << " height " << lidar_data.height;
//     if (x_datatype == sensor_msgs::PointField::FLOAT32) {
//       for (int i = 0; i < lidar_data.height; ++i) {
//         for (int j = 0; j < lidar_data.width; j += 2) {
//           int index = i * lidar_data.width + j;
//           Vector3D pt3d;
//           int offset = index * lidar_data.point_step;
//           pt3d[0] = static_cast<const double>(*reinterpret_cast<const float*>(
//               &lidar_data.data[offset + x_offset]));
//           pt3d[1] = static_cast<const double>(*reinterpret_cast<const float*>(
//               &lidar_data.data[offset + y_offset]));
//           pt3d[2] = static_cast<const double>(*reinterpret_cast<const float*>(
//               &lidar_data.data[offset + z_offset]));
//           if (!std::isnan(pt3d[0])) {
//             unsigned char intensity = *reinterpret_cast<const unsigned char*>(
//                 &lidar_data.data[offset + i_offset]);
//             lidar_frame.pt3ds.push_back(pt3d);
//             lidar_frame.intensities.push_back(intensity);
//             lidar_frame.laser_ids.push_back(j);
//           }
//         }
//       }
//     } else if (x_datatype == sensor_msgs::PointField::FLOAT64) {
//       for (int i = 0; i < lidar_data.height; ++i) {
//         for (int j = 0; j < lidar_data.width; j += 2) {
//           int index = i * lidar_data.width + j;
//           Vector3D pt3d;
//           int offset = index * lidar_data.point_step;
//           pt3d[0] = *reinterpret_cast<const double*>(
//               &lidar_data.data[offset + x_offset]);
//           pt3d[1] = *reinterpret_cast<const double*>(
//               &lidar_data.data[offset + y_offset]);
//           pt3d[2] = *reinterpret_cast<const double*>(
//               &lidar_data.data[offset + z_offset]);
//           if (!std::isnan(pt3d[0])) {
//             unsigned char intensity = *reinterpret_cast<const unsigned char*>(
//                 &lidar_data.data[offset + i_offset]);
//             lidar_frame.pt3ds.push_back(pt3d);
//             lidar_frame.intensities.push_back(intensity);
//             lidar_frame.laser_ids.push_back(j);
//           }
//         }
//       }
//     } else {
//       LOG(ERROR) << "The point cloud data type is not right!";
//     }
//   }
//   lidar_frame.time = lidar_data.header.stamp.toSec();
//   if (debug_log_flag_) {
//     LOG(INFO) << std::setprecision(16)
//               << "Localization10Hz Debug Log: lidar msg. "
//               << "[time:" << lidar_frame.time << "]"
//               << "[height:" << lidar_data.height << "]"
//               << "[width:" << lidar_data.width << "]"
//               << "[point_step:" << lidar_data.point_step << "]"
//               << "[data_type:" << x_datatype << "]"
//               << "[point_cnt:"
//               << static_cast<unsigned int>(lidar_frame.pt3ds.size()) << "]"
//               << "[intensity_cnt:"
//               << static_cast<unsigned int>(lidar_frame.intensities.size())
//               << "]";
//     // if (lidar_frame.pt3ds.size() > 0) {
//     //   LOG(INFO) << "Localization10Hz Debug Log: lidar msg first point info. "
//     //             << "[x:" << lidar_frame.pt3ds[0][0] << "]"
//     //             << "[y:" << lidar_frame.pt3ds[0][1] << "]"
//     //             << "[z:" << lidar_frame.pt3ds[0][2] << "]";
//     // }
//   }
// }

}  // namespace msf
}  // namespace localization
}  // namespace apollo
