#include "localization_lidar_process.h"
#include <yaml-cpp/yaml.h>
#include "glog/logging.h"
#include "glog/raw_logging.h"
#include "math_util.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/localization/msf/common/util/timer.h"
#include "modules/localization/msf/common/util/sleep.h"

namespace apollo {
namespace localization {
namespace msf {

LocalizationLidarProcess::LocalizationLidarProcess() {
  wheelspeed_state_ = WheelspeedState::NOT_INIT;
  imu_state_ = ImuState::NOT_INIT;
  // predict_location_state_ = PredictLocationState::NOT_VALID;
  inspva_state_ = INSPVAState::NOT_INIT;
  // imu_rate_ = 1.0;

  lidar_filter_size_ = 11;
  lidar_thread_number_ = 2;
  out_map_count_ = 0;
  reinit_flag_ = false;

  lidar_status_ = LidarState::NOT_VALID;
  yaw_align_mode_ = 0;

  forcast_integ_state_ = ForcastState::NOT_VALID;
  forcast_timer_ = -1;
  delta_yaw_limit_ = 0.25 * 3.14159 / 180.0;
  init_delta_yaw_limit_ = 1.5 * 3.14159 / 180.0;
  compensate_pitch_roll_limit_ = 0.035; //tan(2)

  locator_ = new LocalizationLidar();
  pose_forcastor_ = new PoseForcast();

  lidar_extrinsic_ = TransformD::Identity();

  is_pre_state_init_ = false;
  cur_predict_location_ = TransformD::Identity();
  pre_predict_location_ = TransformD::Identity();
  pre_location_ = TransformD::Identity();
  velocity_ = Vector3D::Zero();
  location_ = TransformD::Identity();
  location_covariance_ = Matrix3D::Zero();

  non_zero_odometry_cnt_ = 0;
  max_nan_zero_odemetry_ = 10;

  pthread_mutex_init(&pva_mutex_, NULL);
  pthread_mutex_init(&imu_mutex_, NULL);
}

LocalizationLidarProcess::~LocalizationLidarProcess() {
  pthread_mutex_destroy(&pva_mutex_);
  pthread_mutex_destroy(&imu_mutex_);
  // pthread_mutex_destroy(&wheelspeed_mutex_);

  delete locator_;
  locator_ = NULL;

  delete pose_forcastor_;
  pose_forcastor_ = NULL;
}

LocalizationState LocalizationLidarProcess::Init(
    const LocalizationIntegParam& params) {
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
  // imu_rate_ = params.imu_rate;

  lidar_filter_size_ = params.lidar_filter_size;
  lidar_thread_number_ = params.lidar_thread_num;

  lidar_status_ = LidarState::NOT_VALID;

  // reload_map_flag_ = false;
  reinit_flag_ = false;
  // initial_success_ = true;
  non_zero_odometry_cnt_ = 0;
  max_nan_zero_odemetry_ = 10;

  // buffer
  out_map_count_ = 0;
  imu_buffer_size_ = 200;
  pva_buffer_size_ = 200;

  is_pre_state_init_ = false;
  cur_predict_location_ = TransformD::Identity();
  pre_predict_location_ = TransformD::Identity();
  pre_location_ = TransformD::Identity();
  velocity_ = Vector3D::Zero();
  location_ = TransformD::Identity();
  location_covariance_ = Matrix3D::Zero();

  bool sucess = LoadLidarExtrinsic(lidar_extrinsic_file_, lidar_extrinsic_);
  if (!sucess) {
    LOG(ERROR) << "LocalizationLidar: Fail to access the lidar"
                  " extrinsic file: "
               << lidar_extrinsic_file_;
    return LocalizationState(LocalizationErrorCode::LIDAR_ERROR,
                             "Fail to access the lidar extrinsic file");
  }

  sucess = LoadLidarHeight(lidar_height_file_, lidar_height_);
  if (!sucess) {
    LOG(WARNING) << "LocalizationLidar: Fail to load the lidar"
                    " height file: "
                 << lidar_height_file_ << " Will use default value!";
    lidar_height_.height = params.lidar_height_default;
  }

  if (!locator_->Init(map_path_, lidar_filter_size_,
                     lidar_filter_size_, utm_zone_id_)) {
    return LocalizationState(LocalizationErrorCode::LIDAR_ERROR,
                             "Fail to load localization map!");
  }

  locator_->SetVelodyneExtrinsic(lidar_extrinsic_);
  locator_->SetLocalizationMode(localization_mode_);
  locator_->SetImageAlignMode(yaw_align_mode_);
  locator_->SetValidThreshold(map_coverage_theshold_);
  locator_->SetVehicleHeight(lidar_height_.height);
  locator_->SetDeltaPitchRollLimit(compensate_pitch_roll_limit_);

  pose_forcastor_->SetMaxListNum(400);
  pose_forcastor_->SetMaxAccelInput(5.0);
  pose_forcastor_->SetMaxGyroInput(200 * 0.017453292519943);
  pose_forcastor_->SetZoneId(utm_zone_id_);

  return LocalizationState::OK();
}

double LocalizationLidarProcess::ComputeDeltaYaw(
      long long index_cur, long long index_stable, 
      double limit_min, double limit_max) {
  if (index_cur > index_stable) {
    return limit_min;
  }

  double ratio = (double)index_cur;
  ratio /= (double)index_stable;
  return limit_min * ratio + limit_max * (1.0 - ratio);
}

void LocalizationLidarProcess::PcdProcess(LidarFrame& lidar_frame) {
  if (!CheckState()) {
    return;
  }

  // pcd process cost time
  Timer timer;
  timer.Start();

  static unsigned int pcd_index = 0;

  if (!GetPredictPose(lidar_frame.measurement_time,
                      &cur_predict_location_,
                      &forcast_integ_state_)) {
    return;
  }

  if (forcast_integ_state_ != ForcastState::INCREMENT) {
      forcast_timer_ = -1;
  }
  ++forcast_timer_;

  locator_->SetDeltaYawLimit(ComputeDeltaYaw(forcast_timer_, 10, 
                delta_yaw_limit_, init_delta_yaw_limit_));

  if (!is_pre_state_init_) {
    pre_predict_location_ = cur_predict_location_;
    pre_location_ = cur_predict_location_;
    velocity_ = Vector3D::Zero();
    is_pre_state_init_ = true;
  }

  velocity_ = cur_predict_location_.translation() - pre_location_.translation();

  int ret = -10;
  ret = locator_->Update(pcd_index++, cur_predict_location_, velocity_, lidar_frame);

  UpdateState(ret, lidar_frame.measurement_time);

  timer.End("Lidar process");
}

void LocalizationLidarProcess::GetResult(int& lidar_status,
                                         TransformD& location,
                                         Matrix3D& covariance) {
  lidar_status = int(lidar_status_);
  location = location_;
  covariance = location_covariance_;
}

int LocalizationLidarProcess::GetResult(LocalizationEstimate& lidar_local_msg) {
  apollo::common::Header* headerpb = lidar_local_msg.mutable_header();
  apollo::localization::Pose* posepb = lidar_local_msg.mutable_pose();
  lidar_local_msg.set_measurement_time(pre_location_time_);
  headerpb->set_timestamp_sec(pre_location_time_);

  apollo::common::PointENU* position = posepb->mutable_position();
  apollo::common::Quaternion* quaternion = posepb->mutable_orientation();
  position->set_x(location_.translation()(0));
  position->set_y(location_.translation()(1));
  position->set_z(location_.translation()(2));

  QuaternionD quat(location_.linear());
  quaternion->set_qx(quat.x());
  quaternion->set_qy(quat.y());
  quaternion->set_qz(quat.z());
  quaternion->set_qw(quat.w());

  apollo::localization::Uncertainty* uncertainty =
      lidar_local_msg.mutable_uncertainty();

  apollo::common::Point3D* position_std_dev =
      uncertainty->mutable_position_std_dev();
  position_std_dev->set_x(location_covariance_(0, 0));
  position_std_dev->set_y(location_covariance_(1, 1));
  position_std_dev->set_z(0.0);

  apollo::common::Point3D* orientation_std_dev =
      uncertainty->mutable_orientation_std_dev();
  orientation_std_dev->set_x(0.0);
  orientation_std_dev->set_y(0.0);
  orientation_std_dev->set_z(0.15 * 0.15 * 3.0461742e-04);

  return int(lidar_status_);
}

void LocalizationLidarProcess::IntegPvaProcess(
    const InsPva& sins_pva_msg) {
  const InsPva& pva_forecast = sins_pva_msg;
  pose_forcastor_->PushInspvaData(pva_forecast);

  return;
}

void LocalizationLidarProcess::RawImuProcess(const ImuData& imu_msg) {
  const ImuData& imu_data = imu_msg;
  pose_forcastor_->PushImuData(imu_data);

  return;
}

bool LocalizationLidarProcess::GetPredictPose(double lidar_time,
                                              TransformD *predict_pose,
                                              ForcastState *forcast_state) {
  double latest_imu_time = pose_forcastor_->GetLastestImuTime();
  if (latest_imu_time - lidar_time > imu_lidar_max_delay_time_) {
    LOG(ERROR) << std::setprecision(16) << "LocalizationLidar GetPredictPose: "
               << "Lidar msg too old! "
               << "lidar time: " << lidar_time
               << "delay time: " << latest_imu_time - lidar_time;
    return false;
  }
  
  Pose forcast_pose;
  int state = -1;
  if (lidar_status_ == LidarState::NOT_VALID) {
    Pose init_pose;
    state = pose_forcastor_->GetBestForcastPose(lidar_time, -1,
                                                init_pose, &forcast_pose);
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
    LOG(INFO) << "LocalizationLidar GetPredictPose: "
              << "Recive a lidar msg, but can't query predict pose.";
    *forcast_state = ForcastState::NOT_VALID;
    return false;
  }

  if (std::abs(forcast_pose.x) < 10.0 || std::abs((forcast_pose.y) < 10.0)) {
    LOG(ERROR) << "LocalizationLidar Fatal Error: invalid pose!";
    return false;
  }

  if (non_zero_odometry_cnt_ < max_nan_zero_odemetry_) {
    LOG(INFO) << "LocalizationLidar: Abort several initial lidar frame.";
    ++non_zero_odometry_cnt_;
    return false;
  }

  Eigen::Quaterniond quatd(forcast_pose.qw, forcast_pose.qx,
                           forcast_pose.qy, forcast_pose.qz);
  Eigen::Translation3d transd(
      Eigen::Vector3d(forcast_pose.x, forcast_pose.y, forcast_pose.z));
  *predict_pose = transd * quatd;

  if (state == 0) {
    *forcast_state = ForcastState::INITIAL;
  } else {
    *forcast_state = ForcastState::INCREMENT;
    LOG(INFO) << "The delta translation input lidar localization: "
              << lidar_time << " "
              << forcast_pose.x - pre_location_.translation()(0) << " "
              << forcast_pose.y - pre_location_.translation()(1) << " "
              << forcast_pose.z - pre_location_.translation()(2);
  }

  return true;
}

bool LocalizationLidarProcess::CheckState() {
  return true;
}

// bool LocalizationLidarProcess::CheckDelta(const LidarFrame& frame,
//                                           const TransformD& inspva_pose) {
//   if (pre_inspva_location_.translation()(0)) {
//     Vector3D delta_trans(inspva_pose.translation() -
//                          pre_inspva_location_.translation());
//     double delta_location = std::sqrt(delta_trans.squaredNorm());
//     double delta_time = frame.measurement_time - pre_inspva_timestamp_;
//     if (delta_time <= 0.01) {
//       LOG(ERROR) << "LocalizationLidar Fatal Error: \
//           receive two lidar frames less than 0.01 second!";
//       return false;
//     }
//     double velocity = delta_location / delta_time;
//     if (velocity > 100.0) {
//       LOG(ERROR) << "LocalizationLidar CheckDelta: \
//           detect inspva pose jumped, velocity larger than 360km/h!";
//       return false;
//     } else if (debug_log_flag_) {
//       LOG(INFO) << "LocalizationLidar Estimate velocity: " << velocity * 3.6
//                 << " km/h";
//     }
//   }
//   return true;
// }

void LocalizationLidarProcess::UpdateState(int ret, double time) {
  if (ret == 0) { // OK
    locator_->GetResult(&location_, &location_covariance_);
    lidar_status_ = LidarState::OK;
    pre_location_ = location_;
    pre_location_time_ = time;
    if (out_map_count_ > 0) {
      --out_map_count_;
    }
  } else if (ret == -2) { // out of map
    locator_->GetResult(&location_, &location_covariance_);
    lidar_status_ = LidarState::NOT_STABLE;
    pre_location_ = location_;
    pre_location_time_ = time;
    if (out_map_count_ < 10) {
      ++out_map_count_;
    } else {
      reinit_flag_ = true;
    }
  } else { // NOT_VALID
    LOG(ERROR) << "LocalizationLidar: The reflection map load failed!";
    lidar_status_ = LidarState::NOT_VALID;
  }
}

bool LocalizationLidarProcess::LoadLidarExtrinsic(const std::string& file_path,
                                                  TransformD& lidar_extrinsic) {
  YAML::Node config = YAML::LoadFile(file_path);
  if (config["transform"]) {
    if (config["transform"]["translation"]) {
      lidar_extrinsic.translation()(0) =
          config["transform"]["translation"]["x"].as<double>();
      lidar_extrinsic.translation()(1) =
          config["transform"]["translation"]["y"].as<double>();
      lidar_extrinsic.translation()(2) =
          config["transform"]["translation"]["z"].as<double>();
      if (config["transform"]["rotation"]) {
        double qx = config["transform"]["rotation"]["x"].as<double>();
        double qy = config["transform"]["rotation"]["y"].as<double>();
        double qz = config["transform"]["rotation"]["z"].as<double>();
        double qw = config["transform"]["rotation"]["w"].as<double>();
        // lidar_extrinsic.quaternion() = QuaternionD(qx, qy, qz, qw);
        lidar_extrinsic.linear() =
            Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
        return true;
      }
    }
  }
  return false;
}

bool LocalizationLidarProcess::LoadLidarHeight(const std::string& file_path,
                                               LidarHeight& height) {
  if(!common::util::PathExists(file_path)) {
    return false;
  }

  YAML::Node config = YAML::LoadFile(file_path);
  if (config["vehicle"]) {
    if (config["vehicle"]["parameters"]) {
      height.height = config["vehicle"]["parameters"]["height"].as<double>();
      height.height_var =
          config["vehicle"]["parameters"]["height_var"].as<double>();
      return true;
    }
  }
  return false;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
