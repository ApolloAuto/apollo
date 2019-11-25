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

#include "modules/localization/msf/local_integ/measure_republish_process.h"

#include <fstream>

#include "cyber/common/log.h"
#include "modules/common/math/euler_angles_zxy.h"
#include "modules/common/time/time_util.h"
#include "modules/localization/msf/common/util/math_util.h"
#include "modules/localization/msf/common/util/time_conversion.h"
#include "yaml-cpp/yaml.h"

namespace apollo {
namespace localization {
namespace msf {

using common::Status;
using common::time::TimeUtil;

MeasureRepublishProcess::MeasureRepublishProcess()
    : pre_bestgnsspose_(),
      pre_bestgnsspose_valid_(false),
      send_init_bestgnsspose_(false),
      pva_buffer_size_(150),
      local_utm_zone_id_(50),
      is_trans_gpstime_to_utctime_(true),
      map_height_time_(0.0),
      gnss_mode_(GnssMode::NOVATEL) {}

MeasureRepublishProcess::~MeasureRepublishProcess() {}

Status MeasureRepublishProcess::Init(const LocalizationIntegParam& params) {
  local_utm_zone_id_ = params.utm_zone_id;
  is_trans_gpstime_to_utctime_ = params.is_trans_gpstime_to_utctime;
  gnss_mode_ = GnssMode(params.gnss_mode);

  pva_buffer_size_ = 150;

  map_height_time_ = 0.0;

  novatel_heading_time_ = 0.0;

  std::ifstream imu_ant_fin(params.ant_imu_leverarm_file.c_str());
  AINFO << "the ant_imu_leverarm file: " << params.ant_imu_leverarm_file.c_str()
        << std::endl;
  if (imu_ant_fin) {
    bool success = LoadImuGnssAntennaExtrinsic(params.ant_imu_leverarm_file,
                                               &imu_gnssant_extrinsic_);
    if (!success) {
      AERROR << "IntegratedLocalization: Fail to access the lever arm "
                "between imu and gnss extrinsic file: "
             << params.ant_imu_leverarm_file;
    }
    AINFO << "gnss and imu lever arm in vehicle frame: "
          << " " << imu_gnssant_extrinsic_.ant_num << " "
          << imu_gnssant_extrinsic_.transform_1.translation()[0] << " "
          << imu_gnssant_extrinsic_.transform_1.translation()[1] << " "
          << imu_gnssant_extrinsic_.transform_1.translation()[2] << " "
          << imu_gnssant_extrinsic_.transform_2.translation()[0] << " "
          << imu_gnssant_extrinsic_.transform_2.translation()[1] << " "
          << imu_gnssant_extrinsic_.transform_2.translation()[2];
  } else {
    AINFO << "the ant_imu_leverarm_file does not existence!";
  }

  double vehicle_to_imu_quatern[4] = {
      params.vehicle_to_imu_quatern.w, params.vehicle_to_imu_quatern.x,
      params.vehicle_to_imu_quatern.y, params.vehicle_to_imu_quatern.z};

  double dcm[3][3] = {0.0};
  math::QuaternionToDcm(&vehicle_to_imu_quatern[0], dcm);
  double lever_arm_x = imu_gnssant_extrinsic_.transform_1.translation()[0];
  double lever_arm_y = imu_gnssant_extrinsic_.transform_1.translation()[1];
  double lever_arm_z = imu_gnssant_extrinsic_.transform_1.translation()[2];

  imu_gnssant_extrinsic_.transform_1.translation()[0] =
      dcm[0][0] * lever_arm_x + dcm[0][1] * lever_arm_y +
      dcm[0][2] * lever_arm_z;
  imu_gnssant_extrinsic_.transform_1.translation()[1] =
      dcm[1][0] * lever_arm_x + dcm[1][1] * lever_arm_y +
      dcm[1][2] * lever_arm_z;
  imu_gnssant_extrinsic_.transform_1.translation()[2] =
      dcm[2][0] * lever_arm_x + dcm[2][1] * lever_arm_y +
      dcm[2][2] * lever_arm_z;

  lever_arm_x = imu_gnssant_extrinsic_.transform_2.translation()[0];
  lever_arm_y = imu_gnssant_extrinsic_.transform_2.translation()[1];
  lever_arm_z = imu_gnssant_extrinsic_.transform_2.translation()[2];

  imu_gnssant_extrinsic_.transform_2.translation()[0] =
      dcm[0][0] * lever_arm_x + dcm[0][1] * lever_arm_y +
      dcm[0][2] * lever_arm_z;
  imu_gnssant_extrinsic_.transform_2.translation()[1] =
      dcm[1][0] * lever_arm_x + dcm[1][1] * lever_arm_y +
      dcm[1][2] * lever_arm_z;
  imu_gnssant_extrinsic_.transform_2.translation()[2] =
      dcm[2][0] * lever_arm_x + dcm[2][1] * lever_arm_y +
      dcm[2][2] * lever_arm_z;

  AINFO << "gnss and imu lever arm in imu frame: "
        << " " << imu_gnssant_extrinsic_.ant_num << " "
        << imu_gnssant_extrinsic_.transform_1.translation()[0] << " "
        << imu_gnssant_extrinsic_.transform_1.translation()[1] << " "
        << imu_gnssant_extrinsic_.transform_1.translation()[2] << " "
        << imu_gnssant_extrinsic_.transform_2.translation()[0] << " "
        << imu_gnssant_extrinsic_.transform_2.translation()[1] << " "
        << imu_gnssant_extrinsic_.transform_2.translation()[2];

  is_using_novatel_heading_ = params.is_using_novatel_heading;
  if (imu_gnssant_extrinsic_.ant_num == 1) {
    is_using_novatel_heading_ = false;
  }
  return Status::OK();
}

bool MeasureRepublishProcess::NovatelBestgnssposProcess(
    const GnssBestPose& bestgnsspos_msg, MeasureData* measure) {
  if (gnss_mode_ != GnssMode::NOVATEL) {
    return false;
  }
  CHECK_NOTNULL(measure);

  if (!CheckBestgnssposeStatus(bestgnsspos_msg)) {
    AWARN << "Discard a bestgnsspose msg. "
          << "The status of this msg is not OK.";
    return false;
  }

  // check sins status
  bool is_sins_align = IsSinsAlign();

  // If sins is align, we only need measure of xyz from bestgnsspos.
  // If sins is not align, in order to init sins, we need
  // (1) send an initial measure of xyz; (2) send measure of xyz and velocity.
  if (is_sins_align) {
    TransferXYZFromBestgnsspose(bestgnsspos_msg, measure);
  } else {
    if (!CheckBestgnssPoseXYStd(bestgnsspos_msg)) {
      AWARN << "Discard a bestgnsspose msg for large std.";
      return false;
    }

    if (!send_init_bestgnsspose_) {
      TransferFirstMeasureFromBestgnsspose(bestgnsspos_msg, measure);
      send_init_bestgnsspose_ = true;
    } else {
      if (!CalculateVelFromBestgnsspose(bestgnsspos_msg, measure)) {
        AINFO << "Waiting calculate velocity successfully...";
        return false;
      }
    }
  }

  ADEBUG << std::setprecision(16)
         << "MeasureDataRepublish Debug Log: bestgnsspos msg: "
         << "[time:" << measure->time << "]"
         << "[x:" << measure->gnss_pos.longitude * RAD_TO_DEG << "]"
         << "[y:" << measure->gnss_pos.latitude * RAD_TO_DEG << "]"
         << "[z:" << measure->gnss_pos.height << "]"
         << "[std_x:" << bestgnsspos_msg.longitude_std_dev() << "]"
         << "[std_y:" << bestgnsspos_msg.latitude_std_dev() << "]"
         << "[std_z:" << bestgnsspos_msg.height_std_dev() << "]"
         << "[position_type:" << bestgnsspos_msg.sol_type() << "]";

  return true;
}

void MeasureRepublishProcess::GnssLocalProcess(
    const MeasureData& gnss_local_msg, MeasureData* measure) {
  if (gnss_mode_ != GnssMode::SELF) {
    return;
  }

  CHECK_NOTNULL(measure);

  MeasureData measure_data = gnss_local_msg;
  if (is_trans_gpstime_to_utctime_) {
    measure_data.time = TimeUtil::Gps2unix(measure_data.time);
  }

  AINFO << "the gnss velocity: " << measure_data.gnss_vel.ve << " "
        << measure_data.gnss_vel.vn << " " << measure_data.gnss_vel.vu;

  measure_data.gnss_att.pitch = 0.0;
  measure_data.gnss_att.roll = 0.0;

  Eigen::Vector3d pos_xyz = Eigen::Vector3d::Zero();
  pos_xyz[0] = measure_data.gnss_pos.longitude;
  pos_xyz[1] = measure_data.gnss_pos.latitude;
  pos_xyz[2] = measure_data.gnss_pos.height;

  Eigen::Vector3d pos_blh = Eigen::Vector3d::Zero();
  apollo::localization::msf::FrameTransform::XYZToBlh(pos_xyz, &pos_blh);
  measure_data.gnss_pos.longitude = pos_blh[0];
  measure_data.gnss_pos.latitude = pos_blh[1];
  measure_data.gnss_pos.height = pos_blh[2];

  double ve_std = std::sqrt(measure_data.variance[3][3]);
  double vn_std = std::sqrt(measure_data.variance[4][4]);
  double vu_std = std::sqrt(measure_data.variance[5][5]);
  AINFO << "the gnss velocity std: " << ve_std << " " << vn_std << " "
        << vu_std;

  bool is_sins_align = IsSinsAlign();

  if (is_sins_align) {
    measure_data.measure_type = MeasureType::GNSS_POS_ONLY;
    height_mutex_.lock();
    if ((measure_data.time - 1.0 < map_height_time_)) {
      measure_data.measure_type = MeasureType::GNSS_POS_XY;
    }
    height_mutex_.unlock();
    measure_data.is_have_variance = true;
  } else {
    measure_data.measure_type = MeasureType::GNSS_POS_VEL;
    measure_data.is_have_variance = false;

    static bool is_sent_init_pos = false;
    if (!is_sent_init_pos) {
      is_sent_init_pos = true;
      measure_data.gnss_vel.ve = 0.0;
      measure_data.gnss_vel.vn = 0.0;
      measure_data.gnss_vel.vu = 0.0;
      AINFO << "send sins init position using rtk-gnss position!";
      *measure = measure_data;
      return;
    }

    static int position_good_counter = 0;
    if (position_good_counter < 10) {
      ++position_good_counter;
      return;
    }

    if (gnss_local_msg.measure_type != MeasureType::GNSS_POS_VEL &&
        gnss_local_msg.measure_type != MeasureType::ENU_VEL_ONLY) {
      AERROR << "gnss does not have velocity,"
             << "the gnss velocity std: " << ve_std << " " << vn_std << " "
             << vu_std;
      return;
    }
    if (!gnss_local_msg.is_have_variance) {
      AERROR << "gnss velocity does not have velocity variance!";
      return;
    } else {
      if ((ve_std > 0.1) || (vn_std > 0.1)) {
        AWARN << "gnss velocity variance is large: " << ve_std << " " << vn_std;
        return;
      }
    }

    static double pre_yaw_from_vel = 0.0;
    static double pre_measure_time = 0.0;
    double yaw_from_vel =
        atan2(measure_data.gnss_vel.ve, measure_data.gnss_vel.vn);
    if (pre_measure_time < 0.1) {
      pre_measure_time = measure_data.time;
      pre_yaw_from_vel = yaw_from_vel;
      return;
    } else {
      static constexpr double rad_round = 2 * M_PI;
      static constexpr double rad_pi = M_PI;

      double delta_yaw = yaw_from_vel - pre_yaw_from_vel;
      if (delta_yaw > rad_pi) {
        delta_yaw = delta_yaw - rad_round;
      }
      if (delta_yaw < -rad_pi) {
        delta_yaw = delta_yaw + rad_round;
      }

      AINFO << "yaw from position difference: " << yaw_from_vel * RAD_TO_DEG;
      double delta_time = measure_data.time - pre_measure_time;
      if (delta_time < 1.0e-10) {
        AINFO << "the delta time is too small: " << delta_time;
      }
      double yaw_incr = delta_yaw / delta_time;
      // 0.0872rad = 5deg
      static constexpr double rad_5deg = 5 * DEG_TO_RAD;
      if ((yaw_incr > rad_5deg) || (yaw_incr < -rad_5deg)) {
        AWARN << "yaw velocity is large! pre, "
              << "cur yaw from vel and velocity: "
              << pre_yaw_from_vel * RAD_TO_DEG << " "
              << yaw_from_vel * RAD_TO_DEG << " " << yaw_incr * RAD_TO_DEG;
        pre_measure_time = measure_data.time;
        pre_yaw_from_vel = yaw_from_vel;
        return;
      }
      pre_measure_time = measure_data.time;
      pre_yaw_from_vel = yaw_from_vel;
    }
  }
  *measure = measure_data;

  ADEBUG << std::setprecision(16)
         << "MeasureDataRepublish Debug Log: rtkgnss msg: "
         << "[time:" << measure_data.time << "]"
         << "[x:" << measure_data.gnss_pos.longitude * RAD_TO_DEG << "]"
         << "[y:" << measure_data.gnss_pos.latitude * RAD_TO_DEG << "]"
         << "[z:" << measure_data.gnss_pos.height << "]"
         << "[std_x:" << measure_data.variance[0][0] << "]"
         << "[std_y:" << measure_data.variance[1][1] << "]"
         << "[std_z:" << measure_data.variance[2][2] << "]";
}

void MeasureRepublishProcess::IntegPvaProcess(const InsPva& inspva_msg) {
  const InsPva& integ_pva = inspva_msg;

  std::lock_guard<std::mutex> lock(integ_pva_mutex_);
  if (integ_pva_list_.size() < pva_buffer_size_) {
    integ_pva_list_.push_back(integ_pva);
  } else {
    integ_pva_list_.pop_front();
    integ_pva_list_.push_back(integ_pva);
  }
}

bool MeasureRepublishProcess::LidarLocalProcess(
    const LocalizationEstimate& lidar_local_msg, MeasureData* measure) {
  CHECK_NOTNULL(measure);

  MeasureData measure_data;
  measure_data.time = lidar_local_msg.measurement_time();

  apollo::localization::msf::WGS84Corr temp_wgs;
  apollo::localization::msf::FrameTransform::UtmXYToLatlon(
      lidar_local_msg.pose().position().x(),
      lidar_local_msg.pose().position().y(), local_utm_zone_id_, false,
      &temp_wgs);
  measure_data.gnss_pos.longitude = temp_wgs.log;
  measure_data.gnss_pos.latitude = temp_wgs.lat;
  measure_data.gnss_pos.height = lidar_local_msg.pose().position().z();

  Eigen::Quaterniond temp_quat(lidar_local_msg.pose().orientation().qw(),
                               lidar_local_msg.pose().orientation().qx(),
                               lidar_local_msg.pose().orientation().qy(),
                               lidar_local_msg.pose().orientation().qz());

  common::math::EulerAnglesZXYd euler(temp_quat.w(), temp_quat.x(),
                                      temp_quat.y(), temp_quat.z());

  height_mutex_.lock();
  Eigen::Vector3d trans(lidar_local_msg.pose().position().x(),
                        lidar_local_msg.pose().position().y(),
                        lidar_local_msg.pose().position().z());

  map_height_time_ = measure_data.time;
  height_mutex_.unlock();

  measure_data.gnss_att.yaw = euler.yaw();
  measure_data.measure_type = MeasureType::POINT_CLOUD_POS;
  measure_data.frame_type = FrameType::UTM;

  measure_data.is_have_variance = true;

  double longitude_var = lidar_local_msg.uncertainty().position_std_dev().x();
  double latitude_var = lidar_local_msg.uncertainty().position_std_dev().y();
  double yaw_var = lidar_local_msg.uncertainty().orientation_std_dev().z();

  static constexpr double height_var = 0.03 * 0.03;
  measure_data.variance[0][0] = longitude_var;
  measure_data.variance[1][1] = latitude_var;
  measure_data.variance[2][2] = height_var;
  measure_data.variance[8][8] = yaw_var;

  *measure = measure_data;

  ADEBUG << std::setprecision(16)
         << "MeasureDataRepublish Debug Log: lidarLocal msg: "
         << "[time:" << measure_data.time << "]"
         << "[x:" << measure_data.gnss_pos.longitude * RAD_TO_DEG << "]"
         << "[y:" << measure_data.gnss_pos.latitude * RAD_TO_DEG << "]"
         << "[z:" << measure_data.gnss_pos.height << "]"
         << "[yaw:" << measure_data.gnss_att.yaw * RAD_TO_DEG << "]"
         << "[std_x:" << std::sqrt(longitude_var) << "]"
         << "[std_y:" << std::sqrt(latitude_var) << "]"
         << "[std_z:" << std::sqrt(height_var) << "]";

  return true;
}

bool MeasureRepublishProcess::IsSinsAlign() {
  std::lock_guard<std::mutex> lock(integ_pva_mutex_);
  return !integ_pva_list_.empty() && integ_pva_list_.back().init_and_alignment;
}

void MeasureRepublishProcess::TransferXYZFromBestgnsspose(
    const GnssBestPose& bestgnsspos_msg, MeasureData* measure) {
  CHECK_NOTNULL(measure);

  measure->time = bestgnsspos_msg.measurement_time();
  if (is_trans_gpstime_to_utctime_) {
    measure->time = TimeUtil::Gps2unix(measure->time);
  }

  measure->gnss_pos.longitude = bestgnsspos_msg.longitude() * DEG_TO_RAD;
  measure->gnss_pos.latitude = bestgnsspos_msg.latitude() * DEG_TO_RAD;
  measure->gnss_pos.height =
      bestgnsspos_msg.height_msl() + bestgnsspos_msg.undulation();

  measure->variance[0][0] =
      bestgnsspos_msg.longitude_std_dev() * bestgnsspos_msg.longitude_std_dev();
  measure->variance[1][1] =
      bestgnsspos_msg.latitude_std_dev() * bestgnsspos_msg.latitude_std_dev();
  measure->variance[2][2] =
      bestgnsspos_msg.height_std_dev() * bestgnsspos_msg.height_std_dev();

  measure->measure_type = MeasureType::GNSS_POS_ONLY;
  measure->frame_type = FrameType::ENU;
  height_mutex_.lock();
  if ((measure->time - 1.0 < map_height_time_)) {
    measure->measure_type = MeasureType::GNSS_POS_XY;
  }
  height_mutex_.unlock();
  measure->is_have_variance = true;
}

void MeasureRepublishProcess::TransferFirstMeasureFromBestgnsspose(
    const GnssBestPose& bestgnsspos_msg, MeasureData* measure) {
  CHECK_NOTNULL(measure);

  TransferXYZFromBestgnsspose(bestgnsspos_msg, measure);

  measure->measure_type = MeasureType::GNSS_POS_VEL;
  measure->frame_type = FrameType::ENU;
  measure->is_have_variance = false;

  measure->gnss_vel.ve = 0.0;
  measure->gnss_vel.vn = 0.0;
  measure->gnss_vel.vu = 0.0;
  AINFO << "Novatel bestgnsspose publish: "
        << "send sins init position using novatel bestgnsspos!";
}

bool MeasureRepublishProcess::CalculateVelFromBestgnsspose(
    const GnssBestPose& bestgnsspos_msg, MeasureData* measure) {
  CHECK_NOTNULL(measure);

  TransferXYZFromBestgnsspose(bestgnsspos_msg, measure);

  measure->measure_type = MeasureType::GNSS_POS_VEL;
  measure->frame_type = FrameType::ENU;
  measure->is_have_variance = false;

  if (!pre_bestgnsspose_valid_) {
    pre_bestgnsspose_valid_ = true;
    pre_bestgnsspose_ = *measure;
    return false;
  }

  static int position_good_counter = 0;
  if (position_good_counter < BESTPOSE_GOOD_COUNT) {
    ++position_good_counter;
    pre_bestgnsspose_ = *measure;
    return false;
  }

  const double re = 6378137.0;
  const double e = 0.0033528;
  double temp_sin_lati_2 =
      sin(measure->gnss_pos.latitude) * sin(measure->gnss_pos.latitude);
  double rn = re / (1 - e * temp_sin_lati_2);
  double rm = re / (1 + 2.0 * e - 3.0 * e * temp_sin_lati_2);

  double inv_time = 1.0 / (measure->time - pre_bestgnsspose_.time);
  if (measure->time - pre_bestgnsspose_.time > BESTPOSE_TIME_MAX_INTERVAL) {
    pre_bestgnsspose_ = *measure;
    position_good_counter = 0;
    return false;
  }
  measure->gnss_vel.ve =
      (measure->gnss_pos.longitude - pre_bestgnsspose_.gnss_pos.longitude) *
      rn * inv_time * cos(measure->gnss_pos.latitude);
  measure->gnss_vel.vn =
      (measure->gnss_pos.latitude - pre_bestgnsspose_.gnss_pos.latitude) * rm *
      inv_time;
  measure->gnss_vel.vu =
      (measure->gnss_pos.height - pre_bestgnsspose_.gnss_pos.height) * inv_time;

  pre_bestgnsspose_ = *measure;
  AINFO << "novatel bestgnsspos velocity: " << measure->gnss_vel.ve << " "
        << measure->gnss_vel.vn << " " << measure->gnss_vel.vu;

  static double pre_yaw_from_vel = 0.0;
  double yaw_from_vel = atan2(measure->gnss_vel.ve, measure->gnss_vel.vn);
  if (!pre_yaw_from_vel) {
    pre_yaw_from_vel = yaw_from_vel;
    return false;
  } else {
    static constexpr double rad_round = 2 * M_PI;
    static constexpr double rad_pi = M_PI;

    double delta_yaw = yaw_from_vel - pre_yaw_from_vel;
    if (delta_yaw > rad_pi) {
      delta_yaw = delta_yaw - rad_round;
    }
    if (delta_yaw < -rad_pi) {
      delta_yaw = delta_yaw + rad_round;
    }

    AINFO << "yaw calculated from position difference: "
          << yaw_from_vel * RAD_TO_DEG;
    static constexpr double rad_5deg = 5 * DEG_TO_RAD;
    if (delta_yaw > rad_5deg || delta_yaw < -rad_5deg) {
      AWARN << "novatel bestgnsspos delta yaw is large! "
            << "pre, cur yaw from vel and delta: "
            << pre_yaw_from_vel * RAD_TO_DEG << " " << yaw_from_vel * RAD_TO_DEG
            << " " << delta_yaw * RAD_TO_DEG;
      pre_yaw_from_vel = yaw_from_vel;
      return false;
    }
    pre_yaw_from_vel = yaw_from_vel;
  }

  return true;
}

bool MeasureRepublishProcess::GnssHeadingProcess(
    const drivers::gnss::Heading& heading_msg, MeasureData* measure_data,
    int* status) {
  if ((imu_gnssant_extrinsic_.ant_num == 1)) {
    return false;
  }
  int solution_status = heading_msg.solution_status();
  int position_type = heading_msg.position_type();
  AINFO << "the heading solution_status and position_type: " << solution_status
        << " " << position_type;

  if (solution_status != 0) {
    *status = 93;
    AINFO << "the heading's solution_status is not computed: "
          << solution_status;
    return false;
  }
  *status = position_type;
  if ((position_type == 0) || (position_type == 1)) {
    AINFO << "the heading's solution_type is invalid or fixed: "
          << position_type;
    return false;
  }
  measure_data->time = heading_msg.measurement_time();
  if (is_trans_gpstime_to_utctime_) {
    measure_data->time = GpsToUnixSeconds(measure_data->time);
  }

  novatel_heading_mutex_.lock();
  novatel_heading_time_ = measure_data->time;
  novatel_heading_mutex_.unlock();

  double delta_time_between_height = 0.0;
  height_mutex_.lock();
  delta_time_between_height = measure_data->time - map_height_time_;
  height_mutex_.unlock();

  if (delta_time_between_height < 1.0) {
    AINFO << "the heading time and delta time: " << std::setprecision(15)
          << measure_data->time << " " << delta_time_between_height;
    return false;
  }

  integ_pva_mutex_.lock();
  bool is_sins_align =
      integ_pva_list_.back().init_and_alignment && (integ_pva_list_.size() > 1);
  integ_pva_mutex_.unlock();

  if (is_sins_align) {
    static double pre_publish_time = 0.0;
    if (measure_data->time - pre_publish_time < 0.5) {
      return false;
    }
    pre_publish_time = measure_data->time;
  }
  double gnss_yaw = heading_msg.heading();
  double heading_std = heading_msg.heading_std_dev();

  static double imu_ant_yaw_angle = 0.0;
  if (imu_ant_yaw_angle == 0.0) {
    imu_ant_yaw_angle =
        -atan2(imu_gnssant_extrinsic_.transform_2.translation()[0] -
                   imu_gnssant_extrinsic_.transform_1.translation()[0],
               imu_gnssant_extrinsic_.transform_2.translation()[1] -
                   imu_gnssant_extrinsic_.transform_1.translation()[1]) *
        57.295779513082323;
    AINFO << "imu_gnssant_extrinsic_: "
          << imu_gnssant_extrinsic_.transform_2.translation()[0] << ", "
          << imu_gnssant_extrinsic_.transform_1.translation()[0] << ", "
          << imu_gnssant_extrinsic_.transform_2.translation()[1] << ", "
          << imu_gnssant_extrinsic_.transform_2.translation()[1];
    AINFO << "the yaw between double ant yaw and vehicle: "
          << imu_ant_yaw_angle;
  }
  AINFO << "novatel heading is: " << std::setprecision(15) << measure_data->time
        << " " << std::setprecision(6) << gnss_yaw;
  if (gnss_yaw > 180) {
    // the novatel yaw angle is 0-360deg
    gnss_yaw -= 360.0;
  }
  gnss_yaw += imu_ant_yaw_angle;

  if (gnss_yaw > 180) {
    gnss_yaw -= 360.0;
  }
  measure_data->measure_type = MeasureType::GNSS_DOUBLE_ANT_YAW;
  measure_data->is_have_variance = true;
  // 3.046174197867086e-04 = (pi / 180)^2
  AINFO << "the novatel heading std: " << std::setprecision(15)
        << measure_data->time << " " << heading_std;
  measure_data->variance[8][8] =
      heading_std * heading_std * 3.046174197867086e-04;
  measure_data->gnss_att.yaw = -gnss_yaw * 0.017453292519943;

  AINFO << "measure data heading is: " << std::setprecision(15)
        << measure_data->time << " " << std::setprecision(6)
        << measure_data->gnss_att.yaw;

  return true;
}

bool MeasureRepublishProcess::LoadImuGnssAntennaExtrinsic(
    std::string file_path, VehicleGnssAntExtrinsic* extrinsic) const {
  YAML::Node confige = YAML::LoadFile(file_path);
  if (confige["leverarm"]) {
    if (confige["leverarm"]["primary"]["offset"]) {
      extrinsic->transform_1.translation()[0] =
          confige["leverarm"]["primary"]["offset"]["x"].as<double>();
      extrinsic->transform_1.translation()[1] =
          confige["leverarm"]["primary"]["offset"]["y"].as<double>();
      extrinsic->transform_1.translation()[2] =
          confige["leverarm"]["primary"]["offset"]["z"].as<double>();
    } else {
      return false;
    }
    if (confige["leverarm"]["secondary"]["offset"]) {
      extrinsic->transform_2.translation()[0] =
          confige["leverarm"]["secondary"]["offset"]["x"].as<double>();
      extrinsic->transform_2.translation()[1] =
          confige["leverarm"]["secondary"]["offset"]["y"].as<double>();
      extrinsic->transform_2.translation()[2] =
          confige["leverarm"]["secondary"]["offset"]["z"].as<double>();
      extrinsic->ant_num = 2;

      if (confige["leverarm"]["secondary"]["rotation"]) {
        double qx =
            confige["leverarm"]["secondary"]["rotation"]["x"].as<double>();
        double qy =
            confige["leverarm"]["secondary"]["rotation"]["y"].as<double>();
        double qz =
            confige["leverarm"]["secondary"]["rotation"]["z"].as<double>();
        double qw =
            confige["leverarm"]["secondary"]["rotation"]["w"].as<double>();
        extrinsic->transform_1.linear() =
            Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
      } else {
        double yaw = atan2(extrinsic->transform_2.translation()[0] -
                               extrinsic->transform_1.translation()[0],
                           extrinsic->transform_2.translation()[1] -
                               extrinsic->transform_1.translation()[1]);
        double quat[4] = {0.0};
        math::EulerToQuaternion(0.0, 0.0, yaw, quat);
        extrinsic->transform_1.linear() =
            Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3])
                .toRotationMatrix();
      }
      return true;
    } else {
      extrinsic->ant_num = 1;
      return true;
    }
  }
  return false;
}

bool MeasureRepublishProcess::CheckBestgnssPoseXYStd(
    const GnssBestPose& bestgnsspos_msg) {
  // check the standard deviation of xy
  if ((bestgnsspos_msg.longitude_std_dev() > GNSS_XY_STD_THRESHOLD) ||
      (bestgnsspos_msg.latitude_std_dev() > GNSS_XY_STD_THRESHOLD)) {
    AWARN << "the position std is large: "
          << bestgnsspos_msg.longitude_std_dev() << " "
          << bestgnsspos_msg.latitude_std_dev();
    return false;
  }
  return true;
}

bool MeasureRepublishProcess::CheckBestgnssposeStatus(
    const GnssBestPose& bestgnsspos_msg) {
  int gnss_solution_status = static_cast<int>(bestgnsspos_msg.sol_status());
  int gnss_position_type = static_cast<int>(bestgnsspos_msg.sol_type());
  AINFO << "the gnss solution_status and position_type: "
        << gnss_solution_status << " " << gnss_position_type;

  if (gnss_solution_status != 0) {
    AINFO << "novatel gnsspos's solution_status is not computed: "
          << gnss_solution_status;
    return false;
  }
  if (gnss_position_type == 0 || gnss_position_type == 1 ||
      gnss_position_type == 2) {
    AINFO << "novatel gnsspos's solution_type is invalid "
          << "or xy fixed or height fixed: " << gnss_position_type;
    return false;
  }

  return true;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
