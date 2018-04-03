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

#include "modules/localization/msf/local_integ/measure_republish_process.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <iomanip>
#include "modules/localization/msf/common/util/time_conversion.h"
#include "modules/common/math/euler_angles_zxy.h"
#include "modules/common/log.h"

namespace apollo {
namespace localization {
namespace msf {

MeasureRepublishProcess::MeasureRepublishProcess() {
  gnss_mode_ = GnssMode::NOVATEL;
}

MeasureRepublishProcess::~MeasureRepublishProcess() {
  pthread_mutex_destroy(&integ_pva_mutex_);
  pthread_mutex_destroy(&height_mutex_);
}

LocalizationState MeasureRepublishProcess::Init(
    const LocalizationIntegParam& params) {
  local_utm_zone_id_ = params.utm_zone_id;
  debug_log_flag_ = params.integ_debug_log_flag;
  is_trans_gpstime_to_utctime_ = params.is_trans_gpstime_to_utctime;
  gnss_mode_ = GnssMode(params.gnss_mode);

  // pre_bestgnsspose_ = {0.0};
  pva_buffer_size_ = 150;
  pthread_mutex_init(&integ_pva_mutex_, NULL);
  pthread_mutex_init(&height_mutex_, NULL);

  map_height_ = 0.0;
  lidar_pose_ = TransformD::Identity();
  map_height_time_ = 0.0;
  return LocalizationState::OK();
}

bool MeasureRepublishProcess::NovatelBestgnssposProcess(
    const GnssBestPose& bestgnsspos_msg, MeasureData *measure) {
  if (gnss_mode_ != GnssMode::NOVATEL) {
    return false;
  }

  int gnss_solution_status = static_cast<int>(bestgnsspos_msg.sol_status());
  int gnss_position_type = static_cast<int>(bestgnsspos_msg.sol_type());
  LOG(INFO) << "the gnss solution_status and position_type: "
            << gnss_solution_status << " " << gnss_position_type;

  if (gnss_solution_status != 0) {
    LOG(INFO) << "novatel gnsspos's solution_status is not computed: "
              << gnss_solution_status;
    return false;
  }
  if (gnss_position_type == 0
      || gnss_position_type == 1
      || gnss_position_type == 2) {
    LOG(INFO) << "novatel gnsspos's solution_type is invalid "
              << "or xy fixed or height fixed: "
              << gnss_position_type;
    return false;
  }

  MeasureData measure_data;

  measure_data.time = bestgnsspos_msg.measurement_time();
  if (is_trans_gpstime_to_utctime_) {
    measure_data.time = util::GpsToUnixSeconds(measure_data.time);
  }

  measure_data.gnss_pos.longitude =
      bestgnsspos_msg.longitude() * 0.017453292519943;
  measure_data.gnss_pos.latitude =
      bestgnsspos_msg.latitude() * 0.017453292519943;
  measure_data.gnss_pos.height =
      bestgnsspos_msg.height_msl() + bestgnsspos_msg.undulation();

  measure_data.variance[0][0] =
      bestgnsspos_msg.longitude_std_dev() * bestgnsspos_msg.longitude_std_dev();
  measure_data.variance[1][1] =
      bestgnsspos_msg.latitude_std_dev() * bestgnsspos_msg.latitude_std_dev();
  measure_data.variance[2][2] =
      bestgnsspos_msg.height_std_dev() * bestgnsspos_msg.height_std_dev();

  pthread_mutex_lock(&integ_pva_mutex_);
  bool is_sins_align =
      integ_pva_list_.back().init_and_alignment
      && (integ_pva_list_.size() > 1);
  pthread_mutex_unlock(&integ_pva_mutex_);

  if (is_sins_align) {
    measure_data.measure_type = MeasureType::GNSS_POS_ONLY;
    measure_data.frame_type = FrameType::ENU;
    pthread_mutex_lock(&height_mutex_);
    if ((measure_data.time - 1.0 < map_height_time_)) {
      measure_data.measure_type = MeasureType::GNSS_POS_XY;
    }
    pthread_mutex_unlock(&height_mutex_);
    measure_data.is_have_variance = true;
  } else {
    measure_data.measure_type = MeasureType::GNSS_POS_VEL;
    measure_data.frame_type = FrameType::ENU;
    measure_data.is_have_variance = false;

    static bool is_sent_init_pos = false;
    if (!is_sent_init_pos) {
      is_sent_init_pos = true;
      measure_data.gnss_vel.ve = 0.0;
      measure_data.gnss_vel.vn = 0.0;
      measure_data.gnss_vel.vu = 0.0;
      LOG(INFO) << "Novatel bestgnsspose publish: "
                << "send sins init position using novatel bestgnsspos!";
      *measure = measure_data;
      return true;
    }

    static int position_good_counter = 0;
    if ((bestgnsspos_msg.longitude_std_dev() > 5.0) ||
        (bestgnsspos_msg.latitude_std_dev() > 5.0)) {
      LOG(WARNING) << "the position std is large: "
                << bestgnsspos_msg.longitude_std_dev() << " "
                << bestgnsspos_msg.latitude_std_dev();
      return false;
    }
    if (position_good_counter < 10) {
      ++position_good_counter;
      pre_bestgnsspose_ = measure_data;
      return false;
    }

    if (pre_bestgnsspose_.time) {
      double re = 6378137.0;
      double e = 0.0033528;
      double temp_sin_lati_2 = sin(measure_data.gnss_pos.latitude) *
                               sin(measure_data.gnss_pos.latitude);
      double rn = re / (1 - e * temp_sin_lati_2);
      double rm = re / (1 + 2.0 * e - 3.0 * e * temp_sin_lati_2);

      double inv_time = 1.0 / (measure_data.time - pre_bestgnsspose_.time);
      if (measure_data.time - pre_bestgnsspose_.time > 1.05) {
        pre_bestgnsspose_ = measure_data;
        position_good_counter = 0;
        return false;
      }
      measure_data.gnss_vel.ve = (measure_data.gnss_pos.longitude -
                                  pre_bestgnsspose_.gnss_pos.longitude) *
                                 rn * inv_time *
                                 cos(measure_data.gnss_pos.latitude);
      measure_data.gnss_vel.vn = (measure_data.gnss_pos.latitude -
                                  pre_bestgnsspose_.gnss_pos.latitude) *
                                 rm * inv_time;
      measure_data.gnss_vel.vu =
          (measure_data.gnss_pos.height - pre_bestgnsspose_.gnss_pos.height) *
          inv_time;

      pre_bestgnsspose_ = measure_data;
      LOG(INFO) << "novatel bestgnsspos velocity: "
                << measure_data.gnss_vel.ve << " "
                << measure_data.gnss_vel.vn << " "
                << measure_data.gnss_vel.vu;

      static double pre_yaw_from_vel = 0.0;
      double yaw_from_vel =
          atan2(measure_data.gnss_vel.ve, measure_data.gnss_vel.vn);
      if (!pre_yaw_from_vel) {
        pre_yaw_from_vel = yaw_from_vel;
        return false;
      } else {
        double delta_yaw = yaw_from_vel - pre_yaw_from_vel;
        if (delta_yaw > 6.1087) {
          delta_yaw = delta_yaw - 6.2832;
        }
        if (delta_yaw < -6.1087) {
          delta_yaw = delta_yaw + 6.2832;
        }

        LOG(INFO) << "yaw from position difference: "
                  << yaw_from_vel * 57.2958;
        // 0.0872rad = 5deg
        if ((delta_yaw > 0.0872) || (delta_yaw < -0.0872)) {
          LOG(WARNING) << "novatel bestgnsspos delta yaw is large! "
                    << "pre, cur yaw from vel and delta: "
                    << pre_yaw_from_vel * 57.2958 << " "
                    << yaw_from_vel * 57.2958 << " "
                    << (yaw_from_vel - pre_yaw_from_vel) * 57.2958;
          pre_yaw_from_vel = yaw_from_vel;
          return false;
        }
        pre_yaw_from_vel = yaw_from_vel;
      }
    } else {
      pre_bestgnsspose_ = measure_data;
      return false;
    }
  }

  // _component->publish_integ_measure_data(&measure_data);
  // TranferToIntegMeasureData(measure_data, measure);
  *measure = measure_data;

  if (debug_log_flag_) {
    LOG(INFO) << std::setprecision(16)
              << "MeasureDataRepublish Debug Log: bestgnsspos msg: "
              << "[time:" << measure_data.time << "]"
              << "[x:" << measure_data.gnss_pos.longitude * 57.295779513082323
              << "]"
              << "[y:" << measure_data.gnss_pos.latitude * 57.295779513082323
              << "]"
              << "[z:" << measure_data.gnss_pos.height << "]"
              << "[std_x:" << bestgnsspos_msg.longitude_std_dev() << "]"
              << "[std_y:" << bestgnsspos_msg.latitude_std_dev() << "]"
              << "[std_z:" << bestgnsspos_msg.height_std_dev() << "]"
              << "[position_type:" << bestgnsspos_msg.sol_type() << "]";
  }
  return true;
}

void MeasureRepublishProcess::GnssLocalProcess(
    const MeasureData& gnss_local_msg, MeasureData *measure) {
  if (gnss_mode_ != GnssMode::SELF) {
    return;
  }

  MeasureData measure_data = gnss_local_msg;
  if (is_trans_gpstime_to_utctime_) {
    measure_data.time = util::GpsToUnixSeconds(measure_data.time);
  }

  LOG(INFO) << "the gnss velocity: " << measure_data.gnss_vel.ve << " "
            << measure_data.gnss_vel.vn << " " << measure_data.gnss_vel.vu;

  measure_data.gnss_att.pitch = 0.0;
  measure_data.gnss_att.roll = 0.0;

  Eigen::Vector3d pos_xyz = Eigen::Vector3d::Zero();
  pos_xyz[0] = measure_data.gnss_pos.longitude;
  pos_xyz[1] = measure_data.gnss_pos.latitude;
  pos_xyz[2] = measure_data.gnss_pos.height;

  Eigen::Vector3d pos_blh = Eigen::Vector3d::Zero();
  apollo::localization::msf::xyz_to_blh(pos_xyz, &pos_blh);
  measure_data.gnss_pos.longitude = pos_blh[0];
  measure_data.gnss_pos.latitude = pos_blh[1];
  measure_data.gnss_pos.height = pos_blh[2];

  double ve_std = std::sqrt(measure_data.variance[3][3]);
  double vn_std = std::sqrt(measure_data.variance[4][4]);
  double vu_std = std::sqrt(measure_data.variance[5][5]);
  LOG(INFO) << "the gnss velocity std: " << ve_std << " " << vn_std << " "
            << vu_std;

  pthread_mutex_lock(&integ_pva_mutex_);
  bool is_sins_align =
      integ_pva_list_.back().init_and_alignment && (integ_pva_list_.size() > 1);
  pthread_mutex_unlock(&integ_pva_mutex_);

  if (is_sins_align) {
    measure_data.measure_type = MeasureType::GNSS_POS_ONLY;
    pthread_mutex_lock(&height_mutex_);
    if ((measure_data.time - 1.0 < map_height_time_)) {
      measure_data.measure_type = MeasureType::GNSS_POS_XY;
    }
    pthread_mutex_unlock(&height_mutex_);
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
      LOG(INFO) << "send sins init position using rtk-gnss position!";
      // _component->publish_integ_measure_data(&measure_data);
      // TranferToIntegMeasureData(measure_data, measure);
      *measure = measure_data;
      return;
    }

    static int position_good_counter = 0;
    if (position_good_counter < 10) {
      ++position_good_counter;
      return;
    }

    if (gnss_local_msg.measure_type != MeasureType::GNSS_POS_VEL
        && gnss_local_msg.measure_type != MeasureType::ENU_VEL_ONLY) {
      LOG(ERROR) << "gnss does not have velocity,"
                 << "the gnss velocity std: "
                 << ve_std << " " << vn_std << " " << vu_std;
      return;
    }
    if (!gnss_local_msg.is_have_variance) {
      LOG(ERROR) << "gnss velocity does not have velocity variance!";
      return;
    } else {
      if ((ve_std > 0.1) || (vn_std > 0.1)) {
        LOG(INFO) << "gnss velocity variance is large: " << ve_std << " "
                  << vn_std;
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
      double delta_yaw = yaw_from_vel - pre_yaw_from_vel;
      if (delta_yaw > 6.1087) {
        delta_yaw = delta_yaw - 6.2832;
      }
      if (delta_yaw < -6.1087) {
        delta_yaw = delta_yaw + 6.2832;
      }

      LOG(INFO) << "yaw from position difference: " << yaw_from_vel * 57.2958;
      double delta_time = measure_data.time - pre_measure_time;
      if (delta_time < 1.0e-10) {
        LOG(INFO) << "the delta time is too small: " << delta_time;
      }
      double yaw_incr = delta_yaw / delta_time;
      // 0.0872rad = 5deg
      if ((yaw_incr > 0.0872) || (yaw_incr < -0.0872)) {
        LOG(INFO)
            << "yaw velocity is large! pre, cur yaw from vel and velocity: "
            << pre_yaw_from_vel * 57.2958 << " " << yaw_from_vel * 57.2958
            << " " << yaw_incr * 57.2958;
        pre_measure_time = measure_data.time;
        pre_yaw_from_vel = yaw_from_vel;
        return;
      }
      pre_measure_time = measure_data.time;
      pre_yaw_from_vel = yaw_from_vel;
    }
  }
  // _component->publish_integ_measure_data(&measure_data);
  // TranferToIntegMeasureData(measure_data, measure);
  *measure = measure_data;

  if (debug_log_flag_) {
    LOG(INFO) << std::setprecision(16)
              << "MeasureDataRepublish Debug Log: rtkgnss msg: "
              << "[time:" << measure_data.time << "]"
              << "[x:" << measure_data.gnss_pos.longitude * 57.295779513082323
              << "]"
              << "[y:" << measure_data.gnss_pos.latitude * 57.295779513082323
              << "]"
              << "[z:" << measure_data.gnss_pos.height << "]"
              << "[std_x:" << measure_data.variance[0][0] << "]"
              << "[std_y:" << measure_data.variance[1][1] << "]"
              << "[std_z:" << measure_data.variance[2][2] << "]";
  }
  return;
}

void MeasureRepublishProcess::IntegPvaProcess(const InsPva& inspva_msg) {
  const InsPva& integ_pva = inspva_msg;

  pthread_mutex_lock(&integ_pva_mutex_);
  if (integ_pva_list_.size() < pva_buffer_size_) {
    integ_pva_list_.push_back(integ_pva);
  } else {
    integ_pva_list_.pop_front();
    integ_pva_list_.push_back(integ_pva);
  }
  pthread_mutex_unlock(&integ_pva_mutex_);

  return;
}

int MeasureRepublishProcess::LidarLocalProcess(
    const LocalizationEstimate& lidar_local_msg, MeasureData *measure) {
  MeasureData measure_data;  // = {0.0};
  measure_data.time = lidar_local_msg.measurement_time();

  apollo::localization::msf::WGS84Corr temp_wgs = {0.0};
  apollo::localization::msf::utmxy_to_latlon(
      lidar_local_msg.pose().position().x(),
      lidar_local_msg.pose().position().y(),
      local_utm_zone_id_, false, &temp_wgs);
  measure_data.gnss_pos.longitude = temp_wgs.log;
  measure_data.gnss_pos.latitude = temp_wgs.lat;
  measure_data.gnss_pos.height = lidar_local_msg.pose().position().z();

  QuaternionD temp_quaternion(lidar_local_msg.pose().orientation().qw(),
                              lidar_local_msg.pose().orientation().qx(),
                              lidar_local_msg.pose().orientation().qy(),
                              lidar_local_msg.pose().orientation().qz());

  common::math::EulerAnglesZXYd euler(temp_quaternion.w(),
                                      temp_quaternion.x(),
                                      temp_quaternion.y(),
                                      temp_quaternion.z());

  pthread_mutex_lock(&height_mutex_);
  Eigen::Vector3d trans(lidar_local_msg.pose().position().x(),
                        lidar_local_msg.pose().position().y(),
                        lidar_local_msg.pose().position().z());
  lidar_pose_ = Eigen::Translation3d(trans) * temp_quaternion;

  map_height_ = measure_data.gnss_pos.height;
  map_height_time_ = measure_data.time;
  pthread_mutex_unlock(&height_mutex_);

  measure_data.gnss_att.yaw = euler.yaw();
  measure_data.measure_type = MeasureType::POINT_CLOUD_POS;
  measure_data.frame_type = FrameType::UTM;

  measure_data.is_have_variance = true;

  double longitude_var = lidar_local_msg.uncertainty().position_std_dev().x();
  double latitude_var = lidar_local_msg.uncertainty().position_std_dev().y();
  double yaw_var = lidar_local_msg.uncertainty().orientation_std_dev().z();

  if ((std::abs(longitude_var - 1.0) < 0.000001) &&
      (std::abs(latitude_var - 1.0) < 0.000001)) {
    measure_data.variance[0][0] = 0.1 * 0.1;
    measure_data.variance[1][1] = 0.1 * 0.1;
    measure_data.variance[2][2] = 0.03 * 0.03;
  } else {
    measure_data.variance[0][0] = longitude_var;
    measure_data.variance[1][1] = latitude_var;
    measure_data.variance[2][2] = 0.03 * 0.03;
  }

  if (std::abs(yaw_var - 1.0) < 0.000001) {
    measure_data.variance[8][8] = 0.15 * 0.15 * 3.0461742e-04;
  } else {
    measure_data.variance[8][8] = yaw_var;
  }

  //   _output->publish_integ_measure_data(&measure_data);

  // TranferToIntegMeasureData(measure_data, measure);
  *measure = measure_data;

  //   if (_stop_gnss_with_pointcloud) {
  //     static int lidar_local_counter = 0;
  //     ++lidar_local_counter;
  //     if (lidar_local_counter == 10) {
  //       _stop_gnss = true;
  //     }
  //   }
  if (debug_log_flag_) {
    LOG(INFO) << std::setprecision(16)
              << "MeasureDataRepublish Debug Log: lidarLocal msg: "
              << "[time:" << measure_data.time << "]"
              << "[x:" << measure_data.gnss_pos.longitude * 57.295779513082323
              << "]"
              << "[y:" << measure_data.gnss_pos.latitude * 57.295779513082323
              << "]"
              << "[z:" << measure_data.gnss_pos.height << "]"
              << "[yaw:" << measure_data.gnss_att.yaw * 57.295779513082323
              << "]"
              << "[std_x:" << std::sqrt(longitude_var) << "]"
              << "[std_y:" << std::sqrt(latitude_var) << "]"
              << "[std_z:" << 0.03 << "]";
  }

  return 1;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
