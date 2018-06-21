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

#include "modules/localization/msf/msf_localization.h"

#include <yaml-cpp/yaml.h>
#include <list>

#include "modules/drivers/gnss/proto/config.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/math/euler_angles_zxy.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {

using ::Eigen::Vector3d;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::ImuAdapter;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::Status;
using apollo::common::time::Clock;

MSFLocalization::MSFLocalization()
    : monitor_logger_(MonitorMessageItem::LOCALIZATION),
      localization_state_(msf::LocalizationMeasureState::OK),
      pcd_msg_index_(-1),
      latest_lidar_localization_status_(MeasureState::NOT_VALID),
      latest_gnss_localization_status_(MeasureState::NOT_VALID) {}

Status MSFLocalization::Start() {
  AdapterManager::Init(FLAGS_msf_adapter_config_file);

  Status &&status = Init();
  if (!status.ok()) {
    return status;
  }

  common::monitor::MonitorLogBuffer buffer(&monitor_logger_);

  // Raw Imu
  if (!AdapterManager::GetRawImu()) {
    buffer.ERROR(
        "Raw IMU input not initialized. Check your adapter.conf file!");
    buffer.PrintLog();
    return Status(common::LOCALIZATION_ERROR_MSG, "no Raw IMU adapter");
  }
  AdapterManager::AddRawImuCallback(&MSFLocalization::OnRawImu, this);

  // Point Cloud
  if (FLAGS_enable_lidar_localization) {
    if (!AdapterManager::GetPointCloud()) {
      buffer.ERROR(
          "PointCloud input not initialized. Check your adapter.conf file!");
      buffer.PrintLog();
      return Status(common::LOCALIZATION_ERROR_MSG, "no PointCloud adapter");
    }
    AdapterManager::AddPointCloudCallback(&MSFLocalization::OnPointCloud, this);
  }

  if (FLAGS_gnss_mode == 1) {
    // Gnss Rtk Obs
    if (!AdapterManager::GetGnssRtkObs()) {
      buffer.ERROR(
          "GnssRtkObs input not initialized. Check your adapter.conf file!");
      buffer.PrintLog();
      return Status(common::LOCALIZATION_ERROR_MSG, "no GnssRtkObs adapter");
    }
    AdapterManager::AddGnssRtkObsCallback(&MSFLocalization::OnGnssRtkObs, this);

    // Gnss Rtk Eph
    if (!AdapterManager::GetGnssRtkEph()) {
      buffer.ERROR(
          "GnssRtkEph input not initialized. Check your adapter.conf file!");
      buffer.PrintLog();
      return Status(common::LOCALIZATION_ERROR_MSG, "no GnssRtkEph adapter");
    }
    AdapterManager::AddGnssRtkEphCallback(&MSFLocalization::OnGnssRtkEph, this);
  } else {
    // Gnss Best Pose
    if (!AdapterManager::GetGnssBestPose()) {
      buffer.ERROR(
          "GnssBestPose input not initialized. Check your adapter.conf file!");
      buffer.PrintLog();
      return Status(common::LOCALIZATION_ERROR_MSG, "no GnssBestPose adapter");
    }
    AdapterManager::AddGnssBestPoseCallback(&MSFLocalization::OnGnssBestPose,
                                            this);
  }

  tf2_broadcaster_.reset(new tf2_ros::TransformBroadcaster);

  return Status::OK();
}

Status MSFLocalization::Stop() { return Status::OK(); }

Status MSFLocalization::Init() {
  InitParams();

  return localization_integ_.Init(localizaiton_param_);
}

void MSFLocalization::InitParams() {
  // integration module
  localizaiton_param_.is_ins_can_self_align = FLAGS_integ_ins_can_self_align;
  localizaiton_param_.is_sins_align_with_vel = FLAGS_integ_sins_align_with_vel;
  localizaiton_param_.is_sins_state_check = FLAGS_integ_sins_state_check;
  localizaiton_param_.sins_state_span_time = FLAGS_integ_sins_state_span_time;
  localizaiton_param_.sins_state_pos_std = FLAGS_integ_sins_state_pos_std;
  localizaiton_param_.vel_threshold_get_yaw = FLAGS_vel_threshold_get_yaw;
  localizaiton_param_.integ_debug_log_flag = FLAGS_integ_debug_log_flag;
  localizaiton_param_.is_trans_gpstime_to_utctime =
      FLAGS_trans_gpstime_to_utctime;
  localizaiton_param_.gnss_mode = FLAGS_gnss_mode;
  localizaiton_param_.is_using_raw_gnsspos = true;

  // gnss module
  localizaiton_param_.enable_ins_aid_rtk = FLAGS_enable_ins_aid_rtk;

  // lidar module
  localizaiton_param_.map_path = FLAGS_map_dir + "/" + FLAGS_local_map_name;
  localizaiton_param_.lidar_extrinsic_file = FLAGS_lidar_extrinsics_file;
  localizaiton_param_.lidar_height_file = FLAGS_lidar_height_file;
  localizaiton_param_.lidar_height_default = FLAGS_lidar_height_default;
  localizaiton_param_.lidar_debug_log_flag = FLAGS_lidar_debug_log_flag;
  localizaiton_param_.localization_mode = FLAGS_lidar_localization_mode;
  localizaiton_param_.lidar_yaw_align_mode = FLAGS_lidar_yaw_align_mode;
  localizaiton_param_.lidar_filter_size = FLAGS_lidar_filter_size;
  localizaiton_param_.map_coverage_theshold = FLAGS_lidar_map_coverage_theshold;
  localizaiton_param_.imu_lidar_max_delay_time = FLAGS_lidar_imu_max_delay_time;

  AERROR << "map: " << localizaiton_param_.map_path;
  AERROR << "lidar_extrin: " << localizaiton_param_.lidar_extrinsic_file;
  AERROR << "lidar_height: " << localizaiton_param_.lidar_height_file;

  localizaiton_param_.utm_zone_id = FLAGS_local_utm_zone_id;
  // try load zone id from local_map folder
  if (FLAGS_if_utm_zone_id_from_folder) {
    bool success = LoadZoneIdFromFolder(localizaiton_param_.map_path,
                                      &localizaiton_param_.utm_zone_id);
    if (!success) {
      AWARN << "Can't load utm zone id from map folder, use default value.";
    }
  }
  AINFO << "utm zone id: " << localizaiton_param_.utm_zone_id;

  // vehicle imu extrinsic
  imu_vehicle_quat_.x() = FLAGS_imu_vehicle_qx;
  imu_vehicle_quat_.y() = FLAGS_imu_vehicle_qy;
  imu_vehicle_quat_.z() = FLAGS_imu_vehicle_qz;
  imu_vehicle_quat_.w() = FLAGS_imu_vehicle_qw;
  // try to load imu vehicle quat from file
  if (FLAGS_if_vehicle_imu_from_file) {
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 0.0;

    std::string vehicle_imu_file =
        common::util::TranslatePath(FLAGS_vehicle_imu_file);
    AINFO << "Vehile imu file: " << vehicle_imu_file;

    if (LoadImuVehicleExtrinsic(vehicle_imu_file, &qx, &qy, &qz, &qw)) {
      imu_vehicle_quat_.x() = qx;
      imu_vehicle_quat_.y() = qy;
      imu_vehicle_quat_.z() = qz;
      imu_vehicle_quat_.w() = qw;
    } else {
      AWARN << "Can't load imu vehicle quat from file, use default value.";
    }
  }
  AINFO << "imu_vehicle_quat: "
        << imu_vehicle_quat_.x() << " "
        << imu_vehicle_quat_.y() << " "
        << imu_vehicle_quat_.z() << " "
        << imu_vehicle_quat_.w();

  // common
  localizaiton_param_.enable_lidar_localization =
      FLAGS_enable_lidar_localization;

  if (!FLAGS_if_imuant_from_file) {
    localizaiton_param_.imu_to_ant_offset.offset_x = FLAGS_imu_to_ant_offset_x;
    localizaiton_param_.imu_to_ant_offset.offset_y = FLAGS_imu_to_ant_offset_y;
    localizaiton_param_.imu_to_ant_offset.offset_z = FLAGS_imu_to_ant_offset_z;
    localizaiton_param_.imu_to_ant_offset.uncertainty_x =
        FLAGS_imu_to_ant_offset_ux;
    localizaiton_param_.imu_to_ant_offset.uncertainty_y =
        FLAGS_imu_to_ant_offset_uy;
    localizaiton_param_.imu_to_ant_offset.uncertainty_z =
        FLAGS_imu_to_ant_offset_uz;
  } else {
    double offset_x = 0.0;
    double offset_y = 0.0;
    double offset_z = 0.0;
    double uncertainty_x = 0.0;
    double uncertainty_y = 0.0;
    double uncertainty_z = 0.0;
    std::string ant_imu_leverarm_file =
        common::util::TranslatePath(FLAGS_ant_imu_leverarm_file);
    AERROR << "Ant imu lever arm file: " << ant_imu_leverarm_file;
    CHECK(LoadGnssAntennaExtrinsic(ant_imu_leverarm_file, &offset_x, &offset_y,
                                   &offset_z, &uncertainty_x, &uncertainty_y,
                                   &uncertainty_z));

    localizaiton_param_.imu_to_ant_offset.offset_x = offset_x;
    localizaiton_param_.imu_to_ant_offset.offset_y = offset_y;
    localizaiton_param_.imu_to_ant_offset.offset_z = offset_z;
    localizaiton_param_.imu_to_ant_offset.uncertainty_x = uncertainty_x;
    localizaiton_param_.imu_to_ant_offset.uncertainty_y = uncertainty_y;
    localizaiton_param_.imu_to_ant_offset.uncertainty_z = uncertainty_z;

    AINFO << localizaiton_param_.imu_to_ant_offset.offset_x << " "
          << localizaiton_param_.imu_to_ant_offset.offset_y << " "
          << localizaiton_param_.imu_to_ant_offset.offset_z << " "
          << localizaiton_param_.imu_to_ant_offset.uncertainty_x << " "
          << localizaiton_param_.imu_to_ant_offset.uncertainty_y << " "
          << localizaiton_param_.imu_to_ant_offset.uncertainty_z;
  }
}

void MSFLocalization::OnPointCloud(const sensor_msgs::PointCloud2 &message) {
  ++pcd_msg_index_;
  if (pcd_msg_index_ % FLAGS_point_cloud_step != 0) {
    return;
  }

  localization_integ_.PcdProcess(message);

  if (FLAGS_lidar_debug_log_flag) {
    std::list<msf::LocalizationResult> lidar_localization_list;
    localization_integ_.GetLidarLocalizationList(&lidar_localization_list);

    for (auto itr = lidar_localization_list.begin();
         itr != lidar_localization_list.end(); ++itr) {
      latest_lidar_localization_status_ =
          static_cast<MeasureState>(itr->state());
      if (itr->state() == msf::LocalizationMeasureState::OK ||
          itr->state() == msf::LocalizationMeasureState::VALID) {
        // publish lidar message to debug
        AdapterManager::PublishLocalizationMsfLidar(itr->localization());
      }
    }
  }

  return;
}

void MSFLocalization::OnRawImu(const drivers::gnss::Imu &imu_msg) {
  if (FLAGS_imu_coord_rfu) {
    localization_integ_.RawImuProcessRfu(imu_msg);
  } else {
    localization_integ_.RawImuProcessFlu(imu_msg);
  }

  std::list<msf::LocalizationResult> integ_localization_list;
  localization_integ_.GetIntegLocalizationList(&integ_localization_list);

  for (auto itr = integ_localization_list.begin();
       itr != integ_localization_list.end(); ++itr) {
    // compose localization status
    LocalizationStatus status;
    apollo::common::Header *status_headerpb = status.mutable_header();
    status_headerpb->set_timestamp_sec(
        itr->localization().header().timestamp_sec());
    status.set_fusion_status(static_cast<MeasureState>(itr->state()));
    status.set_lidar_status(latest_lidar_localization_status_);
    status.set_gnss_status(latest_gnss_localization_status_);
    status.set_measurement_time(itr->localization().measurement_time());
    AdapterManager::PublishLocalizationMsfStatus(status);

    if (itr->state() == msf::LocalizationMeasureState::OK ||
        itr->state() == msf::LocalizationMeasureState::VALID) {
      // caculate orientation_vehicle_world
      LocalizationEstimate local_result = itr->localization();
      apollo::localization::Pose *posepb_loc = local_result.mutable_pose();
      const apollo::common::Quaternion& orientation =
          posepb_loc->orientation();
      const Eigen::Quaternion<double> quaternion(
          orientation.qw(), orientation.qx(),
          orientation.qy(), orientation.qz());
      Eigen::Quaternion<double> quat_vehicle_world =
          quaternion * imu_vehicle_quat_;

      // set heading according to rotation of vehicle
      posepb_loc->set_heading(
          common::math::QuaternionToHeading(quat_vehicle_world.w(),
                                            quat_vehicle_world.x(),
                                            quat_vehicle_world.y(),
                                            quat_vehicle_world.z()));

      // set euler angles according to rotation of vehicle
      apollo::common::Point3D *eulerangles =
          posepb_loc->mutable_euler_angles();
      common::math::EulerAnglesZXYd euler_angle(quat_vehicle_world.w(),
                                                quat_vehicle_world.x(),
                                                quat_vehicle_world.y(),
                                                quat_vehicle_world.z());
      eulerangles->set_x(euler_angle.pitch());
      eulerangles->set_y(euler_angle.roll());
      eulerangles->set_z(euler_angle.yaw());

      PublishPoseBroadcastTF(local_result);
      AdapterManager::PublishLocalization(local_result);
    }
  }

  if (integ_localization_list.size()) {
    localization_state_ = integ_localization_list.back().state();
  }

  return;
}  // namespace localization

void MSFLocalization::OnGnssBestPose(
    const drivers::gnss::GnssBestPose &bestgnsspos_msg) {
  if ((localization_state_ == msf::LocalizationMeasureState::OK ||
       localization_state_ == msf::LocalizationMeasureState::VALID) &&
      FLAGS_gnss_only_init) {
    return;
  }

  localization_integ_.GnssBestPoseProcess(bestgnsspos_msg);

  if (FLAGS_gnss_debug_log_flag) {
    std::list<msf::LocalizationResult> gnss_localization_list;
    localization_integ_.GetGnssLocalizationList(&gnss_localization_list);

    for (auto itr = gnss_localization_list.begin();
         itr != gnss_localization_list.end(); ++itr) {
      latest_gnss_localization_status_ =
          static_cast<MeasureState>(itr->state());
      if (itr->state() == msf::LocalizationMeasureState::OK ||
          itr->state() == msf::LocalizationMeasureState::VALID) {
        AdapterManager::PublishLocalizationMsfGnss(itr->localization());
      }
    }
  }

  return;
}

void MSFLocalization::OnGnssRtkObs(
    const drivers::gnss::EpochObservation &raw_obs_msg) {
  if ((localization_state_ == msf::LocalizationMeasureState::OK ||
       localization_state_ == msf::LocalizationMeasureState::VALID) &&
      FLAGS_gnss_only_init) {
    return;
  }

  localization_integ_.RawObservationProcess(raw_obs_msg);

  if (FLAGS_gnss_debug_log_flag) {
    std::list<msf::LocalizationResult> gnss_localization_list;
    localization_integ_.GetGnssLocalizationList(&gnss_localization_list);

    for (auto itr = gnss_localization_list.begin();
         itr != gnss_localization_list.end(); ++itr) {
      latest_gnss_localization_status_ =
          static_cast<MeasureState>(itr->state());
      if (itr->state() == msf::LocalizationMeasureState::OK ||
          itr->state() == msf::LocalizationMeasureState::VALID) {
        AdapterManager::PublishLocalizationMsfGnss(itr->localization());
      }
    }
  }

  return;
}

void MSFLocalization::OnGnssRtkEph(
    const drivers::gnss::GnssEphemeris &gnss_orbit_msg) {
  if ((localization_state_ == msf::LocalizationMeasureState::OK ||
       localization_state_ == msf::LocalizationMeasureState::VALID) &&
      FLAGS_gnss_only_init) {
    return;
  }

  localization_integ_.RawEphemerisProcess(gnss_orbit_msg);
  return;
}

bool MSFLocalization::LoadGnssAntennaExtrinsic(
    const std::string &file_path, double *offset_x, double *offset_y,
    double *offset_z, double *uncertainty_x, double *uncertainty_y,
    double *uncertainty_z) {
  YAML::Node config = YAML::LoadFile(file_path);
  if (config["leverarm"]) {
    if (config["leverarm"]["primary"]["offset"]) {
      *offset_x = config["leverarm"]["primary"]["offset"]["x"].as<double>();
      *offset_y = config["leverarm"]["primary"]["offset"]["y"].as<double>();
      *offset_z = config["leverarm"]["primary"]["offset"]["z"].as<double>();

      if (config["leverarm"]["primary"]["uncertainty"]) {
        *uncertainty_x =
            config["leverarm"]["primary"]["uncertainty"]["x"].as<double>();
        *uncertainty_y =
            config["leverarm"]["primary"]["uncertainty"]["y"].as<double>();
        *uncertainty_z =
            config["leverarm"]["primary"]["uncertainty"]["z"].as<double>();
      }
      return true;
    }
  }
  return false;
}

bool MSFLocalization::LoadImuVehicleExtrinsic(
    const std::string &file_path, double *quat_qx, double *quat_qy,
    double *quat_qz, double *quat_qw) {
  if (!common::util::PathExists(file_path)) {
    return false;
  }
  YAML::Node config = YAML::LoadFile(file_path);
  if (config["transform"]) {
    if (config["transform"]["translation"]) {
      if (config["transform"]["rotation"]) {
        *quat_qx = config["transform"]["rotation"]["x"].as<double>();
        *quat_qy = config["transform"]["rotation"]["y"].as<double>();
        *quat_qz = config["transform"]["rotation"]["z"].as<double>();
        *quat_qw = config["transform"]["rotation"]["w"].as<double>();
        return true;
      }
    }
  }
  return false;
}

bool MSFLocalization::LoadZoneIdFromFolder(const std::string &folder_path,
                                         int *zone_id) {
  std::string map_zone_id_folder;
  if (common::util::DirectoryExists(folder_path + "/map/000/north")) {
    map_zone_id_folder = folder_path + "/map/000/north";
  } else if (common::util::DirectoryExists(folder_path + "/map/000/south")) {
    map_zone_id_folder = folder_path + "/map/000/south";
  } else {
    return false;
  }

  auto folder_list = common::util::ListSubDirectories(map_zone_id_folder);
  for (auto itr = folder_list.begin(); itr != folder_list.end(); ++itr) {
    *zone_id = std::stoi(*itr);
    return true;
  }
  return false;
}

}  // namespace localization
}  // namespace apollo
