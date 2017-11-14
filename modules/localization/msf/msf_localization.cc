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
#include <sstream>
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/drivers/gnss/proto/config.pb.h"
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
    : monitor_(MonitorMessageItem::LOCALIZATION),
      map_offset_{FLAGS_map_offset_x, FLAGS_map_offset_y, FLAGS_map_offset_z} {}

Status MSFLocalization::Start() {
  AdapterManager::Init(FLAGS_msf_adapter_config_file);

  Init();

  // start ROS timer, one-shot = false, auto-start = true
  const double duration = 1.0 / FLAGS_localization_publish_freq;
  timer_ = AdapterManager::CreateTimer(ros::Duration(duration),
                                       &MSFLocalization::OnTimer, this);
  common::monitor::MonitorBuffer buffer(&monitor_);

  // GPS
  if (!AdapterManager::GetGps()) {
    buffer.ERROR() << "GPS input not initialized. Check file "
                   << FLAGS_msf_adapter_config_file;
    buffer.PrintLog();
    return Status(common::LOCALIZATION_ERROR, "no GPS adapter");
  }
  AdapterManager::AddGpsCallback(&MSFLocalization::OnGps, this);

  // Imu
  if (!AdapterManager::GetImu()) {
    buffer.ERROR("IMU input not initialized. Check your adapter.conf file!");
    buffer.PrintLog();
    return Status(common::LOCALIZATION_ERROR, "no IMU adapter");
  }
  AdapterManager::AddImuCallback(&MSFLocalization::OnImu, this);

  // Raw Imu
  if (!AdapterManager::GetRawImu()) {
    buffer.ERROR(
        "Raw IMU input not initialized. Check your adapter.conf file!");
    buffer.PrintLog();
    return Status(common::LOCALIZATION_ERROR, "no Raw IMU adapter");
  }
  AdapterManager::AddRawImuCallback(&MSFLocalization::OnRawImu, this);

  // Point Cloud
  CHECK(AdapterManager::GetPointCloud()) << "PointCloud is not initialized.";
  AdapterManager::AddPointCloudCallback(&MSFLocalization::OnPointCloud, this);

  if (FLAGS_gnss_mode == 1) {
    // Gnss Rtk Obs
    CHECK(AdapterManager::GetGnssRtkObs()) << "GnssRtkObs is not initialized.";
    AdapterManager::AddGnssRtkObsCallback(&MSFLocalization::OnGnssRtkObs, this);

    // Gnss Rtk Eph
    CHECK(AdapterManager::GetGnssRtkEph()) << "GnssRtkEph is not initialized.";
    AdapterManager::AddGnssRtkEphCallback(&MSFLocalization::OnGnssRtkEph, this);
  } else {
    // Gnss Best Pose
    CHECK(AdapterManager::GetGnssBestPose())
        << "GnssBestPose is not initialized.";
    AdapterManager::AddGnssBestPoseCallback(&MSFLocalization::OnGnssBestPose,
                                            this);
  }

  return Status::OK();
}

Status MSFLocalization::Stop() {
  timer_.stop();
  return Status::OK();
}

void MSFLocalization::Init() {
  // integration module
  localizaiton_param_.is_ins_can_self_align = FLAGS_integ_ins_can_self_align;
  localizaiton_param_.is_sins_align_with_vel = FLAGS_integ_sins_align_with_vel;
  localizaiton_param_.vel_threshold_get_yaw = FLAGS_vel_threshold_get_yaw;
  localizaiton_param_.integ_debug_log_flag = FLAGS_integ_debug_log_flag;
  localizaiton_param_.is_trans_gpstime_to_utctime =
      FLAGS_trans_gpstime_to_utctime;
  localizaiton_param_.gnss_mode = FLAGS_gnss_mode;
  localizaiton_param_.is_using_raw_gnsspos = true;

  // gnss module
  localizaiton_param_.enable_ins_aid_rtk = FLAGS_enable_ins_aid_rtk;
  localizaiton_param_.enable_auto_save_eph_file =
      FLAGS_enable_auto_save_eph_file;
  localizaiton_param_.eph_buffer_path = FLAGS_eph_buffer_path;

  // lidar module
  localizaiton_param_.map_path = FLAGS_map_dir + "/" + FLAGS_local_map_name;
  localizaiton_param_.lidar_extrinsic_file =
      common::util::TranslatePath(FLAGS_velodyne_extrinsics_path);
  localizaiton_param_.lidar_height_file =
      common::util::TranslatePath(FLAGS_lidar_height_file);
  localizaiton_param_.lidar_debug_log_flag = FLAGS_lidar_debug_log_flag;
  localizaiton_param_.localization_mode = FLAGS_lidar_localization_mode;
  localizaiton_param_.map_coverage_theshold = FLAGS_lidar_map_coverage_theshold;
  localizaiton_param_.imu_lidar_max_delay_time = FLAGS_lidar_imu_max_delay_time;

  std::cerr << "map: " << localizaiton_param_.map_path << std::endl;
  std::cerr << "lidar_extrin: " << localizaiton_param_.lidar_extrinsic_file
            << std::endl;
  std::cerr << "lidar_height: " << localizaiton_param_.lidar_height_file
            << std::endl;

  // common
  localizaiton_param_.utm_zone_id = FLAGS_local_utm_zone_id;
  localizaiton_param_.imu_rate = FLAGS_imu_rate;

    if (1) {
    localizaiton_param_.imu_to_ant_offset.offset_x = 0.0;
    localizaiton_param_.imu_to_ant_offset.offset_y = 0.788;
    localizaiton_param_.imu_to_ant_offset.offset_z = 1.077;
    localizaiton_param_.imu_to_ant_offset.uncertainty_x = 0.05;
    localizaiton_param_.imu_to_ant_offset.uncertainty_y = 0.05;
    localizaiton_param_.imu_to_ant_offset.uncertainty_z = 0.08;
  } else {
    apollo::drivers::gnss::config::Config gnss_config;
    CHECK(common::util::GetProtoFromASCIIFile<
          apollo::drivers::gnss::config::Config>(
        common::util::TranslatePath(FLAGS_gnss_conf_path), &gnss_config));
    CHECK(gnss_config.login_commands_size() > 1);
    std::string login_commands = gnss_config.login_commands(1);
    std::vector<std::string> segmented_login_commands =
        common::util::StringTokenizer::Split(login_commands, " ");
    CHECK(segmented_login_commands.size() == 7);

    std::string name = "";
    double offset_x = 0.0;
    double offset_y = 0.0;
    double offset_z = 0.0;
    double uncertainty_x = 0.0;
    double uncertainty_y = 0.0;
    double uncertainty_z = 0.0;
    std::stringstream ss_str(login_commands);
    ss_str >> name >> offset_x >> offset_y >> offset_z >> uncertainty_x >>
        uncertainty_y >> uncertainty_z;
    localizaiton_param_.imu_to_ant_offset.offset_x = offset_x;
    localizaiton_param_.imu_to_ant_offset.offset_y = offset_y;
    localizaiton_param_.imu_to_ant_offset.offset_z = offset_z;
    localizaiton_param_.imu_to_ant_offset.uncertainty_x = uncertainty_x;
    localizaiton_param_.imu_to_ant_offset.uncertainty_y = uncertainty_y;
    localizaiton_param_.imu_to_ant_offset.uncertainty_z = uncertainty_z;
  }

  localization_integ_.Init(localizaiton_param_);
}

void MSFLocalization::OnTimer(const ros::TimerEvent &event) {}

void MSFLocalization::OnPointCloud(const sensor_msgs::PointCloud2 &message) {
  localization_integ_.PcdProcess(message);

  LocalizaitonMeasureState state;
  IntegMeasure lidar_measure;
  localization_integ_.GetLidarMeasure(state, lidar_measure);

  if (state == LocalizaitonMeasureState::OK) {
    // publish lidar message to debug
    AdapterManager::PublishIntegMeasureLidar(lidar_measure);
  }

  return;
}

void MSFLocalization::OnImu(const localization::Imu &imu_msg) {
  std::cerr << "get imu msg: " << std::endl;
  localization_integ_.RawImuProcess(imu_msg);

  LocalizaitonMeasureState state;
  IntegSinsPva sins_pva;
  LocalizationEstimate integ_localization;
  localization_integ_.GetIntegMeasure(state, sins_pva, integ_localization);

  if (state != LocalizaitonMeasureState::NOT_VALID) {
    // publish sins_pva for debug
    AdapterManager::PublishIntegSinsPva(sins_pva);
  }

  if (state == LocalizaitonMeasureState::OK) {
    AdapterManager::PublishLocalization(integ_localization);
  }

  return;
}

void MSFLocalization::OnRawImu(const drivers::gnss::Imu &imu_msg) {
  std::cerr << "get raw imu msg: " << std::endl;

  std::cerr << "raw imu acc: "
            << imu_msg.linear_acceleration().x() 
            << imu_msg.linear_acceleration().y()
            << imu_msg.linear_acceleration().z() << std::endl;

  std::cerr << "raw imu acc: "
            << imu_msg.angular_velocity().x()
            << imu_msg.angular_velocity().y()
            << imu_msg.angular_velocity().z() << std::endl;
  return;
}

void MSFLocalization::OnGnssBestPose(const GnssBestPose &bestgnsspos_msg) {
  localization_integ_.GnssBestPoseProcess(bestgnsspos_msg);

  return;
}

void MSFLocalization::OnGnssRtkObs(const EpochObservation &raw_obs_msg) {
  localization_integ_.RawObservationProcess(raw_obs_msg);

  LocalizaitonMeasureState state;
  IntegMeasure measure;
  localization_integ_.GetGnssMeasure(state, measure);

  AdapterManager::PublishIntegMeasureGnss(measure);

  return;
}

void MSFLocalization::OnGnssRtkEph(const GnssEphemeris &gnss_orbit_msg) {
  localization_integ_.RawEphemerisProcess(gnss_orbit_msg);

  // gnss_process_.RawEphemerisProcess(gnss_orbit_msg);
  return;
}

void MSFLocalization::OnGps(const localization::Gps &gps_msg) {}

}  // namespace localization
}  // namespace apollo
