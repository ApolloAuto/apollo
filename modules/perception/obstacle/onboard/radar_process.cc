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

#include "modules/perception/obstacle/onboard/radar_process.h"

#include <string>

#include "Eigen/Core"
#include "eigen_conversions/eigen_msg.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/include/ros/ros.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/base/timer.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/radar/dummy/dummy_algorithms.h"

namespace apollo {
namespace perception {

using apollo::common::adapter::AdapterManager;
using pcl_util::Point;
using pcl_util::PointD;
using Eigen::Matrix4d;
using Eigen::Affine3d;
using std::string;

bool RadarProcess::Init() {
  if (inited_) {
    return true;
  }

  RegistAllAlgorithm();

  if (!InitFrameDependence()) {
    AERROR << "failed to init frame dependence.";
    return false;
  }

  if (!InitAlgorithmPlugin()) {
    AERROR << "failed to init algorithm plugin.";
    return false;
  }

  inited_ = true;
  return true;
}

bool RadarProcess::Process(const RadarObsArray& radar_obs_proto) {
  PERF_FUNCTION("RAdarProcess");
    double timestamp = radar_obs_proto.measurement_time();
    double unix_timestamp = timestamp;
    const double cur_time = common::time::Clock::NowInSecond();
    const double start_latency = (cur_time - unix_timestamp) * 1e3;
    AINFO << "FRAME_STATISTICS:Radar:Start:msg_time[" << GLOG_TIMESTAMP(timestamp)
               << "]:cur_time[" << GLOG_TIMESTAMP(cur_time) << "]:cur_latency["
               << start_latency << "]";
    // correct radar timestamp
    if (radar_obs_proto.type() == apollo::drivers::CONTINENTAL_ARS_40821){
        timestamp -= 0.07;
    } else {
        AERROR << "Unknown sensor type";
        return false;
    }

    if (fabs(timestamp - 0.0) < 10e-6) {
        AERROR << "Error timestamp: " << GLOG_TIMESTAMP(timestamp);
        return false;
    }
    ADEBUG << "recv radar msg: [timestamp: " << GLOG_TIMESTAMP(timestamp)
            << " num_raw_obstacles: " << radar_obs_proto.delphiobs_size() << "]";

  std::shared_ptr<Matrix4d> radar2world_pose = std::make_shared<Matrix4d>();
  if (!GetRadarTrans(timestamp, radar2world_pose.get())) {
    AERROR << "failed to get trans at timestamp: " << timestamp;
    error_code_ = common::PERCEPTION_ERROR_TF;
    return false;
  }

    // Current Localiztion, using 64-velodyne postion.
    PointD position;
    position.x = (*radar2world_pose)(0,3);
    position.y = (*radar2world_pose)(1,3);
    position.z = (*radar2world_pose)(2,3);
    // 2. Get map polygons.
    std::vector<PolygonDType> map_polygons;
    HdmapStructPtr hdmap(new HdmapStruct);
    if (FLAGS_enable_hdmap_input && hdmap_input_ &&
            !hdmap_input_->GetROI(position, FLAGS_front_radar_forward_distance, &hdmap)) {
        AWARN << "Failed to get roi. timestamp: " << GLOG_TIMESTAMP(timestamp)
                     << " position: [" << position.x << ", " << position.y
                     << ", " << position.z << "]";
        // NOTE: if call hdmap failed, using empty map_polygons.
    }
    RadarDetectorOptions options;
    // // 3. get car car_linear_speed
    // if (!get_car_linear_speed(timestamp, &(options.car_linear_speed))) {
    //     XLOG(ERROR) << "Failed to call get_car_linear_speed. [timestamp: "
    //                << GLOG_TIMESTAMP(timestamp);
    //     return;
    // }
    // 4. Call RadarDetector::detect.
    PERF_BLOCK_START();
    options.radar2world_pose = &(*radar2world_pose);
    std::shared_ptr<SensorObjects> radar_objects(new SensorObjects);
    bool result = radar_detector_->Detect(
            radar_obs_proto,
            map_polygons,
            options,
            &radar_objects->objects);
    if (!result) {
        AERROR << "Failed to call RadarDetector. [timestamp: "
                   << GLOG_TIMESTAMP(timestamp)
                   << ", map_polygons_size: " << map_polygons.size()
                   << ", num_raw_conti_obstacles: " << radar_obs_proto.contiobs_size() << "]";
        return false;
    }
    radar_objects->timestamp = timestamp;
    radar_objects->sensor_type = RADAR;
    PERF_BLOCK_END("radar_detect");

    const double end_timestamp = common::time::Clock::NowInSecond();
    const double end_latency = (end_timestamp - unix_timestamp) * 1e3;
    AINFO << "FRAME_STATISTICS:Radar:End:msg_time["
           << GLOG_TIMESTAMP(timestamp)
           << "]:cur_time[" << GLOG_TIMESTAMP(end_timestamp)
           << "]:cur_latency[" << end_latency << "]";
  return true;
}

void RadarProcess::RegistAllAlgorithm() {
  RegisterFactoryDummyRadarDetector();
  RegisterFactoryModestRadarDetector();
}

bool RadarProcess::InitFrameDependence() {
  /// init config manager
  ConfigManager* config_manager = ConfigManager::instance();
  if (!config_manager->Init()) {
    AERROR << "failed to init ConfigManager";
    return false;
  }
  AINFO << "Init config manager successfully, work_root: "
        << config_manager->work_root();

  /// init hdmap
  if (FLAGS_enable_hdmap_input) {
    hdmap_input_ = HDMapInput::instance();
    if (!hdmap_input_) {
      AERROR << "failed to get HDMapInput instance.";
      return false;
    }
    if (!hdmap_input_->Init()) {
      AERROR << "failed to init HDMapInput";
      return false;
    }
    AINFO << "get and init hdmap_input succ.";
  }

  return true;
}

bool RadarProcess::InitAlgorithmPlugin() {
  /// init radar detecor
  radar_detector_.reset(
      BaseRadarDetectorRegisterer::GetInstanceByName(FLAGS_onboard_radar_detector));
  if (!radar_detector_) {
    AERROR << "Failed to get instance: " << FLAGS_onboard_radar_detector;
    return false;
  }
  if (!radar_detector_->Init()) {
    AERROR << "Failed to init tracker: " << radar_detector_->name();
    return false;
  }
  AINFO << "Init algorithm plugin successfully, radar detecor: " << radar_detector_->name();

  return true;
}

bool RadarProcess::GetRadarTrans(const double query_time, Matrix4d* trans) {
  if (!trans) {
    AERROR << "failed to get trans, the trans ptr can not be NULL";
    return false;
  }

  ros::Time query_stamp(query_time);
  const auto& tf2_buffer = AdapterManager::Tf2Buffer();

  const double kTf2BuffSize = FLAGS_tf2_buff_in_ms / 1000.0;
  string err_msg;
  if (!tf2_buffer.canTransform(FLAGS_radar_tf2_frame_id,
                               FLAGS_radar_tf2_child_frame_id, query_stamp,
                               ros::Duration(kTf2BuffSize), &err_msg)) {
    AERROR << "Cannot transform frame: " << FLAGS_radar_tf2_frame_id
           << " to frame " << FLAGS_radar_tf2_child_frame_id
           << " , err: " << err_msg
           << ". Frames: " << tf2_buffer.allFramesAsString();
    return false;
  }

  geometry_msgs::TransformStamped transform_stamped;
  try {
    transform_stamped = tf2_buffer.lookupTransform(
        FLAGS_lidar_tf2_frame_id, FLAGS_lidar_tf2_child_frame_id, query_stamp);
  } catch (tf2::TransformException& ex) {
    AERROR << "Exception: " << ex.what();
    return false;
  }
  Affine3d affine_3d;
  tf::transformMsgToEigen(transform_stamped.transform, affine_3d);
  *trans = affine_3d.matrix();

  ADEBUG << "get " << FLAGS_lidar_tf2_frame_id << " to "
         << FLAGS_lidar_tf2_child_frame_id << " trans: " << *trans;
  return true;
}

bool RadarProcess::GeneratePbMsg(PerceptionObstacles* obstacles) {
  AdapterManager::FillPerceptionObstaclesHeader(FLAGS_obstacle_module_name,
                                                obstacles);
  common::Header* header = obstacles->mutable_header();
  header->set_lidar_timestamp(timestamp_ * 1e9);  // in ns
  header->set_camera_timestamp(0);
  header->set_radar_timestamp(0);

  obstacles->set_error_code(error_code_);

  for (const auto& obj : objects_) {
    PerceptionObstacle* obstacle = obstacles->add_perception_obstacle();
    if (!obj->Serialize(obstacle)) {
      AERROR << "Failed gen PerceptionObstacle. Object:" << obj->ToString();
      return false;
    }
    obstacle->set_timestamp(obstacle->timestamp() * 1000);
  }

  ADEBUG << "PerceptionObstacles: " << obstacles->ShortDebugString();
  return true;
}

}  // namespace perception
}  // namespace apollo
