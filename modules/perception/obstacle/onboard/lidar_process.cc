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

#include "modules/perception/obstacle/onboard/lidar_process.h"

#include <string>

#include "eigen_conversions/eigen_msg.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/include/ros/ros.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/time/timer.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/common/sequence_type_fuser/sequence_type_fuser.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/lidar/dummy/dummy_algorithms.h"
#include "modules/perception/obstacle/lidar/object_builder/min_box/min_box.h"
#include "modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/hdmap_roi_filter.h"
#include "modules/perception/obstacle/lidar/segmentation/cnnseg/cnn_segmentation.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/hm_tracker.h"

namespace apollo {
namespace perception {

using apollo::common::adapter::AdapterManager;
using Eigen::Affine3d;
using Eigen::Matrix4d;
using pcl_util::Point;
using pcl_util::PointCloud;
using pcl_util::PointCloudPtr;
using pcl_util::PointD;
using pcl_util::PointIndices;
using pcl_util::PointIndicesPtr;

bool LidarProcess::Init() {
  if (inited_) {
    return true;
  }

  RegistAllAlgorithm();

  if (!InitFrameDependence()) {
    AERROR << "failed to Init frame dependence.";
    return false;
  }

  if (!InitAlgorithmPlugin()) {
    AERROR << "failed to Init algorithm plugin.";
    return false;
  }

  inited_ = true;
  return true;
}

bool LidarProcess::Process(const sensor_msgs::PointCloud2& message) {
  PERF_FUNCTION("LidarProcess");
  objects_.clear();
  const double kTimeStamp = message.header.stamp.toSec();
  timestamp_ = kTimeStamp;

  PERF_BLOCK_START();
  /// get velodyne2world transfrom
  std::shared_ptr<Matrix4d> velodyne_trans = std::make_shared<Matrix4d>();
  if (!GetVelodyneTrans(kTimeStamp, velodyne_trans.get())) {
    AERROR << "failed to get trans at timestamp: " << kTimeStamp;
    error_code_ = common::PERCEPTION_ERROR_TF;
    return false;
  }
  ADEBUG << "get trans pose succ.";
  PERF_BLOCK_END("lidar_get_velodyne2world_transfrom");

  PointCloudPtr point_cloud(new PointCloud);
  TransPointCloudToPCL(message, &point_cloud);
  ADEBUG << "transform pointcloud success. points num is: "
         << point_cloud->points.size();
  PERF_BLOCK_END("lidar_transform_poindcloud");

  if (!Process(timestamp_, point_cloud, velodyne_trans)) {
    AERROR << "faile to process msg at timestamp: " << kTimeStamp;
    return false;
  }
  error_code_ = common::OK;
  return true;
}

bool LidarProcess::Process(const double timestamp, PointCloudPtr point_cloud,
                           std::shared_ptr<Matrix4d> velodyne_trans) {
  PERF_BLOCK_START();
  /// call hdmap to get ROI
  HdmapStructPtr hdmap = nullptr;
  if (hdmap_input_) {
    PointD velodyne_pose = {0.0, 0.0, 0.0, 0};  // (0,0,0)
    Affine3d temp_trans(*velodyne_trans);
    PointD velodyne_pose_world = pcl::transformPoint(velodyne_pose, temp_trans);
    hdmap.reset(new HdmapStruct);
    hdmap_input_->GetROI(velodyne_pose_world, FLAGS_map_radius, &hdmap);
    PERF_BLOCK_END("lidar_get_roi_from_hdmap");
  }

  /// call roi_filter
  PointCloudPtr roi_cloud(new PointCloud);
  if (roi_filter_ != nullptr) {
    PointIndicesPtr roi_indices(new PointIndices);
    ROIFilterOptions roi_filter_options;
    roi_filter_options.velodyne_trans = velodyne_trans;
    roi_filter_options.hdmap = hdmap;
    if (roi_filter_->Filter(point_cloud, roi_filter_options,
                            roi_indices.get())) {
      pcl::copyPointCloud(*point_cloud, *roi_indices, *roi_cloud);
      roi_indices_ = roi_indices;
    } else {
      AERROR << "failed to call roi filter.";
      error_code_ = common::PERCEPTION_ERROR_PROCESS;
      return false;
    }
  }
  ADEBUG << "call roi_filter succ. The num of roi_cloud is: "
         << roi_cloud->points.size();
  PERF_BLOCK_END("lidar_roi_filter");

  /// call segmentor
  std::vector<std::shared_ptr<Object>> objects;
  if (segmentor_ != nullptr) {
    SegmentationOptions segmentation_options;
    segmentation_options.origin_cloud = point_cloud;
    PointIndices non_ground_indices;
    non_ground_indices.indices.resize(roi_cloud->points.size());
    // non_ground_indices.indices.resize(point_cloud->points.size());

    std::iota(non_ground_indices.indices.begin(),
              non_ground_indices.indices.end(), 0);
    if (!segmentor_->Segment(roi_cloud, non_ground_indices,
                             segmentation_options, &objects)) {
      AERROR << "failed to call segmention.";
      error_code_ = common::PERCEPTION_ERROR_PROCESS;
      return false;
    }
  }
  ADEBUG << "call segmentation succ. The num of objects is: " << objects.size();
  PERF_BLOCK_END("lidar_segmentation");

  /// call object builder
  if (object_builder_ != nullptr) {
    ObjectBuilderOptions object_builder_options;
    if (!object_builder_->Build(object_builder_options, &objects)) {
      AERROR << "failed to call object builder.";
      error_code_ = common::PERCEPTION_ERROR_PROCESS;
      return false;
    }
  }
  ADEBUG << "call object_builder succ.";
  PERF_BLOCK_END("lidar_object_builder");

  /// call tracker
  if (tracker_ != nullptr) {
    TrackerOptions tracker_options;
    tracker_options.velodyne_trans = velodyne_trans;
    tracker_options.hdmap = hdmap;
    tracker_options.hdmap_input = hdmap_input_;
    if (!tracker_->Track(objects, timestamp, tracker_options, &objects_)) {
      AERROR << "failed to call tracker.";
      error_code_ = common::PERCEPTION_ERROR_PROCESS;
      return false;
    }
  }
  ADEBUG << "call tracker succ, there are " << objects_.size()
         << " tracked objects.";
  PERF_BLOCK_END("lidar_tracker");

  /// call type fuser
  if (type_fuser_ != nullptr) {
    TypeFuserOptions type_fuser_options;
    type_fuser_options.timestamp = timestamp;
    if (!type_fuser_->FuseType(type_fuser_options, &objects_)) {
      AERROR << "failed to call type fuser";
      error_code_ = common::PERCEPTION_ERROR_PROCESS;
      return false;
    }
  }
  ADEBUG << "lidar process succ.";
  PERF_BLOCK_END("lidar_type_fuser");

  return true;
}

void LidarProcess::RegistAllAlgorithm() {
  RegisterFactoryDummyROIFilter();
  RegisterFactoryDummySegmentation();
  RegisterFactoryDummyObjectBuilder();
  RegisterFactoryDummyTracker();
  RegisterFactoryDummyTypeFuser();

  RegisterFactoryHdmapROIFilter();
  RegisterFactoryCNNSegmentation();
  RegisterFactoryMinBoxObjectBuilder();
  RegisterFactoryHmObjectTracker();
  RegisterFactorySequenceTypeFuser();
}

bool LidarProcess::InitFrameDependence() {
  /// init config manager
  ConfigManager* config_manager = ConfigManager::instance();
  if (!config_manager->Init()) {
    AERROR << "failed to Init ConfigManager";
    return false;
  }
  AINFO << "Init config manager successfully, work_root: "
        << config_manager->WorkRoot();

  /// init hdmap
  if (FLAGS_enable_hdmap_input) {
    hdmap_input_ = HDMapInput::instance();
    if (!hdmap_input_) {
      AERROR << "failed to get HDMapInput instance.";
      return false;
    }
    if (!hdmap_input_->Init()) {
      AERROR << "failed to Init HDMapInput";
      return false;
    }
    AINFO << "get and Init hdmap_input succ.";
  }

  /// init roi indices
  roi_indices_ = pcl_util::PointIndicesPtr(new pcl_util::PointIndices);

  return true;
}

bool LidarProcess::InitAlgorithmPlugin() {
  /// init roi filter
  roi_filter_.reset(
      BaseROIFilterRegisterer::GetInstanceByName(FLAGS_onboard_roi_filter));
  if (!roi_filter_) {
    AERROR << "Failed to get instance: " << FLAGS_onboard_roi_filter;
    return false;
  }
  if (!roi_filter_->Init()) {
    AERROR << "Failed to Init roi filter: " << roi_filter_->name();
    return false;
  }
  AINFO << "Init algorithm plugin successfully, roi_filter_: "
        << roi_filter_->name();

  /// init segmentation
  segmentor_.reset(
      BaseSegmentationRegisterer::GetInstanceByName(FLAGS_onboard_segmentor));
  if (!segmentor_) {
    AERROR << "Failed to get instance: " << FLAGS_onboard_segmentor;
    return false;
  }
  if (!segmentor_->Init()) {
    AERROR << "Failed to Init segmentor: " << segmentor_->name();
    return false;
  }
  AINFO << "Init algorithm plugin successfully, segmentor: "
        << segmentor_->name();

  /// init object build
  object_builder_.reset(BaseObjectBuilderRegisterer::GetInstanceByName(
      FLAGS_onboard_object_builder));
  if (!object_builder_) {
    AERROR << "Failed to get instance: " << FLAGS_onboard_object_builder;
    return false;
  }
  if (!object_builder_->Init()) {
    AERROR << "Failed to Init object builder: " << object_builder_->name();
    return false;
  }
  AINFO << "Init algorithm plugin successfully, object builder: "
        << object_builder_->name();

  /// init tracker
  tracker_.reset(
      BaseTrackerRegisterer::GetInstanceByName(FLAGS_onboard_tracker));
  if (!tracker_) {
    AERROR << "Failed to get instance: " << FLAGS_onboard_tracker;
    return false;
  }
  if (!tracker_->Init()) {
    AERROR << "Failed to Init tracker: " << tracker_->name();
    return false;
  }
  AINFO << "Init algorithm plugin successfully, tracker: " << tracker_->name();

  /// init type fuser
  type_fuser_.reset(
      BaseTypeFuserRegisterer::GetInstanceByName(FLAGS_onboard_type_fuser));
  if (!type_fuser_) {
    AERROR << "Failed to get instance: " << FLAGS_onboard_type_fuser;
    return false;
  }
  if (!type_fuser_->Init()) {
    AERROR << "Failed to Init type_fuser: " << type_fuser_->name();
    return false;
  }
  AINFO << "Init algorithm plugin successfully, type_fuser: "
        << type_fuser_->name();

  return true;
}

void LidarProcess::TransPointCloudToPCL(const sensor_msgs::PointCloud2& in_msg,
                                        PointCloudPtr* out_cloud) {
  // transform from ros to pcl
  pcl::PointCloud<pcl_util::PointXYZIT> in_cloud;
  pcl::fromROSMsg(in_msg, in_cloud);
  // transform from xyzit to xyzi
  PointCloudPtr& cloud = *out_cloud;
  cloud->header = in_cloud.header;
  cloud->width = in_cloud.width;
  cloud->height = in_cloud.height;
  cloud->is_dense = in_cloud.is_dense;
  cloud->sensor_origin_ = in_cloud.sensor_origin_;
  cloud->sensor_orientation_ = in_cloud.sensor_orientation_;
  cloud->points.resize(in_cloud.points.size());
  size_t points_num = 0;
  for (size_t idx = 0; idx < in_cloud.size(); ++idx) {
    pcl_util::PointXYZIT& pt = in_cloud.points[idx];
    if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z) &&
        !std::isnan(pt.intensity)) {
      cloud->points[points_num].x = pt.x;
      cloud->points[points_num].y = pt.y;
      cloud->points[points_num].z = pt.z;
      cloud->points[points_num].intensity = pt.intensity;
      ++points_num;
    }
  }
  cloud->points.resize(points_num);
}

bool LidarProcess::GetVelodyneTrans(const double query_time, Matrix4d* trans) {
  if (!trans) {
    AERROR << "failed to get trans, the trans ptr can not be nullptr";
    return false;
  }

  ros::Time query_stamp(query_time);
  const auto& tf2_buffer = AdapterManager::Tf2Buffer();

  const double kTf2BuffSize = FLAGS_tf2_buff_in_ms / 1000.0;
  std::string err_msg;
  if (!tf2_buffer.canTransform(FLAGS_lidar_tf2_frame_id,
                               FLAGS_lidar_tf2_child_frame_id, query_stamp,
                               ros::Duration(kTf2BuffSize), &err_msg)) {
    AERROR << "Cannot transform frame: " << FLAGS_lidar_tf2_frame_id
           << " to frame " << FLAGS_lidar_tf2_child_frame_id
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

void LidarProcess::GeneratePbMsg(PerceptionObstacles* obstacles) {
  common::Header* header = obstacles->mutable_header();
  header->set_lidar_timestamp(timestamp_ * 1e9);  // in ns
  header->set_camera_timestamp(0);
  header->set_radar_timestamp(0);

  obstacles->set_error_code(error_code_);

  for (const auto& obj : objects_) {
    PerceptionObstacle* obstacle = obstacles->add_perception_obstacle();
    obj->Serialize(obstacle);
    obstacle->set_timestamp(obstacle->timestamp() * 1000);
  }

  ADEBUG << "PerceptionObstacles: " << obstacles->ShortDebugString();
}

}  // namespace perception
}  // namespace apollo
