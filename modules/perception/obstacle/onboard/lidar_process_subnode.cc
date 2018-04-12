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

#include "modules/perception/obstacle/onboard/lidar_process_subnode.h"

#include <unordered_map>

#include "eigen_conversions/eigen_msg.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/include/ros/ros.h"

#include "modules/common/log.h"
#include "modules/common/time/time_util.h"
#include "modules/common/time/timer.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/common/sequence_type_fuser/sequence_type_fuser.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/lidar/dummy/dummy_algorithms.h"
#include "modules/perception/obstacle/lidar/object_builder/min_box/min_box.h"
#include "modules/perception/obstacle/lidar/object_filter/low_object_filter/low_object_filter.h"
#include "modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/hdmap_roi_filter.h"
#include "modules/perception/obstacle/lidar/segmentation/cnnseg/cnn_segmentation.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/hm_tracker.h"
#include "modules/perception/onboard/subnode_helper.h"
#include "modules/perception/onboard/transform_input.h"

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

bool LidarProcessSubnode::InitInternal() {
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
  // parse reserve fileds
  std::unordered_map<std::string, std::string> reserve_field_map;
  if (!SubnodeHelper::ParseReserveField(reserve_, &reserve_field_map)) {
    AERROR << "Failed to parse reserve filed: " << reserve_;
    return false;
  }

  if (reserve_field_map.find("device_id") == reserve_field_map.end()) {
    AERROR << "Failed to find field device_id, reserve: " << reserve_;
    return false;
  }
  device_id_ = reserve_field_map["device_id"];

  CHECK(AdapterManager::GetPointCloud()) << "PointCloud is not initialized.";
  AdapterManager::AddPointCloudCallback(&LidarProcessSubnode::OnPointCloud,
                                        this);
  inited_ = true;

  return true;
}

void LidarProcessSubnode::OnPointCloud(
    const sensor_msgs::PointCloud2& message) {
  AINFO << "process OnPointCloud.";
  PERF_FUNCTION("LidarProcessSubnode");
  if (!inited_) {
    AERROR << "the LidarProcessSubnode has not been Init";
    return;
  }
  const double kTimeStamp = message.header.stamp.toSec();
  timestamp_ = kTimeStamp;
  ++seq_num_;

  std::shared_ptr<SensorObjects> out_sensor_objects(new SensorObjects);
  out_sensor_objects->timestamp = timestamp_;
  out_sensor_objects->sensor_type = SensorType::VELODYNE_64;
  out_sensor_objects->sensor_id = device_id_;
  out_sensor_objects->seq_num = seq_num_;

  PERF_BLOCK_START();
  /// get velodyne2world transfrom
  std::shared_ptr<Matrix4d> velodyne_trans = std::make_shared<Matrix4d>();
  if (!GetVelodyneTrans(kTimeStamp, velodyne_trans.get())) {
    AERROR << "failed to get trans at timestamp: "
           << GLOG_TIMESTAMP(kTimeStamp);
    out_sensor_objects->error_code = common::PERCEPTION_ERROR_TF;
    PublishDataAndEvent(timestamp_, out_sensor_objects);
    return;
  }
  out_sensor_objects->sensor2world_pose = *velodyne_trans;
  AINFO << "get lidar trans pose succ. pose: \n" << *velodyne_trans;
  PERF_BLOCK_END("lidar_get_velodyne2world_transfrom");

  PointCloudPtr point_cloud(new PointCloud);
  TransPointCloudToPCL(message, &point_cloud);
  ADEBUG << "transform pointcloud success. points num is: "
         << point_cloud->points.size();
  PERF_BLOCK_END("lidar_transform_poindcloud");

  // error_code_ = common::OK;

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
      out_sensor_objects->error_code = common::PERCEPTION_ERROR_PROCESS;
      PublishDataAndEvent(timestamp_, out_sensor_objects);
      return;
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
      out_sensor_objects->error_code = common::PERCEPTION_ERROR_PROCESS;
      PublishDataAndEvent(timestamp_, out_sensor_objects);
      return;
    }
  }
  ADEBUG << "call segmentation succ. The num of objects is: " << objects.size();
  PERF_BLOCK_END("lidar_segmentation");

  /// call object filter
  if (object_filter_ != nullptr) {
    ObjectFilterOptions object_filter_options;
    object_filter_options.velodyne_trans.reset(new Eigen::Matrix4d);
    object_filter_options.velodyne_trans = velodyne_trans;
    // object_filter_options.hdmap_struct_ptr = hdmap;

    if (!object_filter_->Filter(object_filter_options, &objects)) {
      AERROR << "failed to call object filter.";
      out_sensor_objects->error_code = common::PERCEPTION_ERROR_PROCESS;
      PublishDataAndEvent(timestamp_, out_sensor_objects);
      return;
    }
  }
  ADEBUG << "call object filter succ. The num of objects is: "
         << objects.size();
  PERF_BLOCK_END("lidar_object_filter");

  /// call object builder
  if (object_builder_ != nullptr) {
    ObjectBuilderOptions object_builder_options;
    if (!object_builder_->Build(object_builder_options, &objects)) {
      AERROR << "failed to call object builder.";
      out_sensor_objects->error_code = common::PERCEPTION_ERROR_PROCESS;
      PublishDataAndEvent(timestamp_, out_sensor_objects);
      return;
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
    if (!tracker_->Track(objects, timestamp_, tracker_options,
                         &(out_sensor_objects->objects))) {
      AERROR << "failed to call tracker.";
      out_sensor_objects->error_code = common::PERCEPTION_ERROR_PROCESS;
      PublishDataAndEvent(timestamp_, out_sensor_objects);
      return;
    }
  }
  ADEBUG << "call tracker succ, there are "
         << out_sensor_objects->objects.size() << " tracked objects.";
  PERF_BLOCK_END("lidar_tracker");

  /// call type fuser
  if (type_fuser_ != nullptr) {
    TypeFuserOptions type_fuser_options;
    type_fuser_options.timestamp = timestamp_;
    if (!type_fuser_->FuseType(type_fuser_options,
                               &(out_sensor_objects->objects))) {
      out_sensor_objects->error_code = common::PERCEPTION_ERROR_PROCESS;
      PublishDataAndEvent(timestamp_, out_sensor_objects);
      return;
    }
  }
  ADEBUG << "lidar process succ.";
  PERF_BLOCK_END("lidar_type_fuser");

  PublishDataAndEvent(timestamp_, out_sensor_objects);
  return;
}

void LidarProcessSubnode::RegistAllAlgorithm() {
  RegisterFactoryDummyROIFilter();
  RegisterFactoryDummySegmentation();
  RegisterFactoryDummyObjectBuilder();
  RegisterFactoryDummyTracker();
  RegisterFactoryDummyTypeFuser();

  RegisterFactoryHdmapROIFilter();
  RegisterFactoryLowObjectFilter();
  RegisterFactoryCNNSegmentation();
  RegisterFactoryMinBoxObjectBuilder();
  RegisterFactoryHmObjectTracker();
  RegisterFactorySequenceTypeFuser();
}

bool LidarProcessSubnode::InitFrameDependence() {
  /// init share data
  CHECK(shared_data_manager_ != nullptr);
  // init preprocess_data
  const std::string lidar_processing_data_name("LidarObjectData");
  processing_data_ = dynamic_cast<LidarObjectData*>(
      shared_data_manager_->GetSharedData(lidar_processing_data_name));
  if (processing_data_ == nullptr) {
    AERROR << "Failed to get shared data instance "
           << lidar_processing_data_name;
    return false;
  }
  AINFO << "Init shared data successfully, data: " << processing_data_->name();

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

  return true;
}

bool LidarProcessSubnode::InitAlgorithmPlugin() {
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

  /// init pre object filter
  object_filter_.reset(
      BaseObjectFilterRegisterer::GetInstanceByName("LowObjectFilter"));
  if (!object_filter_) {
    AERROR << "Failed to get instance: ExtHdmapObjectFilter";
    return false;
  }
  if (!object_filter_->Init()) {
    AERROR << "Failed to Init object filter: " << object_filter_->name();
    return false;
  }
  AINFO << "Init algorithm plugin successfully, object filter: "
        << object_filter_->name();

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

void LidarProcessSubnode::TransPointCloudToPCL(
    const sensor_msgs::PointCloud2& in_msg, PointCloudPtr* out_cloud) {
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

void LidarProcessSubnode::PublishDataAndEvent(
    double timestamp, const SharedDataPtr<SensorObjects>& data) {
  // set shared data
  std::string key;
  if (!SubnodeHelper::ProduceSharedDataKey(timestamp, device_id_, &key)) {
    AERROR << "Failed to produce shared key. time: "
           << GLOG_TIMESTAMP(timestamp) << ", device_id: " << device_id_;
    return;
  }

  processing_data_->Add(key, data);
  // pub events
  for (size_t idx = 0; idx < pub_meta_events_.size(); ++idx) {
    const EventMeta& event_meta = pub_meta_events_[idx];
    Event event;
    event.event_id = event_meta.event_id;
    event.timestamp = timestamp;
    event.reserve = device_id_;
    event_manager_->Publish(event);
  }
}

}  // namespace perception
}  // namespace apollo
