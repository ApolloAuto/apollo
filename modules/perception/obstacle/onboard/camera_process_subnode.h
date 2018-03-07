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

#ifndef MODULES_PERCEPTION_OBSTACLE_ONBORAD_LIDAR_PROCESS_SUBNODE_H_
#define MODULES_PERCEPTION_OBSTACLE_ONBORAD_LIDAR_PROCESS_SUBNODE_H_

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "sensor_msgs/Image.h"

#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/camera/interface/base_camera_converter.h"
#include "modules/perception/obstacle/camera/interface/base_camera_detector.h"
#include "modules/perception/obstacle/camera/interface/base_camera_filter.h"
#include "modules/perception/obstacle/camera/interface/base_camera_tracker.h"
#include "modules/perception/obstacle/camera/interface/base_camera_transformer.h"
#include "modules/perception/obstacle/camera/interface/base_lane_post_processor.h"
#include "modules/perception/obstacle/onboard/object_shared_data.h"
#include "modules/perception/onboard/subnode.h"

#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "lib/base/file_util.h"
#include "lib/base/macros.h"
#include "lib/base/perf.h"
#include "lib/base/time_util.h"
#include "lib/config_manager/config_manager.h"
#include "onboard/event_manager.h"
#include "onboard/shared_data_manager.h"
#include "onboard/types.h"

namespace apollo {
namespace perception {

class CameraProcessSubnode : public Subnode {
 public:
  CameraProcessSubnode() = default;
  ~CameraProcessSubnode() = default;

  apollo::common::Status ProcEvents() override {
    return apollo::common::Status::OK();
  }

 private:
  bool InitInternal() override;

  void OnPointCloud(const sensor_msgs::PointCloud2& message);

  void RegistAllAlgorithm();
  bool InitFrameDependence();
  bool InitAlgorithmPlugin();

  void PublishDataAndEvent(double timestamp,
                           const SharedDataPtr<SensorObjects>& data);

  bool inited_ = false;
  float timestamp_ = 0.0f;
  SeqId seq_num_ = 0;
  common::ErrorCode error_code_ = common::OK;
  LidarObjectData* processing_data_ = nullptr;
  std::string device_id_;

  std::unique_ptr<BaseCameraDetector> detector_;
  std::unique_ptr<BaseCameraConverter> converter_;
  std::unique_ptr<BaseCameraTracker> tracker_;
  std::unique_ptr<BaseCameraTransformer> transformer_;
  std::unique_ptr<BaseCameraFilter> filter_;
  std::unique_ptr<BaseCameraLanePostProcessor> lane_processor_;

  bool init_shared_data();
  bool init_alg_plugins();
  bool init_work_root();
  bool init_calibration_input(
      const std::map<std::string, std::string>& reserve_field_map);
  bool init_subscriber(
      const std::map<std::string, std::string>& reserve_field_map);
  bool resize_image(const sensor_msgs::Image::ConstPtr& image_message_src,
                    sensor_msgs::Image::Ptr image_message);
  void image_callback(const sensor_msgs::Image::ConstPtr& image_message);
  void image_process(const cv::Mat& image_mat_src, double time_stamp);

  bool trans_message_to_cv_mat(const sensor_msgs::Image::ConstPtr& image_msg,
                               cv::Mat* mat);

  bool get_camera_car_trans(double timestamp,
                            Eigen::Matrix4d* camera_to_car_pose);

  void trans_visualobject_to_sensorobject(
      const std::vector<VisualObjectPtr> track_objects,
      onboard::SharedDataPtr<SensorObjects>* sensor_object);

  void update_l3_sensorobject_elements(
      onboard::SharedDataPtr<SensorObjects>* sensor_objects);

  void publish_data_and_event(
      double timestamp,
      const onboard::SharedDataPtr<SensorObjects>& sensor_object,
      const onboard::SharedDataPtr<CameraItem>& camera_item);

  CameraObjectData* _camera_object_data = nullptr;  // release by framework
  CameraSharedData* _camera_shared_data = nullptr;  // release by framework

  adu::perception::config_manager::CameraUndistortionPtr _undistortion_handler;
  Eigen::Matrix4d _camera_to_car_mat;
  Eigen::Matrix<double, 3, 4> _camera_intrinsic;  // camera intrinsic

  std::unique_ptr<BaseCameraDetector> _camera_detector;
  std::unique_ptr<BaseCameraTransformer> _camera_transformer;
  std::unique_ptr<BaseCameraTracker> _camera_tracker;
  std::unique_ptr<BaseCameraParser> _lane_camera_parser;
  std::string _device_id;
  std::string _work_root_dir;
  double _msg_average_latency = 0.0;
  int _msg_count_in_stat_window = 0;
  uint64_t _total_msg_count = 0;
  base::Mutex _mutex;  // Protect _seq_num.
  SeqId _seq_num = 0;

  onboard::StreamInput<sensor_msgs::Image, MixDetectorSubnode> _stream_input;
};

REGISTER_SUBNODE(CameraProcessSubnode);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBORAD_LIDAR_PROCESS_SUBNODE_H_
