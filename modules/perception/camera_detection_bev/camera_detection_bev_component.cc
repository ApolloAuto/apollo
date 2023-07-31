/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/camera_detection_bev/camera_detection_bev_component.h"

#include <string>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"
#include "modules/perception/common/base/camera.h"

namespace apollo {
namespace perception {
namespace camera {

bool CameraDetectionBevComponent::InitDetector(
    const CameraDetectionBEV& detection_param) {
  ObstacleDetectorInitOptions init_options;
  // Init conf file
  auto plugin_param = detection_param.plugin_param();
  init_options.config_path = plugin_param.config_path();
  init_options.config_file = plugin_param.config_file();
  init_options.gpu_id = detection_param.gpu_id();

  // Init camera params
  std::string camera_name = detection_param.camera_name();
  base::BaseCameraModelPtr model =
      algorithm::SensorManager::Instance()->GetUndistortCameraModel(
          camera_name);
  auto pinhole = static_cast<base::PinholeCameraModel*>(model.get());
  init_options.intrinsic = pinhole->get_intrinsic_params();
  init_options.image_height = model->get_height();
  init_options.image_width = model->get_width();
  image_height_ = model->get_height();
  image_width_ = model->get_width();

  // Init detector
  detector_.reset(
      BaseObstacleDetectorRegisterer::GetInstanceByName(plugin_param.name()));
  detector_->Init(init_options);
  return true;
}

bool CameraDetectionBevComponent::InitCameraFrame(
    const CameraDetectionBEV& detection_param) {
  DataProvider::InitOptions init_options;
  init_options.image_height = image_height_;
  init_options.image_width = image_width_;
  init_options.do_undistortion = detection_param.enable_undistortion();
  init_options.sensor_name = detection_param.camera_name();
  init_options.device_id = detection_param.gpu_id();
  AINFO << "init_options.device_id: " << init_options.device_id
        << " camera_name: " << init_options.sensor_name;

  frame_ptr_ = std::make_shared<CameraFrame>();
  int num = detection_param.channel().input_camera_channel_name().size();
  frame_ptr_->data_provider.resize(num);
  for (int i = 0; i < num; ++i) {
    frame_ptr_->data_provider[i] = std::make_shared<DataProvider>();
    frame_ptr_->data_provider[i]->Init(init_options);
  }
  return true;
}

bool CameraDetectionBevComponent::InitListeners(
    const CameraDetectionBEV& detection_param) {
  for (const auto& channel :
       detection_param.channel().input_camera_channel_name()) {
    auto reader = node_->CreateReader<drivers::Image>(channel);
    readers_.emplace_back(reader);
  }
  return true;
}

bool CameraDetectionBevComponent::InitTransformWrapper(
    const CameraDetectionBEV& detection_param) {
  trans_wrapper_.reset(new onboard::TransformWrapper());
  // tf_camera_frame_id
  trans_wrapper_->Init(detection_param.frame_id());
  return true;
}

bool CameraDetectionBevComponent::Init() {
  CameraDetectionBEV detection_param;
  if (!GetProtoConfig(&detection_param)) {
    AERROR << "Load camera detection 3d component config failed!";
    return false;
  }

  InitCameraFrame(detection_param);
  InitListeners(detection_param);
  InitDetector(detection_param);
  InitTransformWrapper(detection_param);

  writer_ = node_->CreateWriter<CameraFrame>(
      detection_param.channel().output_obstacles_channel_name());
  return true;
}

void CameraDetectionBevComponent::CameraToWorldCoor(
    const Eigen::Affine3d& camera2world, std::vector<base::ObjectPtr>* objs) {
  for (auto& obj : *objs) {
    Eigen::Vector3d local_center =
        obj->camera_supplement.local_center.cast<double>();
    obj->center = camera2world * local_center;

    obj->direction = camera2world.linear().cast<float>() * obj->direction;
    obj->theta = std::atan2(obj->direction(1), obj->direction(0));
  }
}

bool CameraDetectionBevComponent::Proc(
    const std::shared_ptr<drivers::Image>& msg) {
  const double msg_timestamp = msg->measurement_time() + timestamp_offset_;

  for (size_t i = 0; i < readers_.size(); ++i) {
    readers_[i]->Observe();
    const auto& camera_msg = readers_[i]->GetLatestObserved();
    frame_ptr_->data_provider[i]->FillImageData(
        image_height_, image_width_,
        reinterpret_cast<const uint8_t*>(camera_msg->data().data()),
        camera_msg->encoding());
  }

  frame_ptr_->timestamp = msg_timestamp;
  ++frame_ptr_->frame_id;

  // Detect
  detector_->Detect(frame_ptr_.get());

  // Get sensor to world pose from TF
  Eigen::Affine3d lidar2world_trans;
  if (!trans_wrapper_->GetSensor2worldTrans(msg_timestamp,
                                            &lidar2world_trans)) {
    const std::string err_str =
        absl::StrCat("failed to get camera to world pose, ts: ", msg_timestamp,
                     " camera_name: ", msg->frame_id());
    AERROR << err_str;
  }

  CameraToWorldCoor(lidar2world_trans, &frame_ptr_->detected_objects);

  // Send msg
  writer_->Write(frame_ptr_);

  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
