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
#include "cyber/time/clock.h"
#include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"
#include "modules/perception/common/base/camera.h"

namespace apollo {
namespace perception {
namespace camera {

bool CameraDetectionBevComponent::InitDetector(
    const CameraDetectionBEV &detection_param) {
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
  ACHECK(model) << "Can't find " << camera_name
                << " in data/conf/sensor_meta.pb.txt";
  auto pinhole = static_cast<base::PinholeCameraModel *>(model.get());
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
    const CameraDetectionBEV &detection_param) {
  DataProvider::InitOptions init_options;
  init_options.image_height = image_height_;
  init_options.image_width = image_width_;
  init_options.do_undistortion = detection_param.enable_undistortion();
  init_options.sensor_name = detection_param.camera_name();
  init_options.device_id = detection_param.gpu_id();
  AINFO << "init_options.device_id: " << init_options.device_id
        << " camera_name: " << init_options.sensor_name;

  frame_ptr_ = std::make_shared<CameraFrame>();
  int num = 6;
  frame_ptr_->data_provider.resize(num);
  for (int i = 0; i < num; ++i) {
    frame_ptr_->data_provider[i] = std::make_shared<DataProvider>();
    frame_ptr_->data_provider[i]->Init(init_options);
  }
  return true;
}

bool CameraDetectionBevComponent::InitListeners(
    const CameraDetectionBEV &detection_param) {
  for (const auto &channel :
       detection_param.channel().input_camera_channel_name()) {
    std::shared_ptr<cyber::Reader<drivers::Image>> reader;
    if (channel == "/apollo/sensor/camera/CAM_BACK/image") {
      reader = node_->CreateReader<drivers::Image>(
          channel, [&](const std::shared_ptr<drivers::Image> &msg) {
            OnReceiveImage(msg);
          });
    } else {
      reader = node_->CreateReader<drivers::Image>(channel);
    }
    readers_.emplace_back(reader);
  }
  return true;
}

bool CameraDetectionBevComponent::InitTransformWrapper(
    const CameraDetectionBEV &detection_param) {
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

  // todo(zero): need init image_height_\image_width_
  InitDetector(detection_param);
  InitCameraFrame(detection_param);
  InitListeners(detection_param);
  InitTransformWrapper(detection_param);

  writer_ = node_->CreateWriter<PerceptionObstacles>(
      detection_param.channel().output_obstacles_channel_name());
  return true;
}

void CameraDetectionBevComponent::CameraToWorldCoor(
    const Eigen::Affine3d &camera2world, std::vector<base::ObjectPtr> *objs) {
  for (auto &obj : *objs) {
    Eigen::Vector3d local_center =
        obj->camera_supplement.local_center.cast<double>();
    obj->center = camera2world * local_center;

    obj->direction = camera2world.linear().cast<float>() * obj->direction;
    obj->theta = std::atan2(obj->direction(1), obj->direction(0));
  }
}

int CameraDetectionBevComponent::ConvertObjectToPb(
    const base::ObjectPtr &object_ptr, PerceptionObstacle *pb_msg) {
  if (!object_ptr || !pb_msg) {
    return cyber::FAIL;
  }
  pb_msg->set_id(object_ptr->track_id);

  apollo::common::Point3D *obj_center = pb_msg->mutable_position();
  obj_center->set_x(object_ptr->center(0));
  obj_center->set_y(object_ptr->center(1));
  obj_center->set_z(object_ptr->center(2));

  pb_msg->set_length(object_ptr->size(0));
  pb_msg->set_width(object_ptr->size(1));
  pb_msg->set_height(object_ptr->size(2));

  pb_msg->set_theta(object_ptr->theta);

  // for camera results, set object's center as anchor point
  pb_msg->set_type(static_cast<PerceptionObstacle::Type>(object_ptr->type));
  pb_msg->set_sub_type(
      static_cast<PerceptionObstacle::SubType>(object_ptr->sub_type));
  pb_msg->set_timestamp(object_ptr->latest_tracked_time);  // in seconds.

  pb_msg->set_height_above_ground(object_ptr->size(2));

  return cyber::SUCC;
}

int CameraDetectionBevComponent::MakeProtobufMsg(
    double msg_timestamp, int seq_num,
    const std::vector<base::ObjectPtr> &objects,
    PerceptionObstacles *obstacles) {
  double publish_time = apollo::cyber::Clock::NowInSeconds();
  auto header = obstacles->mutable_header();
  header->set_timestamp_sec(publish_time);
  header->set_module_name("perception_camera");
  header->set_sequence_num(seq_num);
  // in nanosecond
  // PnC would use lidar timestamp to predict
  header->set_lidar_timestamp(static_cast<uint64_t>(msg_timestamp * 1e9));
  header->set_camera_timestamp(static_cast<uint64_t>(msg_timestamp * 1e9));

  // write out obstacles in world coordinates
  int count = 0;
  for (const auto &obj : objects) {
    obj->track_id = count++;
    PerceptionObstacle *obstacle = obstacles->add_perception_obstacle();
    if (ConvertObjectToPb(obj, obstacle) != cyber::SUCC) {
      AERROR << "ConvertObjectToPb failed, Object:" << obj->ToString();
      return cyber::FAIL;
    }
  }

  return cyber::SUCC;
}

bool CameraDetectionBevComponent::OnReceiveImage(
    const std::shared_ptr<drivers::Image> &msg) {
  const double msg_timestamp = msg->measurement_time() + timestamp_offset_;

  for (size_t i = 0; i < readers_.size(); ++i) {
    readers_[i]->Observe();
    const auto &camera_msg = readers_[i]->GetLatestObserved();
    if (camera_msg == nullptr) {
      return false;
    }
    frame_ptr_->data_provider[i]->FillImageData(
        image_height_, image_width_,
        reinterpret_cast<const uint8_t *>(camera_msg->data().data()),
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

  std::shared_ptr<PerceptionObstacles> out_message(new (std::nothrow)
                                                       PerceptionObstacles);
  if (MakeProtobufMsg(msg_timestamp, seq_num_, frame_ptr_->detected_objects,
                      out_message.get()) != cyber::SUCC) {
    AERROR << "MakeProtobufMsg failed ts: " << msg_timestamp;
    return false;
  }
  seq_num_++;
  // Send msg
  writer_->Write(out_message);

  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
