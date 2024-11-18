/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/camera_detection_occupancy/camera_detection_occupancy_component.h"

#include <string>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/profiler/profiler.h"
#include "cyber/time/clock.h"
#include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"
#include "modules/perception/common/base/camera.h"
#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {
namespace camera {

bool CameraDetectionOccComponent::InitDetector(
    const CameraDetectionBEV &detection_param) {
  // Init conf file
  auto detector_plugin_param = detection_param.detector_plugin_param();
  init_options.config_path = detector_plugin_param.config_path();
  init_options.config_file = detector_plugin_param.config_file();
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
  detector_.reset(BaseObstacleDetectorRegisterer::GetInstanceByName(
      detector_plugin_param.name()));
  detector_->Init(init_options);
  return true;
}

bool CameraDetectionOccComponent::InitCameraFrame(
    const CameraDetectionBEV &detection_param) {
  DataProvider::InitOptions init_options;
  init_options.image_height = image_height_;
  init_options.image_width = image_width_;
  init_options.do_undistortion = detection_param.enable_undistortion();
  init_options.device_id = detection_param.gpu_id();
  AINFO << "init_options.device_id: " << init_options.device_id
        << " camera_name: " << init_options.sensor_name;

  std::vector<std::string> bev_sensor_name{
      detection_param.sensors().bev_camera_name().begin(),
      detection_param.sensors().bev_camera_name().end()};

  frame_ptr_ = std::make_shared<CameraFrame>();
  frame_ptr_->data_provider.resize(bev_sensor_name.size());
  frame_ptr_->k_lidar2img.clear();

  for (int i = 0; i < bev_sensor_name.size(); ++i) {
    init_options.sensor_name = bev_sensor_name[i];
    // get extrinsic
    // std::string camera_extrinsic_file_path =
    //     algorithm::SensorManager::Instance()->ExtrinsicPath(
    //         init_options.sensor_name);
    std::string camera_extrinsic_file_path = FLAGS_obs_sensor_intrinsic_path +
                                             "/" + init_options.sensor_name +
                                             "_extrinsics.yaml";
    Eigen::Affine3d camera2lidar_rt = Eigen::Affine3d::Identity();

    if (detection_param.dataset_type() == "nuscenes") {
      std::string lidar_extrinsic_file_path =
        FLAGS_obs_sensor_intrinsic_path + "/LIDAR_TOP_novatel_extrinsics.yaml";
      LoadCameraExtrinsicNus(
        camera_extrinsic_file_path,
        lidar_extrinsic_file_path,
        &camera2lidar_rt);
    } else {
      LoadCameraExtrinsic(camera_extrinsic_file_path, &camera2lidar_rt);
    }
    Eigen::Matrix4d lidar2camera_rt =
        camera2lidar_rt.inverse().matrix();

    // get intrinsic
    algorithm::SensorManager *sensor_manager =
        algorithm::SensorManager::Instance();
    if (!sensor_manager->IsSensorExist(init_options.sensor_name)) {
      AERROR << "Sensor '" << init_options.sensor_name << "' not exists!";
      return false;
    }

    Eigen::Matrix3f camera_intrinsic =
        std::dynamic_pointer_cast<base::BrownCameraDistortionModel>(
            sensor_manager->GetDistortCameraModel(init_options.sensor_name))
            ->get_intrinsic_params();
    Eigen::Matrix4f viewpad = Eigen::Matrix4f::Identity();
    viewpad(0, 0) = camera_intrinsic(0, 0);
    viewpad(0, 2) = camera_intrinsic(0, 2);
    viewpad(1, 1) = camera_intrinsic(1, 1);
    viewpad(1, 2) = camera_intrinsic(1, 2);

    // lidar2img_rt = (viewpad @ lidar2cam_rt.T)
    Eigen::Matrix4d lidar2img_rt =
        viewpad.cast<double>() * lidar2camera_rt;

    for (size_t i = 0; i < lidar2img_rt.rows(); i++) {
      for (size_t j = 0; j < lidar2img_rt.cols(); j++) {
        frame_ptr_->k_lidar2img.push_back(lidar2img_rt(i, j));
      }
    }

    frame_ptr_->data_provider[i] = std::make_shared<DataProvider>();
    frame_ptr_->data_provider[i]->Init(init_options);
  }
  return true;
}

bool CameraDetectionOccComponent::LoadCameraExtrinsic(
    const std::string &file_path, Eigen::Affine3d *camera2lidar_rt) {
  CHECK_NOTNULL(camera2lidar_rt);

  YAML::Node config = YAML::LoadFile(file_path);
  if (config["transform"]) {
    if (config["transform"]["translation"]) {
      camera2lidar_rt->translation()(0) =
          config["transform"]["translation"]["x"].as<double>();
      camera2lidar_rt->translation()(1) =
          config["transform"]["translation"]["y"].as<double>();
      camera2lidar_rt->translation()(2) =
          config["transform"]["translation"]["z"].as<double>();
      if (config["transform"]["rotation"]) {
        double qx = config["transform"]["rotation"]["x"].as<double>();
        double qy = config["transform"]["rotation"]["y"].as<double>();
        double qz = config["transform"]["rotation"]["z"].as<double>();
        double qw = config["transform"]["rotation"]["w"].as<double>();
        camera2lidar_rt->linear() =
            Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
        return true;
      }
    }
  }
  return false;
}

bool CameraDetectionOccComponent::LoadCameraExtrinsicNus(
    const std::string &camera_extrinsic_file_path,
    const std::string &lidar_extrinsic_file_path,
    Eigen::Affine3d *camera2lidar_rt) {
  CHECK_NOTNULL(camera2lidar_rt);
  Eigen::Affine3d local2lidar_rt = Eigen::Affine3d::Identity();
  YAML::Node config = YAML::LoadFile(lidar_extrinsic_file_path);
  if (config["transform"]) {
    if (config["transform"]["translation"]) {
      local2lidar_rt.translation()(0) =
          config["transform"]["translation"]["x"].as<double>();
      local2lidar_rt.translation()(1) =
          config["transform"]["translation"]["y"].as<double>();
      local2lidar_rt.translation()(2) =
          config["transform"]["translation"]["z"].as<double>();
      if (config["transform"]["rotation"]) {
        double qx = config["transform"]["rotation"]["x"].as<double>();
        double qy = config["transform"]["rotation"]["y"].as<double>();
        double qz = config["transform"]["rotation"]["z"].as<double>();
        double qw = config["transform"]["rotation"]["w"].as<double>();
        local2lidar_rt.linear() =
            Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
      }
    }
  }
  // AERROR << config["transform"]["translation"]["x"].as<double>() << " "
  //        << config["transform"]["translation"]["y"].as<double>() << " "
  //        << config["transform"]["translation"]["z"].as<double>() << " "
  //        << config["transform"]["rotation"]["x"].as<double>() << " "
  //        << config["transform"]["rotation"]["y"].as<double>() << " "
  //        << config["transform"]["rotation"]["z"].as<double>() << " "
  //        << config["transform"]["rotation"]["w"].as<double>();

  Eigen::Affine3d local2cam_rt = Eigen::Affine3d::Identity();
  config = YAML::LoadFile(camera_extrinsic_file_path);
  if (config["transform"]) {
    if (config["transform"]["translation"]) {
      local2cam_rt.translation()(0) =
          config["transform"]["translation"]["x"].as<double>();
      local2cam_rt.translation()(1) =
          config["transform"]["translation"]["y"].as<double>();
      local2cam_rt.translation()(2) =
          config["transform"]["translation"]["z"].as<double>();
      if (config["transform"]["rotation"]) {
        double qx = config["transform"]["rotation"]["x"].as<double>();
        double qy = config["transform"]["rotation"]["y"].as<double>();
        double qz = config["transform"]["rotation"]["z"].as<double>();
        double qw = config["transform"]["rotation"]["w"].as<double>();
        local2cam_rt.linear() =
            Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
      }
    }
  }

  // AERROR << config["transform"]["translation"]["x"].as<double>() << " "
  //        << config["transform"]["translation"]["y"].as<double>() << " "
  //        << config["transform"]["translation"]["z"].as<double>() << " "
  //        << config["transform"]["rotation"]["x"].as<double>() << " "
  //        << config["transform"]["rotation"]["y"].as<double>() << " "
  //        << config["transform"]["rotation"]["z"].as<double>() << " "
  //        << config["transform"]["rotation"]["w"].as<double>();

  *camera2lidar_rt = (local2cam_rt.inverse() * local2lidar_rt).inverse();
  return false;
}

bool CameraDetectionOccComponent::InitListeners(
    const CameraDetectionBEV &detection_param) {
  for (const auto &channel :
       detection_param.channel().input_camera_channel_name()) {
    std::shared_ptr<cyber::Reader<drivers::Image>> reader;
    if (channel ==
        detection_param.channel().input_camera_channel_name()
            [detection_param.channel().input_camera_channel_name().size() -
             1]) {
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

bool CameraDetectionOccComponent::InitTransformWrapper(
    const CameraDetectionBEV &detection_param) {
  trans_wrapper_.reset(new onboard::TransformWrapper());
  // tf_camera_frame_id
  trans_wrapper_->Init(detection_param.frame_id());
  return true;
}

bool CameraDetectionOccComponent::Init() {
  CameraDetectionBEV detection_param;
  if (!GetProtoConfig(&detection_param)) {
    AERROR << "Load camera detection 3d component config failed!";
    return false;
  }

  // Init tracker
  auto tracker_plugin_param = detection_param.tracker_plugin_param();
  BaseTracker *tracker =
      BaseTrackerRegisterer::GetInstanceByName(tracker_plugin_param.name());
  CHECK_NOTNULL(tracker);
  tracker_.reset(tracker);

  TrackerInitOptions tracker_init_options;
  tracker_init_options.config_path = tracker_plugin_param.config_path();
  tracker_init_options.config_file = tracker_plugin_param.config_file();
  ACHECK(tracker_->Init(tracker_init_options)) << "camera tracker init error";

  // segmentor init
  // auto segmentor_plugin_param = detection_param.segmentor_plugin_param();
  // BaseSegmentor *segmentor =
  //     BaseSegmentorRegisterer::GetInstanceByName(segmentor_plugin_param.name());
  // CHECK_NOTNULL(segmentor);
  // segmentor_.reset(segmentor);

  // SegmentorInitOptions segmentr_init_options;
  // segmentr_init_options.config_path = segmentor_plugin_param.config_path();
  // segmentr_init_options.config_file = segmentor_plugin_param.config_file();
  // ACHECK(segmentor_->Init(segmentr_init_options))
  //     << "lidar segmentor init error";

  // todo(zero): need init image_height_\image_width_
  InitDetector(detection_param);
  InitCameraFrame(detection_param);
  InitListeners(detection_param);
  InitTransformWrapper(detection_param);

  writer_ = node_->CreateWriter<PerceptionObstacles>(
      detection_param.channel().output_obstacles_channel_name());
  return true;
}

void CameraDetectionOccComponent::CameraToWorldCoor(
    const Eigen::Affine3d &camera2world, std::vector<base::ObjectPtr> *objs) {
  for (auto &obj : *objs) {
    Eigen::Vector3d local_center =
        obj->camera_supplement.local_center.cast<double>();
    obj->center = camera2world * local_center;

    obj->direction = camera2world.linear().cast<float>() * obj->direction;
    obj->theta = std::atan2(obj->direction(1), obj->direction(0));

    for (size_t i = 0; i < obj->polygon.size(); ++i) {
      auto &pt = obj->polygon.at(i);
      Eigen::Vector3d local_polygon_point =
          Eigen::Vector3f(pt.x, pt.y, pt.z).cast<double>();
      Eigen::Vector3d world_polygon_point = camera2world * local_polygon_point;
      pt.x = world_polygon_point[0];
      pt.y = world_polygon_point[1];
      pt.z = world_polygon_point[2];
    }
  }
}

int CameraDetectionOccComponent::ConvertObjectToPb(
    const base::ObjectPtr &object_ptr, PerceptionObstacle *pb_msg) {
  if (!object_ptr || !pb_msg) {
    return cyber::FAIL;
  }
  pb_msg->set_id(object_ptr->track_id);
  pb_msg->set_theta(object_ptr->theta);
  pb_msg->set_confidence(object_ptr->confidence);

  apollo::common::Point3D *obj_center = pb_msg->mutable_position();
  obj_center->set_x(object_ptr->center(0));
  obj_center->set_y(object_ptr->center(1));
  obj_center->set_z(object_ptr->center(2));

  apollo::common::Point3D *obj_velocity = pb_msg->mutable_velocity();
  obj_velocity->set_x(object_ptr->velocity(0));
  obj_velocity->set_y(object_ptr->velocity(1));
  obj_velocity->set_z(object_ptr->velocity(2));

  apollo::common::Point3D *obj_acceleration = pb_msg->mutable_acceleration();
  obj_acceleration->set_x(object_ptr->acceleration(0));
  obj_acceleration->set_y(object_ptr->acceleration(1));
  obj_acceleration->set_z(object_ptr->acceleration(2));

  pb_msg->set_length(object_ptr->size(0));
  pb_msg->set_width(object_ptr->size(1));
  pb_msg->set_height(object_ptr->size(2));

  // for camera results, set object's center as anchor point
  pb_msg->set_type(static_cast<PerceptionObstacle::Type>(object_ptr->type));
  pb_msg->set_sub_type(
      static_cast<PerceptionObstacle::SubType>(object_ptr->sub_type));
  pb_msg->set_timestamp(object_ptr->latest_tracked_time);  // in seconds.

  pb_msg->set_height_above_ground(object_ptr->size(2));

  for (size_t i = 0; i < object_ptr->polygon.size(); ++i) {
    auto &pt = object_ptr->polygon.at(i);
    apollo::common::Point3D *p = pb_msg->add_polygon_point();
    p->set_x(pt.x);
    p->set_y(pt.y);
    p->set_z(pt.z);
  }
  return cyber::SUCC;
}

int CameraDetectionOccComponent::MakeProtobufMsg(
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

bool CameraDetectionOccComponent::OnReceiveImage(
    const std::shared_ptr<drivers::Image> &msg) {
  const double msg_timestamp = msg->measurement_time() + timestamp_offset_;

  for (size_t i = 0; i < readers_.size(); ++i) {
    readers_[i]->Observe();
    const auto &camera_msg = readers_[i]->GetLatestObserved();
    if (camera_msg == nullptr) {
      AERROR << "Failed to get camera message, camera_name: "
             << msg->frame_id();
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
  PERF_BLOCK("foreground objects detection")
  detector_->Detect(frame_ptr_.get());
  PERF_BLOCK_END

  // // segmention
  // PERF_BLOCK("occ cluster")
  // if (!segmentor_->Detect(frame_ptr_.get())) {
  //   AERROR << "occ cluster error.";
  //   return false;
  // }
  // PERF_BLOCK_END

  // // object builder
  // PERF_BLOCK("occ object builder")
  // ObjectBuilderOptions builder_options;
  // if (!builder_.Build(builder_options, frame_ptr_.get())) {
  //   AERROR << "Lidar segmentation, object builder error.";
  //   return false;
  // }
  // PERF_BLOCK_END

  // Get sensor to world pose from TF
  Eigen::Affine3d lidar2world_trans;
  if (!trans_wrapper_->GetSensor2worldTrans(msg_timestamp,
                                            &lidar2world_trans)) {
    const std::string err_str =
        absl::StrCat("failed to get camera to world pose, ts: ", msg_timestamp,
                     " camera_name: ", msg->frame_id());
    AERROR << err_str;
    return false;
  }

  CameraToWorldCoor(lidar2world_trans, &frame_ptr_->detected_objects);

  // Tracker
  // Tracker: converter
  // std::shared_ptr<base::Frame> base_frame_ptr_(new base::Frame);
  // if (!ConvertToBaseFrame(frame_ptr_.get(), base_frame_ptr_.get())) {
  //     AERROR << "Failed to convert frame";
  //     return false;
  // }
  // // Tracker: object track
  // base_frame_ptr_->sensor2world_pose = lidar2world_trans;
  // base::FramePtr tracker_frame_ptr(new base::Frame);
  // if (!tracker_->Track(*base_frame_ptr_, tracker_frame_ptr)) {
  //     AERROR << "track error";
  //     return false;
  // }

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

bool CameraDetectionOccComponent::ConvertToBaseFrame(const CameraFrame *frame,
                                                     base::Frame *base_frame) {
  base_frame->timestamp = frame->timestamp;
  for (size_t i = 0; i < frame->detected_objects.size(); ++i) {
    base::ObjectPtr object = frame->detected_objects[i];
    base_frame->objects.emplace_back(object);
  }
  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
