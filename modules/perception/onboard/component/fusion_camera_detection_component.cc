/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/onboard/component/fusion_camera_detection_component.h"

#include <boost/format.hpp>
#include <yaml-cpp/yaml.h>

#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time_util.h"
#include "modules/common/util/file.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/lib/utils/time_util.h"
#include "modules/perception/onboard/common_flags/common_flags.h"
#include "modules/perception/onboard/component/camera_perception_viz_message.h"

namespace apollo {
namespace perception {
namespace onboard {

using apollo::common::util::GetAbsolutePath;

static int GetGpuId(const camera::CameraPerceptionInitOptions &options) {
  camera::app::PerceptionParam perception_param;
  std::string work_root = "";
  camera::GetCyberWorkRoot(&work_root);
  std::string config_file =
      GetAbsolutePath(options.root_dir, options.conf_file);
  config_file = GetAbsolutePath(work_root, config_file);
  if (!apollo::common::util::GetProtoFromFile(config_file, &perception_param)) {
    AERROR << "Read config failed: " << config_file;
    return -1;
  }
  if (!perception_param.has_gpu_id()) {
    AINFO << "gpu id not found.";
    return -1;
  }
  return perception_param.gpu_id();
}

bool SetCameraHeight(const std::string &sensor_name,
                     const std::string &params_dir, float default_camera_height,
                     float *camera_height) {
  float base_h = default_camera_height;
  float camera_offset = 0.0f;
  try {
    YAML::Node lidar_height =
        YAML::LoadFile(params_dir + "/" + "velodyne128_height.yaml");
    base_h = lidar_height["vehicle"]["parameters"]["height"].as<float>();
    AINFO << base_h;
    YAML::Node camera_ex =
        YAML::LoadFile(params_dir + "/" + sensor_name + "_extrinsics.yaml");
    camera_offset = camera_ex["transform"]["translation"]["z"].as<float>();
    AINFO << camera_offset;
    *camera_height = base_h + camera_offset;
  } catch (YAML::InvalidNode &in) {
    AERROR << "load camera extrisic file error, YAML::InvalidNode exception";
    return false;
  } catch (YAML::TypedBadConversion<float> &bc) {
    AERROR << "load camera extrisic file error, "
           << "YAML::TypedBadConversion exception";
    return false;
  } catch (YAML::Exception &e) {
    AERROR << "load camera extrisic file "
           << " error, YAML exception:" << e.what();
    return false;
  }
  return true;
}

// @description: load camera extrinsics from yaml file
bool LoadExtrinsics(const std::string &yaml_file,
                    Eigen::Matrix4d *camera_extrinsic) {
  if (!apollo::common::util::PathExists(yaml_file)) {
    AINFO << yaml_file << " not exist!";
    return false;
  }
  YAML::Node node = YAML::LoadFile(yaml_file);
  double qw = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double tx = 0.0;
  double ty = 0.0;
  double tz = 0.0;
  try {
    if (node.IsNull()) {
      AINFO << "Load " << yaml_file << " failed! please check!";
      return false;
    }
    qw = node["transform"]["rotation"]["w"].as<double>();
    qx = node["transform"]["rotation"]["x"].as<double>();
    qy = node["transform"]["rotation"]["y"].as<double>();
    qz = node["transform"]["rotation"]["z"].as<double>();
    tx = node["transform"]["translation"]["x"].as<double>();
    ty = node["transform"]["translation"]["y"].as<double>();
    tz = node["transform"]["translation"]["z"].as<double>();
  } catch (YAML::InvalidNode &in) {
    AERROR << "load camera extrisic file " << yaml_file
           << " with error, YAML::InvalidNode exception";
    return false;
  } catch (YAML::TypedBadConversion<double> &bc) {
    AERROR << "load camera extrisic file " << yaml_file
           << " with error, YAML::TypedBadConversion exception";
    return false;
  } catch (YAML::Exception &e) {
    AERROR << "load camera extrisic file " << yaml_file
           << " with error, YAML exception:" << e.what();
    return false;
  }
  camera_extrinsic->setConstant(0);
  Eigen::Quaterniond q;
  q.x() = qx;
  q.y() = qy;
  q.z() = qz;
  q.w() = qw;
  (*camera_extrinsic).block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  (*camera_extrinsic)(0, 3) = tx;
  (*camera_extrinsic)(1, 3) = ty;
  (*camera_extrinsic)(2, 3) = tz;
  (*camera_extrinsic)(3, 3) = 1;
  return true;
}

// @description: get project matrix
bool GetProjectMatrix(
    const std::vector<std::string> &camera_names,
    const std::map<std::string, Eigen::Matrix4d> &extrinsic_map,
    const std::map<std::string, Eigen::Matrix3f> &intrinsic_map,
    Eigen::Matrix3d *project_matrix, double *pitch_diff = nullptr) {
  if (camera_names.size() != 2) {
    AINFO << "camera number must be 2!";
    return false;
  }
  *project_matrix =
      intrinsic_map.at(camera_names[0]).cast<double>() *
      extrinsic_map.at(camera_names[0]).block<3, 3>(0, 0).inverse() *
      extrinsic_map.at(camera_names[1]).block<3, 3>(0, 0) *
      intrinsic_map.at(camera_names[1]).cast<double>().inverse();
  // extract the pitch_diff = pitch_narrow - pitch_obstacle
  if (pitch_diff != nullptr) {
    Eigen::Vector3d euler =
        (extrinsic_map.at(camera_names[0]).block<3, 3>(0, 0).inverse() *
         extrinsic_map.at(camera_names[1]).block<3, 3>(0, 0))
            .eulerAngles(0, 1, 2);
    *pitch_diff = euler(0);
    AINFO << "pitch diff: " << *pitch_diff;
  }
  return true;
}

FusionCameraDetectionComponent::~FusionCameraDetectionComponent() {}

bool FusionCameraDetectionComponent::Init() {
  // TODO(someone): read output-channel name from config
  writer_ = node_->CreateWriter<PerceptionObstacles>("/perception/obstacles");
  sensorframe_writer_ = node_->CreateWriter<SensorFrameMessage>(
      "/perception/inner/PrefusedObjects");
  camera_viz_writer_ = node_->CreateWriter<CameraPerceptionVizMessage>(
      "/perception/inner/camera_viz_msg");
  if (InitConfig() != cyber::SUCC) {
    AERROR << "InitConfig() failed.";
    return false;
  }
  if (InitSensorInfo() != cyber::SUCC) {
    AERROR << "InitSensorInfo() failed.";
    return false;
  }
  if (InitAlgorithmPlugin() != cyber::SUCC) {
    AERROR << "InitAlgorithmPlugin() failed.";
    return false;
  }
  if (InitCameraFrames() != cyber::SUCC) {
    AERROR << "InitCameraFrames() failed.";
    return false;
  }
  if (InitProjectMatrix() != cyber::SUCC) {
    AERROR << "InitProjectMatrix() failed.";
    return false;
  }
  if (InitCameraListeners() != cyber::SUCC) {
    AERROR << "InitCameraListeners() failed.";
    return false;
  }
  SetCameraHeightAndPitch();

  return true;
}

void FusionCameraDetectionComponent::OnReceiveImage(
    const std::shared_ptr<apollo::drivers::Image> &message,
    const std::string &camera_name) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double msg_timestamp = message->measurement_time() + timestamp_offset_;
  AINFO << "Enter FusionCameraDetectionComponent::Proc(), "
        << " camera_name: " << camera_name
        << " image ts: " + std::to_string(msg_timestamp);
  // timestamp should be almost monotonic
  if (last_timestamp_ - msg_timestamp > ts_diff_) {
    AINFO << "Received an old message. Last ts is " << std::setprecision(19)
          << last_timestamp_ << " current ts is " << msg_timestamp
          << " last - current is " << last_timestamp_ - msg_timestamp;
    return;
  }
  last_timestamp_ = msg_timestamp;
  ++seq_num_;

  // for e2e lantency statistics
  {
    const double cur_time = lib::TimeUtil::GetCurrentTime();
    const double start_latency = (cur_time - message->measurement_time()) * 1e3;
    AINFO << "FRAME_STATISTICS:Camera:Start:msg_time[" << camera_name << "-"
          << GLOG_TIMESTAMP(message->measurement_time()) << "]:cur_time["
          << GLOG_TIMESTAMP(cur_time) << "]:cur_latency[" << start_latency
          << "]";
  }

  // protobuf msg
  std::shared_ptr<apollo::perception::PerceptionObstacles> out_message(
      new (std::nothrow) apollo::perception::PerceptionObstacles);
  apollo::common::ErrorCode error_code = apollo::common::OK;

  // prefused msg
  std::shared_ptr<SensorFrameMessage> prefused_message(new (std::nothrow)
                                                           SensorFrameMessage);

  if (InternalProc(message, camera_name, &error_code, prefused_message.get(),
                   out_message.get()) != cyber::SUCC) {
    AERROR << "InternalProc failed, error_code: " << error_code;
    if (MakeProtobufMsg(msg_timestamp, seq_num_, std::vector<base::ObjectPtr>(),
                        error_code, out_message.get()) != cyber::SUCC) {
      AERROR << "MakeProtobufMsg failed";
      return;
    }
    if (output_final_obstacles_) {
      writer_->Write(out_message);
    }
    return;
  }

  bool send_sensorframe_ret =
      sensorframe_writer_->Write(prefused_message);
  AINFO << "send out prefused msg, ts: " << std::to_string(msg_timestamp)
      << "ret: " << send_sensorframe_ret;
  // Send output msg
  if (output_final_obstacles_) {
    writer_->Write(out_message);
  }
  // for e2e lantency statistics
  {
    const double end_timestamp =
        apollo::perception::lib::TimeUtil::GetCurrentTime();
    const double end_latency =
        (end_timestamp - message->measurement_time()) * 1e3;
    AINFO << "FRAME_STATISTICS:Camera:End:msg_time[" << camera_name << "-"
          << GLOG_TIMESTAMP(message->measurement_time()) << "]:cur_time["
          << GLOG_TIMESTAMP(end_timestamp) << "]:cur_latency[" << end_latency
          << "]";
  }
}

int FusionCameraDetectionComponent::InitConfig() {
  // the macro READ_CONF would return cyber::FAIL if config not exists
  apollo::perception::onboard::FusionCameraDetection
      fusion_camera_detection_param;
  if (!GetProtoConfig(&fusion_camera_detection_param)) {
    AINFO << "load fusion camera detection component proto param failed";
    return false;
  }

  std::string camera_names_str = fusion_camera_detection_param.camera_names();
  boost::algorithm::split(camera_names_, camera_names_str,
                          boost::algorithm::is_any_of(","));
  if (camera_names_.size() != 2) {
    AERROR << "Now FusionCameraDetectionComponent only support 2 cameras";
    return cyber::FAIL;
  }

  std::string input_camera_channel_names_str =
      fusion_camera_detection_param.input_camera_channel_names();
  boost::algorithm::split(input_camera_channel_names_,
                          input_camera_channel_names_str,
                          boost::algorithm::is_any_of(","));
  if (input_camera_channel_names_.size() != camera_names_.size()) {
    AERROR << "wrong input_camera_channel_names_.size(): "
           << input_camera_channel_names_.size();
    return cyber::FAIL;
  }

  camera_perception_init_options_.root_dir =
      fusion_camera_detection_param.camera_obstacle_perception_conf_dir();
  camera_perception_init_options_.conf_file =
      fusion_camera_detection_param.camera_obstacle_perception_conf_file();
  camera_perception_init_options_.lane_calibration_working_sensor_name =
      fusion_camera_detection_param.lane_calibration_working_sensor_name();
  camera_perception_init_options_.use_cyber_work_root = true;
  frame_capacity_ = fusion_camera_detection_param.frame_capacity();
  image_channel_num_ = fusion_camera_detection_param.image_channel_num();
  enable_undistortion_ = fusion_camera_detection_param.enable_undistortion();
  enable_visualization_ = fusion_camera_detection_param.enable_visualization();
  output_obstacles_channel_name_ =
      fusion_camera_detection_param.output_obstacles_channel_name();
  camera_perception_viz_message_channel_name_ =
      fusion_camera_detection_param
          .camera_perception_viz_message_channel_name();
  output_final_obstacles_ =
      fusion_camera_detection_param.output_final_obstacles();
  prefused_channel_name_ =
      fusion_camera_detection_param.prefused_channel_name();
  default_camera_pitch_ =
    static_cast<float>(fusion_camera_detection_param.default_camera_pitch());
  default_camera_height_ =
    static_cast<float>(fusion_camera_detection_param.default_camera_height());
  output_camera_debug_msg_ =
      fusion_camera_detection_param.output_camera_debug_msg();
  camera_debug_channel_name_ =
      fusion_camera_detection_param.camera_debug_channel_name();
  ts_diff_ = fusion_camera_detection_param.ts_diff();

  std::string format_str = R"(
      FusionCameraDetectionComponent InitConfig success
      camera_names:    %s, %s
      camera_obstacle_perception_conf_dir:    %s
      camera_obstacle_perception_conf_file:    %s
      frame_capacity:    %d
      image_channel_num:    %d
      enable_undistortion:    %d
      enable_visualization:    %d
      output_obstacles_channel_name:    %s
      camera_perception_viz_message_channel_name:    %s
      output_final_obstacles:    %s
      prefused_channel_name:    %s)";
  std::string config_info_str =
      str(boost::format(format_str.c_str()) % camera_names_[0] %
          camera_names_[1] % camera_perception_init_options_.root_dir %
          camera_perception_init_options_.conf_file % frame_capacity_ %
          image_channel_num_ % enable_undistortion_ % enable_visualization_ %
          output_obstacles_channel_name_ %
          camera_perception_viz_message_channel_name_ %
          output_final_obstacles_ % prefused_channel_name_);
  AINFO << config_info_str;

  return cyber::SUCC;
}

int FusionCameraDetectionComponent::InitSensorInfo() {
  if (camera_names_.size() != 2) {
    AERROR << "invalid camera_names_.size(): " << camera_names_.size();
    return cyber::FAIL;
  }

  auto* sensor_manager = common::SensorManager::Instance();
  for (size_t i = 0; i < camera_names_.size(); ++i) {
    if (!sensor_manager->IsSensorExist(camera_names_[i])) {
      AERROR << ("sensor_name: " + camera_names_[i] + " not exists.");
      return cyber::FAIL;
    }

    base::SensorInfo sensor_info;
    if (!(sensor_manager->GetSensorInfo(camera_names_[i], &sensor_info))) {
      AERROR << "Failed to get sensor info, sensor name: " << camera_names_[i];
      return cyber::FAIL;
    }
    sensor_info_map_[camera_names_[i]] = sensor_info;

    std::string tf_camera_frame_id =
        sensor_manager->GetFrameId(camera_names_[i]);
    tf_camera_frame_id_map_[camera_names_[i]] = tf_camera_frame_id;
    std::shared_ptr<TransformWrapper> trans_wrapper(new TransformWrapper);
    trans_wrapper->Init(tf_camera_frame_id);
    camera2world_trans_wrapper_map_[camera_names_[i]] = trans_wrapper;
  }

  // assume all camera have same image size
  base::BaseCameraModelPtr camera_model_ptr =
      sensor_manager->GetUndistortCameraModel(camera_names_[0]);
  image_width_ = static_cast<int>(camera_model_ptr->get_width());
  image_height_ = static_cast<int>(camera_model_ptr->get_height());

  std::string format_str = R"(
      camera_names: %s %s
      tf_camera_frame_ids: %s %s
      image_width: %d
      image_height: %d
      image_channel_num: %d)";
  std::string sensor_info_str =
      str(boost::format(format_str.c_str()) % camera_names_[0] %
          camera_names_[0] % tf_camera_frame_id_map_[camera_names_[0]] %
          tf_camera_frame_id_map_[camera_names_[1]] % image_width_ %
          image_height_ % image_channel_num_);
  AINFO << sensor_info_str;

  return cyber::SUCC;
}

int FusionCameraDetectionComponent::InitAlgorithmPlugin() {
  camera_obstacle_pipeline_.reset(new camera::ObstacleCameraPerception);
  if (!camera_obstacle_pipeline_->Init(camera_perception_init_options_)) {
    AERROR << "camera_obstacle_pipeline_->Init() failed";
    return cyber::FAIL;
  }
  return cyber::SUCC;
}

int FusionCameraDetectionComponent::InitCameraFrames() {
  if (camera_names_.size() != 2) {
    AERROR << "invalid camera_names_.size(): " << camera_names_.size();
    return cyber::FAIL;
  }
  // fixed size
  camera_frames_.resize(frame_capacity_);
  if (camera_frames_.size() == 0) {
    AERROR << "frame_capacity_ must > 0";
    return cyber::FAIL;
  }

  // init data_providers for each camera
  for (const auto &camera_name : camera_names_) {
    camera::DataProvider::InitOptions data_provider_init_options;
    data_provider_init_options.image_height = image_height_;
    data_provider_init_options.image_width = image_width_;
    data_provider_init_options.do_undistortion = enable_undistortion_;
    data_provider_init_options.sensor_name = camera_name;
    int gpu_id = GetGpuId(camera_perception_init_options_);
    if (gpu_id == -1) {
      return cyber::FAIL;
    }
    data_provider_init_options.device_id = gpu_id;
    AINFO << "data_provider_init_options.device_id: "
          << data_provider_init_options.device_id
          << " camera_name: " << camera_name;

    std::shared_ptr<camera::DataProvider> data_provider(
        new camera::DataProvider);
    data_provider->Init(data_provider_init_options);
    data_providers_map_[camera_name] = data_provider;
  }

  //  init extrinsic/intrinsic
  for (const auto &camera_name : camera_names_) {
    base::BaseCameraModelPtr model;
    model = common::SensorManager::Instance()->GetUndistortCameraModel(
        camera_name);
    auto pinhole = static_cast<base::PinholeCameraModel *>(model.get());
    Eigen::Matrix3f intrinsic = pinhole->get_intrinsic_params();
    intrinsic_map_[camera_name] = intrinsic;
    AINFO << "#intrinsics of " << camera_name << ": "
          << intrinsic_map_[camera_name];
    Eigen::Matrix4d extrinsic;
    LoadExtrinsics(FLAGS_obs_sensor_intrinsic_path + "/" + camera_name +
                       "_extrinsics.yaml",
                   &extrinsic);
    extrinsic_map_[camera_name] = extrinsic;
  }

  // Init camera height
  for (const auto &camera_name : camera_names_) {
    float height = 0.0f;
    SetCameraHeight(camera_name, FLAGS_obs_sensor_intrinsic_path,
                    default_camera_height_, &height);
    camera_height_map_[camera_name] = height;
  }

  for (auto &frame : camera_frames_) {
    frame.track_feature_blob.reset(new base::Blob<float>());
    frame.lane_detected_blob.reset(new base::Blob<float>());
  }
  return cyber::SUCC;
}

int FusionCameraDetectionComponent::InitProjectMatrix() {
  if (!GetProjectMatrix(camera_names_, extrinsic_map_, intrinsic_map_,
                        &project_matrix_, &pitch_diff_)) {
    AERROR << "GetProjectMatrix failed";
    return cyber::FAIL;
  }
  AINFO << "project_matrix_: " << project_matrix_;
  AINFO << "pitch_diff_:" << pitch_diff_;
  name_camera_pitch_angle_diff_map_[camera_names_[0]] = 0.f;
  name_camera_pitch_angle_diff_map_[camera_names_[1]] =
      static_cast<float>(pitch_diff_);

  return cyber::SUCC;
}

int FusionCameraDetectionComponent::InitCameraListeners() {
  for (size_t i = 0; i < camera_names_.size(); ++i) {
    const std::string &camera_name = camera_names_[i];
    const std::string &channel_name = input_camera_channel_names_[i];
    const std::string &listener_name = camera_name + "_fusion_camera_listener";
    AINFO << "listener name: " << listener_name;

    typedef std::shared_ptr<apollo::drivers::Image> ImageMsgType;
    std::function<void(const ImageMsgType &)> camera_callback =
        std::bind(&FusionCameraDetectionComponent::OnReceiveImage, this,
                  std::placeholders::_1, camera_name);
    auto camera_reader = node_->CreateReader(channel_name, camera_callback);
  }
  return cyber::SUCC;
}

void FusionCameraDetectionComponent::SetCameraHeightAndPitch() {
  camera_obstacle_pipeline_->SetCameraHeightAndPitch(
      camera_height_map_, name_camera_pitch_angle_diff_map_,
      default_camera_pitch_);
}

int FusionCameraDetectionComponent::InternalProc(
    const std::shared_ptr<apollo::drivers::Image const> &in_message,
    const std::string &camera_name, apollo::common::ErrorCode *error_code,
    SensorFrameMessage *prefused_message,
    apollo::perception::PerceptionObstacles *out_message) {
  const double msg_timestamp =
      in_message->measurement_time() + timestamp_offset_;
  const int frame_size = static_cast<int>(camera_frames_.size());
  camera::CameraFrame &camera_frame = camera_frames_[frame_id_ % frame_size];

  prefused_message->timestamp_ = msg_timestamp;
  prefused_message->seq_num_ = seq_num_;
  prefused_message->process_stage_ = ProcessStage::MONOCULAR_CAMERA_DETECTION;
  prefused_message->sensor_id_ = camera_name;
  prefused_message->frame_ = base::FramePool::Instance().Get();
  prefused_message->frame_->sensor_info = sensor_info_map_[camera_name];
  prefused_message->frame_->timestamp = msg_timestamp;

  // Get sensor to world pose from TF
  Eigen::Affine3d camera2world_trans;
  if (!camera2world_trans_wrapper_map_[camera_name]->GetSensor2worldTrans(
          msg_timestamp, &camera2world_trans)) {
    std::string err_str = "failed to get camera to world pose, ts: " +
                          std::to_string(msg_timestamp) +
                          " camera_name: " + camera_name;
    AERROR << err_str;
    *error_code = apollo::common::ErrorCode::PERCEPTION_ERROR_TF;
    prefused_message->error_code_ = *error_code;
    return cyber::FAIL;
  }
  prefused_message->frame_->sensor2world_pose = camera2world_trans;

  // Fill camera frame
  // frame_size != 0, see InitCameraFrames()
  camera_frame.camera2world_pose = camera2world_trans;
  camera_frame.data_provider = data_providers_map_[camera_name].get();
  camera_frame.data_provider->FillImageData(
      image_height_, image_width_,
      reinterpret_cast<const uint8_t *>(in_message->data().data()),
      in_message->encoding());

  camera_frame.frame_id = frame_id_;
  camera_frame.timestamp = msg_timestamp;
  // get narrow to short projection matrix
  if (camera_frame.data_provider->sensor_name() == camera_names_[1]) {
    camera_frame.project_matrix = project_matrix_;
  } else {
    camera_frame.project_matrix.setIdentity();
  }

  ++frame_id_;
  // Run camera perception pipeline
  camera_obstacle_pipeline_->GetCalibrationService(
      &camera_frame.calibration_service);

  if (!camera_obstacle_pipeline_->Perception(camera_perception_options_,
                                             &camera_frame)) {
    AERROR << "camera_obstacle_pipeline_->Perception() failed"
           << " msg_timestamp: " << std::to_string(msg_timestamp);
    *error_code = apollo::common::ErrorCode::PERCEPTION_ERROR_PROCESS;
    prefused_message->error_code_ = *error_code;
    return cyber::FAIL;
  }
  AINFO << "##" << camera_name << ": pitch "
        << camera_frame.calibration_service->QueryPitchAngle()
        << " | camera_grond_height "
        << camera_frame.calibration_service->QueryCameraToGroundHeight();
  prefused_message->frame_->objects = camera_frame.tracked_objects;
  // TODO(gaohan02, wanji): check the boxes with 0-width in perception-camera
  prefused_message->frame_->objects.clear();
  for (auto obj : camera_frame.tracked_objects) {
    auto &box = obj->camera_supplement.box;
    if (box.xmin < box.xmax && box.ymin < box.ymax) {
      prefused_message->frame_->objects.push_back(obj);
    }
  }

  // process success, make pb msg
  if (output_final_obstacles_ &&
      MakeProtobufMsg(msg_timestamp, seq_num_, camera_frame.tracked_objects,
                      *error_code, out_message) != cyber::SUCC) {
    AERROR << "MakeProtobufMsg failed"
           << " ts: " << std::to_string(msg_timestamp);
    *error_code = apollo::common::ErrorCode::PERCEPTION_ERROR_UNKNOWN;
    prefused_message->error_code_ = *error_code;
    return cyber::FAIL;
  }
  *error_code = apollo::common::ErrorCode::OK;
  prefused_message->error_code_ = *error_code;

  prefused_message->frame_->camera_frame_supplement.on_use = true;
  if (FLAGS_obs_enable_visualization) {
    camera::DataProvider::ImageOptions image_options;
    image_options.target_color = base::Color::RGB;

    // Use Blob to pass image data
    prefused_message->frame_->camera_frame_supplement.image_blob.reset(
        new base::Blob<uint8_t>);
    camera_frame.data_provider->GetImageBlob(
        image_options,
        prefused_message->frame_->camera_frame_supplement.image_blob.get());
  }

  // Send msg for visualization
  if (enable_visualization_) {
    camera::DataProvider::ImageOptions image_options;
    image_options.target_color = base::Color::RGB;
    std::shared_ptr<base::Blob<uint8_t>> image_blob(new base::Blob<uint8_t>);
    camera_frame.data_provider->GetImageBlob(image_options, image_blob.get());
    std::shared_ptr<CameraPerceptionVizMessage> viz_msg(
        new (std::nothrow) CameraPerceptionVizMessage(
            camera_name, msg_timestamp, camera2world_trans.matrix(), image_blob,
            camera_frame.tracked_objects, camera_frame.lane_objects,
            *error_code));
    bool send_viz_ret = camera_viz_writer_->Write(viz_msg);
    AINFO << "send out camera visualization msg, ts: "
        << std::to_string(msg_timestamp) << " send_viz_ret: " << send_viz_ret;
  }

  return cyber::SUCC;
}

int FusionCameraDetectionComponent::MakeProtobufMsg(
    double msg_timestamp, int seq_num,
    const std::vector<base::ObjectPtr> &objects,
    const apollo::common::ErrorCode error_code,
    apollo::perception::PerceptionObstacles *obstacles) {
  double publish_time = apollo::cyber::Time::Now().ToSecond();
  apollo::common::Header *header = obstacles->mutable_header();
  header->set_timestamp_sec(publish_time);
  header->set_module_name("perception_camera");
  header->set_sequence_num(seq_num);
  // in nanosecond
  // PnC would use lidar timestamp to predict
  header->set_lidar_timestamp(static_cast<uint64_t>(msg_timestamp * 1e9));
  header->set_camera_timestamp(static_cast<uint64_t>(msg_timestamp * 1e9));
  // header->set_radar_timestamp(0);

  obstacles->set_error_code(error_code);
  for (const auto &obj : objects) {
    apollo::perception::PerceptionObstacle *obstacle =
        obstacles->add_perception_obstacle();
    if (ConvertObjectToPb(obj, obstacle) != cyber::SUCC) {
      AERROR << "ConvertObjectToPb failed, Object:" << obj->ToString();
      return cyber::FAIL;
    }
  }
  return cyber::SUCC;
}

int FusionCameraDetectionComponent::ConvertObjectToPb(
    const base::ObjectPtr &object_ptr,
    apollo::perception::PerceptionObstacle *pb_msg) {
  if (!object_ptr || !pb_msg) {
    return cyber::FAIL;
  }

  pb_msg->set_id(object_ptr->track_id);
  pb_msg->set_theta(object_ptr->theta);

  apollo::common::Point3D *obj_center = pb_msg->mutable_position();
  obj_center->set_x(object_ptr->center(0));
  obj_center->set_y(object_ptr->center(1));
  obj_center->set_z(object_ptr->center(2));

  apollo::common::Point3D *obj_velocity = pb_msg->mutable_velocity();
  obj_velocity->set_x(object_ptr->velocity(0));
  obj_velocity->set_y(object_ptr->velocity(1));
  obj_velocity->set_z(object_ptr->velocity(2));

  pb_msg->set_length(object_ptr->size(0));
  pb_msg->set_width(object_ptr->size(1));
  pb_msg->set_height(object_ptr->size(2));

  // convert 3d bbox to polygon
  int polygon_point_size = static_cast<int>(object_ptr->polygon.size());
  for (int i = 0; i < polygon_point_size; ++i) {
    auto &pt = object_ptr->polygon.at(i);
    apollo::common::Point3D *p = pb_msg->add_polygon_point();
    p->set_x(pt.x);
    p->set_y(pt.y);
    p->set_z(pt.z);
  }

  // for camera results, set object's center as anchor point
  apollo::common::Point3D *obj_anchor_point = pb_msg->mutable_anchor_point();
  obj_anchor_point->set_x(object_ptr->center(0));
  obj_anchor_point->set_y(object_ptr->center(1));
  obj_anchor_point->set_z(object_ptr->center(2));

  apollo::perception::BBox2D *obj_bbox2d = pb_msg->mutable_bbox2d();
  const base::BBox2DF &box = object_ptr->camera_supplement.box;
  obj_bbox2d->set_xmin(box.xmin);
  obj_bbox2d->set_ymin(box.ymin);
  obj_bbox2d->set_xmax(box.xmax);
  obj_bbox2d->set_ymax(box.ymax);

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      pb_msg->add_position_covariance(object_ptr->center_uncertainty(i, j));
      pb_msg->add_velocity_covariance(object_ptr->velocity_uncertainty(i, j));
    }
  }

  pb_msg->set_tracking_time(object_ptr->tracking_time);
  pb_msg->set_type(static_cast<PerceptionObstacle::Type>(object_ptr->type));
  pb_msg->set_sub_type(
      static_cast<PerceptionObstacle::SubType>(object_ptr->sub_type));
  pb_msg->set_timestamp(object_ptr->latest_tracked_time);  // in seconds.

  pb_msg->set_height_above_ground(object_ptr->size(2));

  if (object_ptr->type == base::ObjectType::VEHICLE) {
    apollo::perception::LightStatus *light_status =
        pb_msg->mutable_light_status();
    const base::CarLight &car_light = object_ptr->car_light;
    light_status->set_brake_visible(car_light.brake_visible);
    light_status->set_brake_switch_on(car_light.brake_switch_on);

    light_status->set_left_turn_visible(car_light.left_turn_visible);
    light_status->set_left_turn_switch_on(car_light.left_turn_switch_on);

    light_status->set_right_turn_visible(car_light.right_turn_visible);
    light_status->set_right_turn_switch_on(car_light.right_turn_switch_on);
  }

  return cyber::SUCC;
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
