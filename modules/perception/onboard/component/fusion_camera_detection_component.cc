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

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include "absl/strings/str_cat.h"
#include "yaml-cpp/yaml.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time.h"
#include "modules/common/time/time_util.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/onboard/common_flags/common_flags.h"
#include "modules/perception/onboard/component/camera_perception_viz_message.h"

namespace apollo {
namespace perception {
namespace onboard {

using apollo::cyber::common::GetAbsolutePath;

static void fill_lane_msg(const base::LaneLineCubicCurve &curve_coord,
                          apollo::perception::LaneMarker *lane_marker) {
  lane_marker->set_c0_position(curve_coord.d);
  lane_marker->set_c1_heading_angle(curve_coord.c);
  lane_marker->set_c2_curvature(curve_coord.b);
  lane_marker->set_c3_curvature_derivative(curve_coord.a);
  lane_marker->set_longitude_start(curve_coord.x_start);
  lane_marker->set_longitude_end(curve_coord.x_end);
}

static int GetGpuId(const camera::CameraPerceptionInitOptions &options) {
  camera::app::PerceptionParam perception_param;
  std::string work_root = camera::GetCyberWorkRoot();
  std::string config_file =
      GetAbsolutePath(options.root_dir, options.conf_file);
  config_file = GetAbsolutePath(work_root, config_file);
  if (!cyber::common::GetProtoFromFile(config_file, &perception_param)) {
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
  if (!apollo::cyber::common::PathExists(yaml_file)) {
    AINFO << yaml_file << " does not exist!";
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
  // TODO(techoe): This condition should be removed.
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
  if (InitConfig() != cyber::SUCC) {
    AERROR << "InitConfig() failed.";
    return false;
  }
  writer_ =
      node_->CreateWriter<PerceptionObstacles>(output_obstacles_channel_name_);
  sensorframe_writer_ =
      node_->CreateWriter<SensorFrameMessage>(prefused_channel_name_);
  camera_viz_writer_ = node_->CreateWriter<CameraPerceptionVizMessage>(
      camera_perception_viz_message_channel_name_);
  camera_debug_writer_ =
      node_->CreateWriter<apollo::perception::camera::CameraDebug>(
          camera_debug_channel_name_);
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
  if (InitMotionService() != cyber::SUCC) {
    AERROR << "InitMotionService() failed.";
    return false;
  }

  SetCameraHeightAndPitch();

  // Init visualizer
  // TODO(techoe, yg13): homography from image to ground should be
  // computed from camera height and pitch.
  // Apply online calibration to adjust pitch/height automatically
  // Temporary code is used here for test

  double pitch_adj_degree = 0.0;
  double yaw_adj_degree = 0.0;
  double roll_adj_degree = 0.0;
  // load in lidar to imu extrinsic
  Eigen::Matrix4d ex_lidar2imu;
  LoadExtrinsics(FLAGS_obs_sensor_intrinsic_path + "/" +
                     "velodyne128_novatel_extrinsics.yaml",
                 &ex_lidar2imu);
  AINFO << "velodyne128_novatel_extrinsics: " << ex_lidar2imu;

  CHECK(visualize_.Init_all_info_single_camera(
      camera_names_, visual_camera_, intrinsic_map_, extrinsic_map_,
      ex_lidar2imu, pitch_adj_degree, yaw_adj_degree, roll_adj_degree,
      image_height_, image_width_));

  homography_im2car_ = visualize_.homography_im2car(visual_camera_);
  camera_obstacle_pipeline_->SetIm2CarHomography(homography_im2car_);

  if (enable_cipv_) {
    cipv_.Init(homography_im2car_, min_laneline_length_for_cipv_,
               average_lane_width_in_meter_, max_vehicle_width_in_meter_,
               average_frame_rate_, image_based_cipv_, debug_level_);
  }

  if (enable_visualization_) {
    if (write_visual_img_) {
      visualize_.write_out_img_ = true;
      visualize_.SetDirectory(visual_debug_folder_);
    }
  }

  return true;
}

void FusionCameraDetectionComponent::OnReceiveImage(
    const std::shared_ptr<apollo::drivers::Image> &message,
    const std::string &camera_name) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double msg_timestamp = message->measurement_time() + timestamp_offset_;
  AINFO << "Enter FusionCameraDetectionComponent::Proc(), "
        << " camera_name: " << camera_name << " image ts: " << msg_timestamp;
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
    const double cur_time = apollo::common::time::Clock::NowInSeconds();
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
    if (MakeProtobufMsg(msg_timestamp, seq_num_, {}, {}, error_code,
                        out_message.get()) != cyber::SUCC) {
      AERROR << "MakeProtobufMsg failed";
      return;
    }
    if (output_final_obstacles_) {
      writer_->Write(out_message);
    }
    return;
  }

  bool send_sensorframe_ret = sensorframe_writer_->Write(prefused_message);
  AINFO << "send out prefused msg, ts: " << msg_timestamp
        << "ret: " << send_sensorframe_ret;
  // Send output msg
  if (output_final_obstacles_) {
    writer_->Write(out_message);
  }
  // for e2e lantency statistics
  {
    const double end_timestamp = apollo::common::time::Clock::NowInSeconds();
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
  visual_debug_folder_ = fusion_camera_detection_param.visual_debug_folder();
  visual_camera_ = fusion_camera_detection_param.visual_camera();
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
  write_visual_img_ = fusion_camera_detection_param.write_visual_img();

  min_laneline_length_for_cipv_ = static_cast<float>(
      fusion_camera_detection_param.min_laneline_length_for_cipv());
  average_lane_width_in_meter_ = static_cast<float>(
      fusion_camera_detection_param.average_lane_width_in_meter());
  max_vehicle_width_in_meter_ = static_cast<float>(
      fusion_camera_detection_param.max_vehicle_width_in_meter());
  average_frame_rate_ =
      static_cast<float>(fusion_camera_detection_param.average_frame_rate());

  image_based_cipv_ =
      static_cast<float>(fusion_camera_detection_param.image_based_cipv());

  debug_level_ = static_cast<int>(fusion_camera_detection_param.debug_level());
  enable_cipv_ = fusion_camera_detection_param.enable_cipv();

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
      visual_debug_folder_:     %s
      visual_camera_:     %s
      output_final_obstacles:    %s
      prefused_channel_name:    %s
      write_visual_img_:    %s)";
  std::string config_info_str =
      str(boost::format(format_str.c_str()) % camera_names_[0] %
          camera_names_[1] % camera_perception_init_options_.root_dir %
          camera_perception_init_options_.conf_file % frame_capacity_ %
          image_channel_num_ % enable_undistortion_ % enable_visualization_ %
          output_obstacles_channel_name_ %
          camera_perception_viz_message_channel_name_ % visual_debug_folder_ %
          visual_camera_ % output_final_obstacles_ % prefused_channel_name_ %
          write_visual_img_);
  AINFO << config_info_str;

  return cyber::SUCC;
}

int FusionCameraDetectionComponent::InitSensorInfo() {
  if (camera_names_.size() != 2) {
    AERROR << "invalid camera_names_.size(): " << camera_names_.size();
    return cyber::FAIL;
  }

  auto *sensor_manager = common::SensorManager::Instance();
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
          camera_names_[1] % tf_camera_frame_id_map_[camera_names_[0]] %
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
  AINFO << "camera_obstacle_pipeline_->Init() succeed";
  return cyber::SUCC;
}

int FusionCameraDetectionComponent::InitCameraFrames() {
  if (camera_names_.size() != 2) {
    AERROR << "invalid camera_names_.size(): " << camera_names_.size();
    return cyber::FAIL;
  }
  // fixed size
  camera_frames_.resize(frame_capacity_);
  if (camera_frames_.empty()) {
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
    model =
        common::SensorManager::Instance()->GetUndistortCameraModel(camera_name);
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
    AINFO << "#extrinsics of " << camera_name << ": "
          << extrinsic_map_[camera_name];
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

int FusionCameraDetectionComponent::InitMotionService() {
  const std::string &channel_name_local = "/apollo/perception/motion_service";
  std::function<void(const MotionServiceMsgType &)> motion_service_callback =
      std::bind(&FusionCameraDetectionComponent::OnMotionService, this,
                std::placeholders::_1);
  auto motion_service_reader =
      node_->CreateReader(channel_name_local, motion_service_callback);
  // initialize motion buffer
  if (motion_buffer_ == nullptr) {
    motion_buffer_ = std::make_shared<base::MotionBuffer>(motion_buffer_size_);
  } else {
    motion_buffer_->set_capacity(motion_buffer_size_);
  }
  return cyber::SUCC;
}

// On receiving motion service input, convert it to motion_buff_
void FusionCameraDetectionComponent::OnMotionService(
    const MotionServiceMsgType &message) {
  // Comment: use the circular buff to do it smartly, only push the latest
  // circular_buff only saves only the incremental motion between frames.
  // motion_service is now hard-coded for camera front 6mm
  base::VehicleStatus vehicledata;
  vehicledata.roll_rate = message->vehicle_status()[0].roll_rate();
  vehicledata.pitch_rate = message->vehicle_status()[0].pitch_rate();
  vehicledata.yaw_rate = message->vehicle_status()[0].yaw_rate();
  vehicledata.velocity = message->vehicle_status()[0].velocity();
  vehicledata.velocity_x = message->vehicle_status()[0].velocity_x();
  vehicledata.velocity_y = message->vehicle_status()[0].velocity_y();
  vehicledata.velocity_z = message->vehicle_status()[0].velocity_z();
  vehicledata.time_ts = message->vehicle_status()[0].time_ts();
  vehicledata.time_d = message->vehicle_status()[0].time_d();

  base::MotionType motion_2d = base::MotionType::Identity();
  motion_2d(0, 0) = message->vehicle_status()[0].motion().m00();
  motion_2d(0, 1) = message->vehicle_status()[0].motion().m01();
  motion_2d(0, 2) = message->vehicle_status()[0].motion().m02();
  motion_2d(0, 3) = message->vehicle_status()[0].motion().m03();
  motion_2d(1, 0) = message->vehicle_status()[0].motion().m10();
  motion_2d(1, 1) = message->vehicle_status()[0].motion().m11();
  motion_2d(1, 2) = message->vehicle_status()[0].motion().m12();
  motion_2d(1, 3) = message->vehicle_status()[0].motion().m13();
  motion_2d(2, 0) = message->vehicle_status()[0].motion().m20();
  motion_2d(2, 1) = message->vehicle_status()[0].motion().m21();
  motion_2d(2, 2) = message->vehicle_status()[0].motion().m22();
  motion_2d(2, 3) = message->vehicle_status()[0].motion().m23();
  motion_2d(3, 0) = message->vehicle_status()[0].motion().m30();
  motion_2d(3, 1) = message->vehicle_status()[0].motion().m31();
  motion_2d(3, 2) = message->vehicle_status()[0].motion().m32();
  motion_2d(3, 3) = message->vehicle_status()[0].motion().m33();
  vehicledata.motion = motion_2d;

  motion_buffer_->push_back(vehicledata);
  // TODO(@yg13): output motion in text file
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
    const std::string err_str =
        absl::StrCat("failed to get camera to world pose, ts: ", msg_timestamp,
                     " camera_name: ", camera_name);
    AERROR << err_str;
    *error_code = apollo::common::ErrorCode::PERCEPTION_ERROR_TF;
    prefused_message->error_code_ = *error_code;
    return cyber::FAIL;
  }
  Eigen::Affine3d world2camera = camera2world_trans.inverse();

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
           << " msg_timestamp: " << msg_timestamp;
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
                      camera_frame.lane_objects, *error_code,
                      out_message) != cyber::SUCC) {
    AERROR << "MakeProtobufMsg failed ts: " << msg_timestamp;
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

  //  Determine CIPV
  if (enable_cipv_) {
    CipvOptions cipv_options;
    if (motion_buffer_ != nullptr) {
      if (motion_buffer_->size() == 0) {
        AWARN << "motion_buffer_ is empty";
        cipv_options.velocity = 5.0f;
        cipv_options.yaw_rate = 0.0f;
      } else {
        cipv_options.velocity = motion_buffer_->back().velocity;
        cipv_options.yaw_rate = motion_buffer_->back().yaw_rate;
      }
      ADEBUG << "[CIPV] velocity " << cipv_options.velocity
             << ", yaw rate: " << cipv_options.yaw_rate;
      cipv_.DetermineCipv(camera_frame.lane_objects, cipv_options, world2camera,
                          &camera_frame.tracked_objects);

      // Get Drop points
      if (motion_buffer_->size() > 0) {
        cipv_.CollectDrops(motion_buffer_, world2camera,
                           &camera_frame.tracked_objects);
      } else {
        AWARN << "motion_buffer is empty";
      }
    }
  }

  // Send msg for visualization
  if (enable_visualization_) {
    camera::DataProvider::ImageOptions image_options;
    image_options.target_color = base::Color::BGR;
    std::shared_ptr<base::Blob<uint8_t>> image_blob(new base::Blob<uint8_t>);
    camera_frame.data_provider->GetImageBlob(image_options, image_blob.get());
    std::shared_ptr<CameraPerceptionVizMessage> viz_msg(
        new (std::nothrow) CameraPerceptionVizMessage(
            camera_name, msg_timestamp, camera2world_trans.matrix(), image_blob,
            camera_frame.tracked_objects, camera_frame.lane_objects,
            *error_code));
    bool send_viz_ret = camera_viz_writer_->Write(viz_msg);
    AINFO << "send out camera visualization msg, ts: " << msg_timestamp
          << " send_viz_ret: " << send_viz_ret;

    cv::Mat output_image(image_height_, image_width_, CV_8UC3,
                         cv::Scalar(0, 0, 0));
    base::Image8U out_image(image_height_, image_width_, base::Color::RGB);
    camera_frame.data_provider->GetImage(image_options, &out_image);
    memcpy(output_image.data, out_image.cpu_data(),
           out_image.total() * sizeof(uint8_t));
    visualize_.ShowResult_all_info_single_camera(output_image, camera_frame,
                                                 motion_buffer_, world2camera);
  }

  // send out camera debug message
  if (output_camera_debug_msg_) {
    // camera debug msg
    std::shared_ptr<apollo::perception::camera::CameraDebug> camera_debug_msg(
        new (std::nothrow) apollo::perception::camera::CameraDebug);
    if (MakeCameraDebugMsg(msg_timestamp, camera_name, camera_frame,
                           camera_debug_msg.get()) != cyber::SUCC) {
      AERROR << "make camera_debug_msg failed";
      return cyber::FAIL;
    }
    camera_debug_writer_->Write(camera_debug_msg);
  }

  return cyber::SUCC;
}

int FusionCameraDetectionComponent::MakeProtobufMsg(
    double msg_timestamp, int seq_num,
    const std::vector<base::ObjectPtr> &objects,
    const std::vector<base::LaneLine> &lane_objects,
    const apollo::common::ErrorCode error_code,
    apollo::perception::PerceptionObstacles *obstacles) {
  double publish_time = apollo::common::time::Clock::NowInSeconds();
  apollo::common::Header *header = obstacles->mutable_header();
  header->set_timestamp_sec(publish_time);
  header->set_module_name("perception_camera");
  header->set_sequence_num(seq_num);
  // in nanosecond
  // PnC would use lidar timestamp to predict
  header->set_lidar_timestamp(static_cast<uint64_t>(msg_timestamp * 1e9));
  header->set_camera_timestamp(static_cast<uint64_t>(msg_timestamp * 1e9));
  // header->set_radar_timestamp(0);

  // write out obstacles in world coordinates
  obstacles->set_error_code(error_code);
  for (const auto &obj : objects) {
    apollo::perception::PerceptionObstacle *obstacle =
        obstacles->add_perception_obstacle();
    if (ConvertObjectToPb(obj, obstacle) != cyber::SUCC) {
      AERROR << "ConvertObjectToPb failed, Object:" << obj->ToString();
      return cyber::FAIL;
    }
  }

  // write out lanes in ego coordinates
  apollo::perception::LaneMarkers *lane_markers =
      obstacles->mutable_lane_marker();
  apollo::perception::LaneMarker *lane_marker_l0 =
      lane_markers->mutable_left_lane_marker();
  apollo::perception::LaneMarker *lane_marker_r0 =
      lane_markers->mutable_right_lane_marker();
  apollo::perception::LaneMarker *lane_marker_l1 =
      lane_markers->add_next_left_lane_marker();
  apollo::perception::LaneMarker *lane_marker_r1 =
      lane_markers->add_next_right_lane_marker();

  for (const auto &lane : lane_objects) {
    base::LaneLineCubicCurve curve_coord = lane.curve_car_coord;

    switch (lane.pos_type) {
      case base::LaneLinePositionType::EGO_LEFT:
        fill_lane_msg(curve_coord, lane_marker_l0);
        break;
      case base::LaneLinePositionType::EGO_RIGHT:
        fill_lane_msg(curve_coord, lane_marker_r0);
        break;
      case base::LaneLinePositionType::ADJACENT_LEFT:
        fill_lane_msg(curve_coord, lane_marker_l1);
        break;
      case base::LaneLinePositionType::ADJACENT_RIGHT:
        fill_lane_msg(curve_coord, lane_marker_r1);
        break;
      default:
        break;
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

int FusionCameraDetectionComponent::ConvertObjectToCameraObstacle(
    const base::ObjectPtr &object_ptr,
    apollo::perception::camera::CameraObstacle *camera_obstacle) {
  if (camera_obstacle == nullptr) {
    AERROR << "camera_obstacle is not available";
    return false;
  }
  apollo::perception::PerceptionObstacle *obstacle =
      camera_obstacle->mutable_obstacle();
  ConvertObjectToPb(object_ptr, obstacle);

  camera_obstacle->set_type(
      static_cast<apollo::perception::camera::CameraObstacle::CameraType>(
          object_ptr->type));
  for (const auto &prob : object_ptr->type_probs) {
    camera_obstacle->add_type_probs(prob);
  }
  camera_obstacle->mutable_upper_left()->set_x(
      object_ptr->camera_supplement.box.xmin);
  camera_obstacle->mutable_upper_left()->set_y(
      object_ptr->camera_supplement.box.ymin);
  camera_obstacle->mutable_lower_right()->set_x(
      object_ptr->camera_supplement.box.xmax);
  camera_obstacle->mutable_lower_right()->set_y(
      object_ptr->camera_supplement.box.ymax);

  std::stringstream type_score_msg;
  type_score_msg << apollo::perception::camera::CameraObstacle::CameraType_Name(
                        camera_obstacle->type())
                 << ": "
                 << camera_obstacle->type_probs(camera_obstacle->type());
  camera_obstacle->add_debug_message(type_score_msg.str());

  return cyber::SUCC;
}

int FusionCameraDetectionComponent::ConvertLaneToCameraLaneline(
    const base::LaneLine &lane_line,
    apollo::perception::camera::CameraLaneLine *camera_laneline) {
  if (camera_laneline == nullptr) {
    AERROR << "camera_laneline is not available";
    return false;
  }
  // fill the lane line attribute
  apollo::perception::camera::LaneLineType line_type =
      static_cast<apollo::perception::camera::LaneLineType>(lane_line.type);
  camera_laneline->set_type(line_type);
  apollo::perception::camera::LaneLinePositionType pos_type =
      static_cast<apollo::perception::camera::LaneLinePositionType>(
          lane_line.pos_type);
  camera_laneline->set_pos_type(pos_type);
  camera_laneline->set_track_id(lane_line.track_id);
  camera_laneline->set_confidence(lane_line.confidence);
  apollo::perception::camera::LaneLineUseType use_type =
      static_cast<apollo::perception::camera::LaneLineUseType>(
          lane_line.use_type);
  camera_laneline->set_use_type(use_type);
  // fill the curve camera coord
  camera_laneline->mutable_curve_camera_coord()->set_longitude_min(
      lane_line.curve_camera_coord.x_start);
  camera_laneline->mutable_curve_camera_coord()->set_longitude_max(
      lane_line.curve_camera_coord.x_end);
  camera_laneline->mutable_curve_camera_coord()->set_a(
      lane_line.curve_camera_coord.a);
  camera_laneline->mutable_curve_camera_coord()->set_b(
      lane_line.curve_camera_coord.b);
  camera_laneline->mutable_curve_camera_coord()->set_c(
      lane_line.curve_camera_coord.c);
  camera_laneline->mutable_curve_camera_coord()->set_d(
      lane_line.curve_camera_coord.d);
  // fill the curve image coord
  camera_laneline->mutable_curve_image_coord()->set_longitude_min(
      lane_line.curve_image_coord.x_start);
  camera_laneline->mutable_curve_image_coord()->set_longitude_max(
      lane_line.curve_image_coord.x_end);
  camera_laneline->mutable_curve_image_coord()->set_a(
      lane_line.curve_image_coord.a);
  camera_laneline->mutable_curve_image_coord()->set_b(
      lane_line.curve_image_coord.b);
  camera_laneline->mutable_curve_image_coord()->set_c(
      lane_line.curve_image_coord.c);
  camera_laneline->mutable_curve_image_coord()->set_d(
      lane_line.curve_image_coord.d);
  // fill the curve image point set
  for (size_t i = 0; i < lane_line.curve_image_point_set.size(); i++) {
    const base::Point2DF &image_point2d = lane_line.curve_image_point_set[i];
    apollo::common::Point2D *lane_point_2d =
        camera_laneline->add_curve_image_point_set();
    lane_point_2d->set_x(image_point2d.x);
    lane_point_2d->set_y(image_point2d.y);
  }
  // fill the curve camera point set
  for (size_t i = 0; i < lane_line.curve_camera_point_set.size(); i++) {
    const base::Point3DF &point3d = lane_line.curve_camera_point_set[i];
    apollo::common::Point3D *lane_point_3d =
        camera_laneline->add_curve_camera_point_set();
    lane_point_3d->set_x(point3d.x);
    lane_point_3d->set_y(point3d.y);
    lane_point_3d->set_z(point3d.z);
  }
  // fill the line end point set
  for (size_t i = 0; i < lane_line.image_end_point_set.size(); i++) {
    const base::EndPoints &end_points = lane_line.image_end_point_set[i];
    apollo::perception::camera::EndPoints *lane_end_points =
        camera_laneline->add_image_end_point_set();
    lane_end_points->mutable_start()->set_x(end_points.start.x);
    lane_end_points->mutable_start()->set_y(end_points.start.y);
    lane_end_points->mutable_end()->set_x(end_points.end.x);
    lane_end_points->mutable_end()->set_y(end_points.end.y);
  }
  return cyber::SUCC;
}

int FusionCameraDetectionComponent::MakeCameraDebugMsg(
    double msg_timestamp, const std::string &camera_name,
    const camera::CameraFrame &camera_frame,
    apollo::perception::camera::CameraDebug *camera_debug_msg) {
  if (camera_debug_msg == nullptr) {
    AERROR << "camera_debug_msg is not available";
    return false;
  }

  auto itr = std::find(camera_names_.begin(), camera_names_.end(), camera_name);
  if (itr == camera_names_.end()) {
    AERROR << "invalid camera_name: " << camera_name;
    return cyber::FAIL;
  }
  int input_camera_channel_names_idx =
      static_cast<int>(itr - camera_names_.begin());
  int input_camera_channel_names_size =
      static_cast<int>(input_camera_channel_names_.size());
  if (input_camera_channel_names_idx < 0 ||
      input_camera_channel_names_idx >= input_camera_channel_names_size) {
    AERROR << "invalid input_camera_channel_names_idx: "
           << input_camera_channel_names_idx
           << " input_camera_channel_names_.size(): "
           << input_camera_channel_names_.size();
    return cyber::FAIL;
  }
  std::string source_channel_name =
      input_camera_channel_names_[input_camera_channel_names_idx];
  camera_debug_msg->set_source_topic(source_channel_name);

  // Fill header info.
  apollo::common::Header *header = camera_debug_msg->mutable_header();
  header->set_camera_timestamp(static_cast<uint64_t>(msg_timestamp * 1.0e9));

  // Fill the tracked objects
  const std::vector<base::ObjectPtr> &objects = camera_frame.tracked_objects;
  for (const auto &obj : objects) {
    apollo::perception::camera::CameraObstacle *camera_obstacle =
        camera_debug_msg->add_camera_obstacle();
    ConvertObjectToCameraObstacle(obj, camera_obstacle);
  }

  // Fill the laneline objects
  const std::vector<base::LaneLine> &lane_objects = camera_frame.lane_objects;
  for (const auto &lane_obj : lane_objects) {
    apollo::perception::camera::CameraLaneLine *camera_laneline =
        camera_debug_msg->add_camera_laneline();
    ConvertLaneToCameraLaneline(lane_obj, camera_laneline);
  }

  // Fill the calibrator information(pitch angle)
  float pitch_angle = camera_frame.calibration_service->QueryPitchAngle();
  camera_debug_msg->mutable_camera_calibrator()->set_pitch_angle(pitch_angle);
  return cyber::SUCC;
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
