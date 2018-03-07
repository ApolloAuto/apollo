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

#include "modules/perception/obstacle/onboard/camera_process_subnode.h"

namespace apollo {
namespace perception {

DEFINE_string(onboard_mix_camera_detector, "DummyCameraDetector",
              "onboard camera detector.");
DEFINE_string(onboard_mix_camera_transformer, "DummyCameraTransformer",
              "onboard camera transformer.");
DEFINE_string(onboard_mix_camera_tracker, "DummyCameraTracker",
              "onboard camera tracker.");
DEFINE_bool(use_center_buffer, false, "use center buffer");
using base::FileUtil;
using base::TimeUtil;
using common::perception::ERROR_TF;
using common::perception::ERROR_PROCESS;
using config_manager::ConfigManager;
using config_manager::CalibrationConfigManager;
using config_manager::CameraCalibrationPtr;
using Eigen::Matrix4d;
using onboard::Event;
using onboard::EventMeta;
using onboard::IoStreamType;
using onboard::SharedDataPtr;
using onboard::Subnode;
using onboard::SubnodeHelper;
using onboard::TransformType;
using std::endl;
using std::map;
using std::vector;
using std::string;

bool MixDetectorSubnode::init_internal() {
  // init shared data
  if (!init_shared_data()) {
    XLOG(ERROR) << "Failed to init shared data.";
    return false;
  }
  // init plugins
  if (!init_alg_plugins()) {
    XLOG(ERROR) << "Failed to init algorithm plugins.";
    return false;
  }
  // init work_root
  if (!init_work_root()) {
    XLOG(ERROR) << "Failed to init work root.";
    return false;
  }

  // parse reserve fileds
  map<string, string> reserve_field_map;
  if (!SubnodeHelper::parse_reserve_field(_reserve, &reserve_field_map)) {
    XLOG(ERROR) << "Failed to parse reserve filed: " << _reserve;
    return false;
  }

  // init transform input
  if (!init_calibration_input(reserve_field_map)) {
    XLOG(ERROR) << "Failed to init transform input: " << _reserve;
    return false;
  }

  // init subscriber
  if (!init_subscriber(reserve_field_map)) {
    XLOG(ERROR) << "Failed to init subscriber, reserve: " << _reserve;
    return false;
  }

  XLOG(INFO) << "Init MixDetectorSubnode sucessfully";
  return true;
}

bool MixDetectorSubnode::init_shared_data() {
  CHECK(_shared_data_manager != nullptr);
  // init preprocess_data
  _camera_object_data = dynamic_cast<CameraObjectData *>(
      _shared_data_manager->get_shared_data("CameraObjectData"));
  if (_camera_object_data == nullptr) {
    XLOG(ERROR) << "Failed to get shared data instance: CameraObjectData ";
    return false;
  }
  XLOG(INFO) << "Init shared data successfully, data: "
             << _camera_object_data->name();

  _camera_shared_data = dynamic_cast<CameraSharedData *>(
      _shared_data_manager->get_shared_data("CameraSharedData"));
  if (_camera_shared_data == nullptr) {
    XLOG(ERROR) << "Failed to get shared data instance: CameraSharedData ";
    return false;
  }
  XLOG(INFO) << "Init shared data successfully, data: "
             << _camera_shared_data->name();
  return true;
}

bool MixDetectorSubnode::init_alg_plugins() {
  // init camera detector
  _camera_detector.reset(BaseCameraDetectorRegisterer::get_instance_by_name(
      FLAGS_onboard_mix_camera_detector));
  if (!_camera_detector) {
    XLOG(ERROR) << "Failed to get instance: "
                << FLAGS_onboard_mix_camera_detector;
    return false;
  }
  if (!_camera_detector->init()) {
    XLOG(ERROR) << "Failed to init camera detector: "
                << _camera_detector->name();
    return false;
  }
  // init camera transformer
  _camera_transformer.reset(
      BaseCameraTransformerRegisterer::get_instance_by_name(
          FLAGS_onboard_mix_camera_transformer));
  if (!_camera_transformer) {
    XLOG(ERROR) << "Failed to get instance: "
                << FLAGS_onboard_mix_camera_transformer;
    return false;
  }
  if (!_camera_transformer->init()) {
    XLOG(ERROR) << "Failed to init camera transformer: "
                << _camera_transformer->name();
    return false;
  }

  // init camera tracker
  _camera_tracker.reset(BaseCameraTrackerRegisterer::get_instance_by_name(
      FLAGS_onboard_mix_camera_tracker));
  if (!_camera_tracker) {
    XLOG(ERROR) << "Failed to get instance: "
                << FLAGS_onboard_mix_camera_tracker;
    return false;
  }
  if (!_camera_tracker->init()) {
    XLOG(ERROR) << "Failed to init camera tracker: " << _camera_tracker->name();
    return false;
  }

  XLOG(INFO) << "Init alg pulgins successfully\n"
             << "  camera_detector:         "
             << FLAGS_onboard_mix_camera_detector << "\n"
             << "  camera_transformer:      "
             << FLAGS_onboard_mix_camera_transformer << "\n"
             << "  camera_tracker:          "
             << FLAGS_onboard_mix_camera_tracker;
  return true;
}

bool MixDetectorSubnode::init_work_root() {
  ConfigManager *config_manager = base::Singleton<ConfigManager>::get();
  if (config_manager == NULL) {
    XLOG(ERROR) << "failed to get ConfigManager instance.";
    return false;
  }

  if (!config_manager->init()) {
    XLOG(ERROR) << "failed to init ConfigManager";
    return false;
  }
  // get work root dir
  _work_root_dir = config_manager->work_root();

  XLOG(INFO) << "Init config manager successfully, work_root: "
             << _work_root_dir;
  return true;
}

bool MixDetectorSubnode::init_calibration_input(
    const map<string, string> &reserve_field_map) {
  CalibrationConfigManager *config_manager =
      base::Singleton<CalibrationConfigManager>::get();
  CameraCalibrationPtr calibrator = config_manager->get_camera_calibration();
  const onboard::TransformInput &camera2car_trans =
      calibrator->get_camera_transform();
  if (!camera2car_trans.query_pos(0.0, &_camera_to_car_mat)) {
    XLOG(ERROR) << "Failed to query camera to ego car space";
    return false;
  }
  _camera_intrinsic = calibrator->get_camera_intrinsic();
  _undistortion_handler = calibrator->get_camera_undistort_handler();

  XLOG(INFO) << "Init calibration successfully.";
  return true;
}

bool MixDetectorSubnode::init_subscriber(
    const map<string, string> &reserve_field_map) {
  auto citer = reserve_field_map.find("source_type");
  if (citer == reserve_field_map.end()) {
    XLOG(ERROR) << "Failed to find field source_type, reserve: " << _reserve;
    return false;
  }
  IoStreamType source_type =
      static_cast<IoStreamType>(atoi((citer->second).c_str()));
  citer = reserve_field_map.find("source_name");
  if (citer == reserve_field_map.end()) {
    XLOG(ERROR) << "Failed to find field source_name, reserve: " << _reserve;
    return false;
  }
  const string &source_name = citer->second;

  citer = reserve_field_map.find("device_id");
  if (citer == reserve_field_map.end()) {
    XLOG(ERROR) << "Failed to find field device_id, reserve: " << _reserve;
    return false;
  }
  _device_id = citer->second;

  string new_source_name = source_name;
  if (source_type == onboard::FILE_SOURCE) {
    new_source_name = FileUtil::get_absolute_path(_work_root_dir, source_name);
  }
  // register subscriber
  bool ret = _stream_input.register_subscriber(
      source_type, new_source_name, &MixDetectorSubnode::image_callback, this);
  if (!ret) {
    XLOG(ERROR) << "Failed to register input stream. [type: " << source_type
                << "] [name: " << new_source_name << "].";
    return false;
  }

  XLOG(INFO) << "Init subscriber successfully, source_type: " << source_type
             << ", source_name: " << new_source_name;
  return true;
}

bool MixDetectorSubnode::resize_image(
    const sensor_msgs::Image::ConstPtr &image_message_src,
    sensor_msgs::Image::Ptr image_message) {
  // copy image properties
  float divisor = static_cast<float>(image_message_src->width / 1920.0);
  image_message->header = image_message_src->header;
  image_message->height = image_message_src->height / divisor;
  image_message->width = image_message_src->width / divisor;
  image_message->encoding = image_message_src->encoding;
  image_message->is_bigendian = image_message_src->is_bigendian;
  image_message->step = image_message_src->step / divisor;
  image_message->data.resize(image_message->step * image_message->height);
  int data_length = image_message_src->step / image_message_src->width;
  // copy every divisorth byte
  // subpixels will be merged if multiple bytes per pixel
  uint new_index = 0;
  for (uint row = 0; row < image_message->height; ++row) {
    int row_offset = static_cast<int>(row * divisor) * image_message_src->step;
    for (uint col = 0; col < image_message->width; ++col) {
      int old_index =
          row_offset + static_cast<int>(col * divisor) * data_length;
      for (size_t k = 0; k < data_length; ++k) {
        image_message->data[new_index++] =
            image_message_src->data[old_index + k];
      }
    }
  }
}

void MixDetectorSubnode::image_callback(
    const sensor_msgs::Image::ConstPtr &image_message) {
  PERF_FUNCTION();
  // trans image
  cv::Mat image_mat_src;
  if (image_message->width != 1920) {
    sensor_msgs::Image::Ptr image_message_src =
        boost::make_shared<sensor_msgs::Image>();
    resize_image(image_message, image_message_src);
    if (!this->trans_message_to_cv_mat(image_message_src, &image_mat_src)) {
      XLOG(ERROR) << "trans messagea to cv mat error!";
      return;
    }
  } else {
    if (!this->trans_message_to_cv_mat(image_message, &image_mat_src)) {
      XLOG(ERROR) << "trans messagea to cv mat error!";
      return;
    }
  }
  image_process(image_mat_src, image_message->header.stamp.toSec());
}

void MixDetectorSubnode::image_process(const cv::Mat &image_mat_src,
                                       double time_stamp) {
  PERF_FUNCTION();
  ++_seq_num;
  const double timestamp = time_stamp;
  // const double unix_timestamp = base::TimeUtil::gps2unix(timestamp);
  const double unix_timestamp = timestamp;

  // format: FRAME_STATISTICS:device:Event:timestamp:currenttime
  const double cur_time = TimeUtil::get_current_time();
  const double start_latency = (cur_time - unix_timestamp) * 1e3;

  XLOG(INFO) << "FRAME_STATISTICS:Detector:Start:msg_time["
             << GLOG_TIMESTAMP(timestamp) << "]:cur_time["
             << GLOG_TIMESTAMP(cur_time) << "]:cur_latency[" << start_latency
             << "]";

  // create output sensor object
  std::shared_ptr<SensorObjects> out_sensor_objects(new SensorObjects);
  out_sensor_objects->type = CAMERA;
  out_sensor_objects->name = get_sensor_name(CAMERA);
  out_sensor_objects->timestamp = timestamp;
  out_sensor_objects->seq_num = _seq_num;
  // create frame supplements for SensorObjects
  (out_sensor_objects->camera_frame_supplement)
      .reset(new CameraFrameSupplement);

  onboard::SharedDataPtr<CameraItem> camera_item_ptr(new CameraItem);

  camera_item_ptr->image_src_mat = image_mat_src.clone();

  XLOG(INFO) << "trans message to cv mat success";

  // get trans matrix for camera -> car
  Matrix4d camera_to_car_pose;
  if (!this->get_camera_car_trans(timestamp, &camera_to_car_pose)) {
    XLOG(ERROR) << "Failed to get_camera_car_trans at time: "
                << GLOG_TIMESTAMP(timestamp);
    out_sensor_objects->error_code = ERROR_TF;
    publish_data_and_event(timestamp, out_sensor_objects, camera_item_ptr);
    return;
  }

  out_sensor_objects->sensor2world_pose = camera_to_car_pose;

  XLOG(INFO) << "get camera car trans success";

  XLOG(DEBUG) << "Camera2World Transform matrix: \n"
              << camera_to_car_pose << ", time: " << GLOG_TIMESTAMP(timestamp);

  PERF_BLOCK_START();

  // camera detect and track
  CameraDetectorOptions camera_detector_options;
  CameraTrackerOptions camera_tracker_options(&camera_to_car_pose);
  std::vector<VisualObjectPtr> track_objects;
  std::vector<VisualObjectPtr> visual_objects;
  cv::Mat lane_map =
      cv::Mat::zeros(image_mat_src.rows, image_mat_src.cols, CV_32FC1);

  if (!_camera_detector->multitask(image_mat_src, camera_detector_options,
                                   &visual_objects, &lane_map)) {
    XLOG(ERROR) << "Failed to detect and parse.";
    out_sensor_objects->error_code = ERROR_PROCESS;
    publish_data_and_event(timestamp, out_sensor_objects, camera_item_ptr);
    return;
  }

  PERF_BLOCK_END("camera_detect");
  if (!_camera_tracker->associate(image_mat_src, visual_objects, timestamp,
                                  camera_tracker_options, &visual_objects)) {
    XLOG(ERROR) << "Failed to associate.";
    out_sensor_objects->error_code = ERROR_PROCESS;
    publish_data_and_event(timestamp, out_sensor_objects, camera_item_ptr);
    return;
  }

  PERF_BLOCK_END("camera_smooth_center");
  if (FLAGS_use_center_buffer &&
      !_camera_tracker->smooth_center(&visual_objects)) {
    XLOG(ERROR) << "Failed to associate.";
    out_sensor_objects->error_code = ERROR_PROCESS;
    publish_data_and_event(timestamp, out_sensor_objects, camera_item_ptr);
    return;
  }

  PERF_BLOCK_END("camera_associate");
  if (!_camera_tracker->predict_shape(image_mat_src, visual_objects, timestamp,
                                      camera_tracker_options,
                                      &visual_objects)) {
    XLOG(ERROR) << "Failed to predict_shape.";
    out_sensor_objects->error_code = ERROR_PROCESS;
    publish_data_and_event(timestamp, out_sensor_objects, camera_item_ptr);
    return;
  }
  PERF_BLOCK_END("camera_predict_in_2d");
  // bbox transform
  CameraTransformerOptions camera_transformer_options;
  if (!_camera_transformer->transform(image_mat_src, camera_transformer_options,
                                      &visual_objects)) {
    XLOG(ERROR) << "Failed to transform.";
    out_sensor_objects->error_code = ERROR_PROCESS;
    publish_data_and_event(timestamp, out_sensor_objects, camera_item_ptr);
    return;
  }
  XLOG(INFO) << "camera transform visual_objects num: "
             << visual_objects.size();

  PERF_BLOCK_END("camera_transform");

  // camera tracking
  if (!_camera_tracker->predict_velocity(image_mat_src, visual_objects,
                                         timestamp, camera_tracker_options,
                                         &track_objects)) {
    XLOG(ERROR) << "Failed to track.";
    out_sensor_objects->error_code = ERROR_PROCESS;
    publish_data_and_event(timestamp, out_sensor_objects, camera_item_ptr);
    return;
  }
  XLOG(INFO) << "camera track track_objects num: " << track_objects.size();

  PERF_BLOCK_END("camera_tracking");

  // lane parsing
  CameraParserOptions camera_parser_options;

  // transform objects 3D boundingbox information to car
  this->trans_visualobject_to_sensorobject(track_objects, &out_sensor_objects);

  // update elements in sensorobjects
  this->update_l3_sensorobject_elements(&out_sensor_objects);

  // add parsing mats as supplements of SensorObjects
  lane_map.copyTo(out_sensor_objects->camera_frame_supplement->lane_map);

  XLOG(INFO) << "before publish camera objects, objects num: "
             << out_sensor_objects->objects.size();
  publish_data_and_event(timestamp, out_sensor_objects, camera_item_ptr);
  const double end_timestamp = base::TimeUtil::get_current_time();
  const double end_latency = (end_timestamp - unix_timestamp) * 1e3;
}

bool MixDetectorSubnode::trans_message_to_cv_mat(
    const sensor_msgs::Image::ConstPtr &image_msg, cv::Mat *mat) {
  try {
    cv_bridge::CvImageConstPtr cv_ptr =
        cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat tmp_img = cv_ptr->image;
    mat->create(tmp_img.rows, tmp_img.cols, CV_8UC3);
    _undistortion_handler->handle(tmp_img.data, mat->data);
  } catch (cv_bridge::Exception &e) {
    XLOG(ERROR) << "trans image error, cv_bridge exception: " << e.what();
    return false;
  }
  return true;
}

bool MixDetectorSubnode::get_camera_car_trans(double timestamp,
                                              Matrix4d *camera_to_car_pose) {
  *camera_to_car_pose = _camera_to_car_mat;
  XLOG(INFO) << "Timestamp: " << GLOG_TIMESTAMP(timestamp)
             << "Camera2Car matrix: " << endl
             << *camera_to_car_pose;

  return true;
}
void MixDetectorSubnode::trans_visualobject_to_sensorobject(
    const std::vector<VisualObjectPtr> track_objects,
    onboard::SharedDataPtr<SensorObjects> *sensor_objects) {
  for (size_t i = 0; i < track_objects.size(); ++i) {
    ObjectPtr obj(new Object);
    (obj->camera_supplement).reset(new CameraSupplement);

    obj->id = track_objects[i]->id;
    obj->direction = track_objects[i]->direction;
    obj->theta = track_objects[i]->theta;
    obj->center = track_objects[i]->center;
    obj->length = track_objects[i]->length;
    obj->width = track_objects[i]->width;
    obj->height = track_objects[i]->height;
    obj->type = track_objects[i]->type;
    obj->type_probs.assign(track_objects[i]->type_probs,
                           track_objects[i]->type_probs + MAX_OBJECT_TYPE);
    // obj->internal_type = track_objects[i]->internal_type;
    // obj->internal_type_probs = track_objects[i]->internal_type_probs;
    obj->track_id = track_objects[i]->track_id;
    obj->tracking_time = track_objects[i]->tracking_time;
    obj->latest_tracked_time = track_objects[i]->latest_tracked_time;
    obj->velocity = track_objects[i]->velocity;
    obj->velocity_uncertainty = track_objects[i]->velocity_uncertainty;
    obj->position_uncertainty = track_objects[i]->position_uncertainty;
    obj->anchor_point = obj->center;
    XLOG(INFO) << "Target " << obj->track_id << " velocity "
               << obj->velocity.transpose();
    Eigen::Vector3d point;
    point[2] = obj->center[2];
    Eigen::Matrix2f rotate;
    rotate << cos(obj->theta), -sin(obj->theta), sin(obj->theta),
        cos(obj->theta);
    obj->camera_supplement->upper_left << track_objects[i]->upper_left[0],
        track_objects[i]->upper_left[1];
    obj->camera_supplement->lower_right << track_objects[i]->lower_right[0],
        track_objects[i]->lower_right[1];

    obj->camera_supplement->alpha = track_objects[i]->alpha;
    obj->camera_supplement->pts8.assign(track_objects[i]->pts8,
                                        track_objects[i]->pts8 + 16);

    ((*sensor_objects)->objects).push_back(obj);
  }
}

void MixDetectorSubnode::update_l3_sensorobject_elements(
    onboard::SharedDataPtr<SensorObjects> *sensor_objects) {
  for (size_t i = 0; i < (*sensor_objects)->objects.size(); ++i) {
    ObjectPtr &obj = (*sensor_objects)->objects[i];
    obj->heading_c = atan2(obj->center.y(), obj->center.x());
    obj->heading_l = atan2(obj->center.y() + obj->width / 2, obj->center.x());
    obj->heading_r = atan2(obj->center.y() - obj->width / 2, obj->center.x());
  }
}
void MixDetectorSubnode::publish_data_and_event(
    double timestamp,
    const onboard::SharedDataPtr<SensorObjects> &sensor_objects,
    const onboard::SharedDataPtr<CameraItem> &camera_item) {
  string key;
  if (!SubnodeHelper::produce_shared_data_key(timestamp, _device_id, &key)) {
    XLOG(ERROR) << "Failed to produce shared key. time: "
                << GLOG_TIMESTAMP(timestamp) << ", device_id: " << _device_id;
    return;
  }

  if (!_camera_object_data->add(key, sensor_objects)) {
    XLOG(WARN) << "Failed to add CameraObjectData. key: " << key
               << " num_detected_objects: " << (sensor_objects->objects).size();
    return;
  }
  if (!_camera_shared_data->add(key, camera_item)) {
    XLOG(WARN) << "Failed to add CameraSharedData. key: " << key;
    return;
  }
  // pub events
  for (size_t idx = 0; idx < _pub_meta_events.size(); ++idx) {
    const EventMeta &event_meta = _pub_meta_events[idx];
    Event event;
    event.event_id = event_meta.event_id;
    event.timestamp = timestamp;
    event.reserve = _device_id;
    _event_manager->publish(event);
  }
  XLOG(INFO) << "publish data and event success.";
}

}  // namespace perception
}  // namespace apollo
