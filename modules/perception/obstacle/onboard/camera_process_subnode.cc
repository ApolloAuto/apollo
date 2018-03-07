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

bool CameraProcessSubnode::InitInternal() {

  if (!init_shared_data()) {
    XLOG(ERROR) << "Failed to init shared data.";
    return false;
  }

  AlgRgsInit();

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

  XLOG(INFO) << "Init CameraProcessSubnode sucessfully";
  return true;
}

bool CameraProcessSubnode::init_shared_data() {
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

bool CameraProcessSubnode::AlgRgsInit() {
  detector_.reset(BaseCameraDetectorRegisterer::get_instance_by_name("YoloCameraDetector"));
  detector_->Init();

  converter_.reset(BaseCameraConverterRegisterer::get_instance_by_name("GeometryCameraConverter"));
  converter_->Init();

  tracker_.reset(BaseCameraTrackerRegisterer::get_instance_by_name("CascadedCameraTracker"));
  tracker_->Init();

  transformer_.reset(BaseCameraTransformerRegisterer::get_instance_by_name("FlatCameraTransformer"));
  transformer_->Init();
  transformer_->SetExtrinsics(camera_to_car_);

  filter_.reset(BaseCameraFilterRegisterer::get_instance_by_name("ObjectCameraFilter"));
  filter_->Init();

  lane_processor_.reset(BaseCameraLanePostProcessorRegisterer::get_instance_by_name("CCLanePostProcessor"));
  lane_processor_->Init();
  return true;
}

bool CameraProcessSubnode::init_calibration_input(
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

bool CameraProcessSubnode::init_subscriber(
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

  // register subscriber
  bool ret = _stream_input.register_subscriber(
      source_type, new_source_name, &CameraProcessSubnode::image_callback, this);
  if (!ret) {
    XLOG(ERROR) << "Failed to register input stream. [type: " << source_type
                << "] [name: " << new_source_name << "].";
    return false;
  }

  XLOG(INFO) << "Init subscriber successfully, source_type: " << source_type
             << ", source_name: " << new_source_name;
  return true;
}

void CameraProcessSubnode::ImgCallback(
    const sensor_msgs::Image::ConstPtr &message) {
  ++_seq_num;
  float timestamp = message->header.stamp.toSec();

  cv::Mat img;
  MessageToMat(message, &img));

  std::vector<VisualObjectPtr> objects;
  cv::Mat mask = cv::Mat::zeros(img.rows, img.cols, CV_32FC1);
  detector_->Multitask(img, CameraDetectorOptions(), &objects, &mask);
  converter_->Convert(&objects);
  tracker_->Associate(img, timestamp, &objects);
  transformer_->Transform(&objects);
  filter_->Filter(timestamp, &objects);

  std::shared_ptr<SensorObjects> out_objs(new SensorObjects);
  out_objs->type = CAMERA;
  out_objs->name = get_sensor_name(CAMERA);
  out_objs->timestamp = timestamp;
  out_objs->seq_num = _seq_num;
  out_objs->sensor2world_pose = camera_to_car_;
  (out_objs->camera_frame_supplement).reset(new CameraFrameSupplement);
  trans_visualobject_to_sensorobject(objects, &out_objs);

  onboard::SharedDataPtr<CameraItem> camera_item_ptr(new CameraItem);
  camera_item_ptr->image_src_mat = img.clone();
  mask.copyTo(out_objs->camera_frame_supplement->lane_map);
  publish_data_and_event(timestamp, out_objs, camera_item_ptr);
}

bool CameraProcessSubnode::MessageToMat(
    const sensor_msgs::Image::ConstPtr& message, cv::Mat* mat)
    cv_bridge::CvImageConstPtr cv_ptr =
        cv_bridge::toCvShare(message, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image;

    mat->create(img.rows, img.cols, CV_8UC3);
    _undistortion_handler->handle(img.data, mat->data);
    return true;
}

void CameraProcessSubnode::VisualObjToSensorObj(
    const std::vector<VisualObjectPtr> &objects,
    onboard::SharedDataPtr<SensorObjects>* sensor_objects)
  for (size_t i = 0; i < objects.size(); ++i) {
    VisualObjectPtr vobj = objects[i];
    ObjectPtr obj = new Object();

    obj->id = vobj->id;
    obj->direction = vobj->direction;
    obj->theta = vobj->theta;
    obj->center = vobj->center;
    obj->length = vobj->length;
    obj->width = vobj->width;
    obj->height = vobj->height;
    obj->type = vobj->type;
    obj->type_probs.assign(vobj->type_probs,
                           vobj->type_probs + MAX_OBJECT_TYPE);
    obj->track_id = vobj->track_id;
    obj->tracking_time = vobj->tracking_time;
    obj->latest_tracked_time = vobj->latest_tracked_time;
    obj->velocity = vobj->velocity;
    obj->velocity_uncertainty = vobj->velocity_uncertainty;
    obj->position_uncertainty = vobj->position_uncertainty;
    obj->anchor_point = obj->center;
    XLOG(INFO) << "Target " << obj->track_id << " velocity "
               << obj->velocity.transpose();
    Eigen::Vector3d point;
    point[2] = obj->center[2];
    Eigen::Matrix2f rotate;
    rotate << cos(obj->theta), -sin(obj->theta), sin(obj->theta),
        cos(obj->theta);
    (obj->camera_supplement).reset(new CameraSupplement());
    obj->camera_supplement->upper_left << vobj->upper_left[0],
        vobj->upper_left[1];
    obj->camera_supplement->lower_right << vobj->lower_right[0],
        vobj->lower_right[1];

    // obj->camera_supplement->alpha = vobj->alpha;
    // obj->camera_supplement->pts8.assign(vobj->pts8,
    //                                     vobj->pts8 + 16);

    ((*sensor_objects)->objects).emplace_back(obj);
  }
}

void CameraProcessSubnode::publish_data_and_event(
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
