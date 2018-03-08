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

bool CameraProcessSubnode::InitInternal() {
  //  Subnode config in DAG streaming
  std::map<string, string> fields;
  SubnodeHelper::ParseReserveField(reserve_, &fields));
  device_id_ = fields["device_id"];

  // Shared Data
  cam_obj_data_ = static_cast<CameraObjectData*>(
      shared_data_manager_->GetSharedData("CameraObjectData"));
  cam_shared_data_ = static_cast<CameraSharedData*>(
      shared_data_manager_->GetSharedData("CameraSharedData"));

  InitCalibration();

  InitModules();

  AdapterManager::AddImageShortCallback(&CameraProcessSubnode::ImgCallback,
                                        this);
  return true;
}

bool CameraProcessSubnode::InitCalibration() {
  auto ccm = base::Singleton<CalibrationConfigManager>::get();
  CameraCalibrationPtr calibrator = ccm->get_camera_calibration();

  const onboard::TransformInput &camera2car_trans =
      calibrator->get_camera_transform();
  camera2car_trans.query_pos(0.0, &camera_to_car_);

  intrinsics_ = calibrator->get_camera_intrinsic();
  undistortion_handler_ = calibrator->get_camera_undistort_handler();

  return true;
}

bool CameraProcessSubnode::InitModules() {
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
  out_objs->timestamp = timestamp;
  VisualObjToSensorObj(objects, &out_objs);

  onboard::SharedDataPtr<CameraItem> camera_item_ptr(new CameraItem);
  camera_item_ptr->image_src_mat = img.clone();
  mask.copyTo(out_objs->camera_frame_supplement->lane_map);
  PublishDataAndEvent(timestamp, out_objs, camera_item_ptr);
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
  out_objs->type = CAMERA;
  out_objs->name = get_sensor_name(CAMERA);
  out_objs->seq_num = seq_num_;
  out_objs->sensor2world_pose = camera_to_car_;
  (out_objs->camera_frame_supplement).reset(new CameraFrameSupplement);

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
    obj->anchor_point = obj->center;
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

void CameraProcessSubnode::PublishDataAndEvent(
    double timestamp,
    const onboard::SharedDataPtr<SensorObjects> &sensor_objects,
    const onboard::SharedDataPtr<CameraItem> &camera_item) {
  std::string key = "";
  SubnodeHelper::ProduceSharedDataKey(timestamp, device_id_, &key));

  cam_obj_data_->add(key, sensor_objects));
  cam_shared_data_->add(key, camera_item));

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
