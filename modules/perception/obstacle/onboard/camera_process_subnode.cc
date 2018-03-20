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

#include "modules/perception/traffic_light/util/color_space.h"

namespace apollo {
namespace perception {

using apollo::common::adapter::AdapterManager;

bool CameraProcessSubnode::InitInternal() {
  // Subnode config in DAG streaming
  std::map<std::string, std::string> fields;
  SubnodeHelper::ParseReserveField(reserve_, &fields);
  device_id_ = fields["device_id"];

  // Shared Data
  cam_obj_data_ = static_cast<CameraObjectData *>(
      shared_data_manager_->GetSharedData("CameraObjectData"));
  cam_shared_data_ = static_cast<CameraSharedData *>(
      shared_data_manager_->GetSharedData("CameraSharedData"));

  InitCalibration();

  InitModules();

  AdapterManager::AddImageShortCallback(&CameraProcessSubnode::ImgCallback,
                                        this);

  return true;
}

bool CameraProcessSubnode::InitCalibration() {
  auto ccm = Singleton<CalibrationConfigManager>::get();
  CameraCalibrationPtr calibrator = ccm->get_camera_calibration();

  camera_to_car_ = calibrator->get_camera_extrinsics();
  intrinsics_ = calibrator->get_camera_intrinsic();
  undistortion_handler_ = calibrator->get_camera_undistort_handler();
  return true;
}

bool CameraProcessSubnode::InitModules() {
  RegisterFactoryYoloCameraDetector();
  RegisterFactoryGeometryCameraConverter();
  RegisterFactoryCascadedCameraTracker();
  RegisterFactoryFlatCameraTransformer();
  RegisterFactoryObjectCameraFilter();

  detector_.reset(
      BaseCameraDetectorRegisterer::GetInstanceByName("YoloCameraDetector"));
  detector_->Init();

  converter_.reset(BaseCameraConverterRegisterer::GetInstanceByName(
      "GeometryCameraConverter"));
  converter_->Init();

  tracker_.reset(
      BaseCameraTrackerRegisterer::GetInstanceByName("CascadedCameraTracker"));
  tracker_->Init();

  transformer_.reset(BaseCameraTransformerRegisterer::GetInstanceByName(
      "FlatCameraTransformer"));
  transformer_->Init();
  // transformer_->SetExtrinsics(camera_to_car_);

  filter_.reset(
      BaseCameraFilterRegisterer::GetInstanceByName("ObjectCameraFilter"));
  filter_->Init();

  return true;
}

void CameraProcessSubnode::ImgCallback(const sensor_msgs::Image &message) {
  AdapterManager::Observe();
  sensor_msgs::Image msg = AdapterManager::GetImageShort()->GetLatestObserved();

  double timestamp = msg.header.stamp.toSec();
  AINFO << "CameraProcessSubnode ImgCallback: "
        << " frame: "<< ++seq_num_ << " timestamp: ";
  AINFO << std::fixed << std::setprecision(20) << timestamp;

  cv::Mat img;
  if (!FLAGS_image_file_debug) {
    MessageToMat(msg, &img);
  } else {
    img = cv::imread(FLAGS_image_file_path, CV_LOAD_IMAGE_COLOR);
  }

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

  SharedDataPtr<CameraItem> camera_item_ptr(new CameraItem);
  camera_item_ptr->image_src_mat = img.clone();
  mask.copyTo(out_objs->camera_frame_supplement->lane_map);
  PublishDataAndEvent(timestamp, out_objs, camera_item_ptr);
}

bool CameraProcessSubnode::MessageToMat(const sensor_msgs::Image &msg,
                                        cv::Mat *img) {
  cv::Mat cv_img;
  if (msg.encoding.compare("yuyv") == 0) {
    unsigned char *yuv = (unsigned char *)&(msg.data[0]);
    cv_img = cv::Mat(msg.height, msg.width, CV_8UC3);
    traffic_light::Yuyv2rgb(yuv, cv_img.data, msg.height * msg.width);
    cv::cvtColor(cv_img, cv_img, CV_RGB2BGR);
  } else {
    // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg.encoding);
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg,
      sensor_msgs::image_encodings::BGR8);
    cv_img = cv_ptr->image;
  }
  AINFO << "cv_img: " << cv_img.rows << " " << cv_img.cols;

  img->create(cv_img.rows, cv_img.cols, CV_8UC3);
  undistortion_handler_->handle(cv_img.data, img->data);

  return true;
}

void CameraProcessSubnode::VisualObjToSensorObj(
    const std::vector<VisualObjectPtr> &objects,
    SharedDataPtr<SensorObjects> *sensor_objects) {
  (*sensor_objects)->sensor_type = SensorType::CAMERA;
  (*sensor_objects)->sensor_id = device_id_;
  (*sensor_objects)->seq_num = seq_num_;
  (*sensor_objects)->sensor2world_pose = camera_to_car_;
  ((*sensor_objects)->camera_frame_supplement).reset(new CameraFrameSupplement);

  for (size_t i = 0; i < objects.size(); ++i) {
    VisualObjectPtr vobj = objects[i];
    ObjectPtr obj(new Object());

    obj->id = vobj->id;
    obj->direction = vobj->direction.cast<double>();
    obj->theta = vobj->theta;
    obj->center = vobj->center.cast<double>();
    obj->length = vobj->length;
    obj->width = vobj->width;
    obj->height = vobj->height;
    obj->type = vobj->type;
    obj->track_id = vobj->track_id;
    obj->tracking_time = vobj->track_age;
    obj->latest_tracked_time = vobj->last_track_timestamp;
    obj->velocity = vobj->velocity.cast<double>();
    obj->anchor_point = obj->center.cast<double>();
    (obj->camera_supplement).reset(new CameraSupplement());
    obj->camera_supplement->upper_left = vobj->upper_left.cast<double>();
    obj->camera_supplement->lower_right = vobj->lower_right.cast<double>();
    obj->camera_supplement->alpha = vobj->alpha;
    // obj->type_probs.assign(vobj->type_probs,
    //                        vobj->type_probs + MAX_OBJECT_TYPE);
    // obj->camera_supplement->pts8.assign(vobj->pts8,
    //                                     vobj->pts8 + 16);

    ((*sensor_objects)->objects).emplace_back(obj);
  }
}

void CameraProcessSubnode::PublishDataAndEvent(
    const double &timestamp, const SharedDataPtr<SensorObjects> &sensor_objects,
    const SharedDataPtr<CameraItem> &camera_item) {
  std::string key = "";
  SubnodeHelper::ProduceSharedDataKey(timestamp, device_id_, &key);

  cam_obj_data_->Add(key, sensor_objects);
  cam_shared_data_->Add(key, camera_item);

  for (size_t idx = 0; idx < pub_meta_events_.size(); ++idx) {
    const EventMeta &event_meta = pub_meta_events_[idx];
    Event event;
    event.event_id = event_meta.event_id;
    event.timestamp = timestamp;
    event.reserve = device_id_;
    event_manager_->Publish(event);
  }
}

}  // namespace perception
}  // namespace apollo
