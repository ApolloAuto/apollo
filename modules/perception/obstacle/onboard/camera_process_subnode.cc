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
#include "eigen_conversions/eigen_msg.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/onboard/transform_input.h"

namespace apollo {
namespace perception {

using apollo::common::adapter::AdapterManager;
using Eigen::Affine3d;
using Eigen::Matrix4d;

bool CameraProcessSubnode::InitInternal() {
  // Subnode config in DAG streaming
  std::unordered_map<std::string, std::string> fields;
  SubnodeHelper::ParseReserveField(reserve_, &fields);

  if (fields.count("device_id")) {
    device_id_ = fields["device_id"];
  }
  if (fields.count("pb_obj") && stoi(fields["pb_obj"])) {
    pb_obj_ = true;
  }
  if (fields.count("pb_ln_msk") && stoi(fields["pb_ln_msk"])) {
    pb_ln_msk_ = true;
  }

  // Shared Data
  cam_obj_data_ = static_cast<CameraObjectData *>(
      shared_data_manager_->GetSharedData("CameraObjectData"));
  cam_shared_data_ = static_cast<CameraSharedData *>(
      shared_data_manager_->GetSharedData("CameraSharedData"));
  if (FLAGS_enable_pseudo_narrow) {
    cam_obj_data_pnarrow_ = static_cast<CameraObjectData *>(
        shared_data_manager_->GetSharedData("CameraObjectData"));
    cam_shared_data_pnarrow_ = static_cast<CameraSharedData *>(
        shared_data_manager_->GetSharedData("CameraSharedData"));
  }

  InitCalibration();

  InitModules();

  AdapterManager::AddImageFrontCallback(&CameraProcessSubnode::ImgCallback,
                                        this);
  if (pb_obj_) {
    AdapterManager::AddChassisCallback(&CameraProcessSubnode::ChassisCallback,
                                       this);
  }
  return true;
}

bool CameraProcessSubnode::InitCalibration() {
  auto ccm = Singleton<CalibrationConfigManager>::get();
  CameraCalibrationPtr calibrator = ccm->get_camera_calibration();

  calibrator->get_image_height_width(&image_height_, &image_width_);
  camera_to_car_ = calibrator->get_camera_extrinsics();
  intrinsics_ = calibrator->get_camera_intrinsic();

  // init for pseudo_narrow
  // This should be in init function
  if (FLAGS_enable_pseudo_narrow) {
    if (fabs(FLAGS_pseudo_narrow_scale) < dEpsilon) {
      AERROR << "The parameter pseudo_narrow_scale ("
             << FLAGS_pseudo_narrow_scale
             << ") is too small or negative";
      return false;
    }
    pseudo_scale_ = FLAGS_pseudo_narrow_scale;
    inv_pseudo_scale_ = 1.0f / pseudo_scale_;
    int trans_x = static_cast<int>(
                   (1.0f - inv_pseudo_scale_) * image_width_ * 0.5f);
    int trans_y = static_cast<int>(
                   (1.0f - inv_pseudo_scale_) * image_height_ * 0.5f);
    scale_mat_ << inv_pseudo_scale_, 0, trans_x,
                 0, inv_pseudo_scale_, trans_y,
                 0, 0, 1;
    AINFO << "scale_mat_: " << scale_mat_;
    int ps_width = static_cast<int>(image_width_ * inv_pseudo_scale_);
    int ps_height = static_cast<int>(image_height_ * inv_pseudo_scale_);
    pseudo_narrow_roi_ = cv::Rect_<int>(trans_x, trans_y, ps_width, ps_height);
    AINFO << "pseudo_narrow_roi_: " << pseudo_narrow_roi_;

    Eigen::Matrix<double, 3, 3> H_camera2car
      = calibrator->get_camera2car_homography_mat();
    Eigen::Matrix<double, 3, 3> H_car2camera
      = calibrator->get_car2camera_homography_mat();
    pseudo2camera_mat_ = H_camera2car * scale_mat_ * H_car2camera;
    AINFO << "pseudo2camera_mat_: " << pseudo2camera_mat_;
  }

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
  transformer_->SetExtrinsics(camera_to_car_);

  filter_.reset(
      BaseCameraFilterRegisterer::GetInstanceByName("ObjectCameraFilter"));
  filter_->Init();

  return true;
}

void CameraProcessSubnode::ImgCallback(const sensor_msgs::Image &message) {
  double timestamp = message.header.stamp.toSec();
  ADEBUG << "CameraProcessSubnode ImgCallback: timestamp: ";
  ADEBUG << std::fixed << std::setprecision(64) << timestamp;
  AINFO << "camera received image : " << GLOG_TIMESTAMP(timestamp)
        << " at time: " << GLOG_TIMESTAMP(TimeUtil::GetCurrentTime());
  double curr_timestamp = timestamp * 1e9;

  if (FLAGS_skip_camera_frame && timestamp_ns_ > 0.0) {
    if ((curr_timestamp - timestamp_ns_) < (1e9 / FLAGS_camera_hz) &&
        curr_timestamp > timestamp_ns_) {
      ADEBUG << "CameraProcessSubnode Skip frame";
      return;
    }
  }

  timestamp_ns_ = curr_timestamp;
  ADEBUG << "CameraProcessSubnode Process:  frame: " << ++seq_num_;
  PERF_FUNCTION("CameraProcessSubnode");
  PERF_BLOCK_START();

  cv::Mat img;
  if (!FLAGS_image_file_debug) {
    MessageToMat(message, &img);
  } else {
    img = cv::imread(FLAGS_image_file_path, CV_LOAD_IMAGE_COLOR);
  }
  if (img.cols != 1920 || img.rows != 1080) {
    cv::resize(img, img, cv::Size(1920, 1080), 0, 0);
  }
  FilterOptions options;
  if (FLAGS_use_navigation_mode) {
    options.camera_trans = std::make_shared<Eigen::Matrix4d>();
    options.camera_trans->setIdentity();
  } else {
    options.camera_trans = std::make_shared<Eigen::Matrix4d>();
    if (!GetCameraTrans(timestamp, options.camera_trans.get())) {
      AERROR << "failed to get trans at timestamp: " << timestamp;
      return;
    }
  }
  std::shared_ptr<SensorObjects> out_objs(new SensorObjects);
  if (FLAGS_enable_pseudo_narrow) {
    cv::Mat cropped_image = img(pseudo_narrow_roi_);
    cv::resize(cropped_image, cropped_image, cv::Size(1920, 1080), 0, 0);

    std::vector<std::shared_ptr<VisualObject>> objects;
    cv::Mat mask;

    PERF_BLOCK_END("CameraProcessSubnode_Image_Preprocess");
    detector_->Multitask(cropped_image, CameraDetectorOptions(),
                         &objects, &mask);

    // cv::Mat mask_color(mask.rows, mask.cols, CV_32FC1);
    // if (FLAGS_use_whole_lane_line) {
    //   std::vector<cv::Mat> masks;
    //   detector_->Lanetask(cropped_image, &masks);
    //   mask_color.setTo(cv::Scalar(0));
    //   ln_msk_threshold_ = 0.9;
    //   for (int c = 0; c < num_lines; ++c) {
    //     for (int h = 0; h < masks[c].rows; ++h) {
    //       for (int w = 0; w < masks[c].cols; ++w) {
    //         if (masks[c].at<float>(h, w) >= ln_msk_threshold_) {
    //           mask_color.at<float>(h, w) = static_cast<float>(c);
    //         }
    //       }
    //     }
    //   }
    // } else {
    //   mask.copyTo(mask_color);
    //   ln_msk_threshold_ = 0.5;
    //   for (int h = 0; h < mask_color.rows; ++h) {
    //     for (int w = 0; w < mask_color.cols; ++w) {
    //       if (mask_color.at<float>(h, w) >= ln_msk_threshold_) {
    //         mask_color.at<float>(h, w) = static_cast<float>(5);
    //       }
    //     }
    //   }
    // }

    PERF_BLOCK_END("CameraProcessSubnode_detector_");

    converter_->Convert(&objects);
    PERF_BLOCK_END("CameraProcessSubnode_converter_");

    auto ccm = Singleton<CalibrationConfigManager>::get();
    // auto calibrator = ccm->get_camera_calibration();
    // calibrator->SetCar2CameraExtrinsicsAdjPseudoNarrow(camera_to_car_adj_,
    //                                        ps_start_x,
    //                                        ps_start_y,
    //                                        inv_pseudo_scale_,
    //                                        adjusted_extrinsics_);
    auto calibrator = ccm->get_camera_calibration();
    calibrator->SetCar2CameraExtrinsicsAdj(camera_to_car_adj_,
                                           adjusted_extrinsics_);

    transformer_->Transform(&objects);
    adjusted_extrinsics_ =
    transformer_->GetAdjustedExtrinsics(&camera_to_car_adj_);
    PERF_BLOCK_END("CameraProcessSubnode_transformer_");

    tracker_->Associate(cropped_image, timestamp, &objects);
    PERF_BLOCK_END("CameraProcessSubnode_tracker_");

    filter_->Filter(timestamp, &objects, options);
    PERF_BLOCK_END("CameraProcessSubnode_filter_");

    out_objs->sensor_type = SensorType::CAMERA_PSEUDO_NARROW;

    out_objs->timestamp = timestamp;
    VisualObjToSensorObj(objects, &out_objs, options);

    SharedDataPtr<CameraItem> camera_item_ptr(new CameraItem);
//    camera_item_ptr->image_src_mat = cropped_image.clone();
//    camera_item_ptr->image_src_mat = img.clone();

//    mask_color.copyTo(out_objs->camera_frame_supplement->lane_map);
//    PublishDataAndEvent(timestamp, out_objs, camera_item_ptr);
    PublishDataAndEventPsuedoNarrow(timestamp, out_objs, camera_item_ptr);
    PERF_BLOCK_END("CameraProcessSubnode Pseudo Narrow publish in DAG");

    // if (pb_obj_) PublishPerceptionPbObj(out_objs);
    // if (pb_ln_msk_) PublishPerceptionPbLnMsk(mask_color, message);
  }
  std::vector<std::shared_ptr<VisualObject>> objects;
  cv::Mat mask;

  PERF_BLOCK_END("CameraProcessSubnode_Image_Preprocess");
  detector_->Multitask(img, CameraDetectorOptions(), &objects, &mask);

  cv::Mat mask_color(mask.rows, mask.cols, CV_32FC1);
  if (FLAGS_use_whole_lane_line) {
    std::vector<cv::Mat> masks;
    detector_->Lanetask(img, &masks);
    mask_color.setTo(cv::Scalar(0));
    ln_msk_threshold_ = 0.9;
    for (int c = 0; c < num_lines; ++c) {
      for (int h = 0; h < masks[c].rows; ++h) {
        for (int w = 0; w < masks[c].cols; ++w) {
          if (masks[c].at<float>(h, w) >= ln_msk_threshold_) {
            mask_color.at<float>(h, w) = static_cast<float>(c);
          }
        }
      }
    }
  } else {
    mask.copyTo(mask_color);
    ln_msk_threshold_ = 0.5;
    for (int h = 0; h < mask_color.rows; ++h) {
      for (int w = 0; w < mask_color.cols; ++w) {
        if (mask_color.at<float>(h, w) >= ln_msk_threshold_) {
          mask_color.at<float>(h, w) = static_cast<float>(5);
        }
      }
    }
  }

  PERF_BLOCK_END("CameraProcessSubnode_detector_");

  converter_->Convert(&objects);
  PERF_BLOCK_END("CameraProcessSubnode_converter_");

  if (FLAGS_use_navigation_mode) {
    transformer_->Transform(&objects);
    adjusted_extrinsics_ =
        transformer_->GetAdjustedExtrinsics(&camera_to_car_adj_);
    PERF_BLOCK_END("CameraProcessSubnode_transformer_");
  }

  tracker_->Associate(img, timestamp, &objects);
  PERF_BLOCK_END("CameraProcessSubnode_tracker_");

  camera_to_world_ = *(options.camera_trans);
  // need to create camera options here for filter
  filter_->Filter(timestamp, &objects, options);
  PERF_BLOCK_END("CameraProcessSubnode_filter_");

  auto ccm = Singleton<CalibrationConfigManager>::get();
  auto calibrator = ccm->get_camera_calibration();
  calibrator->SetCar2CameraExtrinsicsAdj(camera_to_car_adj_,
                                         adjusted_extrinsics_);

//  std::shared_ptr<SensorObjects> out_objs(new SensorObjects);
  out_objs->sensor_type = SensorType::CAMERA;
  out_objs->timestamp = timestamp;
  VisualObjToSensorObj(objects, &out_objs, options);

  SharedDataPtr<CameraItem> camera_item_ptr(new CameraItem);
  camera_item_ptr->image_src_mat = img;
  mask_color.copyTo(out_objs->camera_frame_supplement->lane_map);
  PublishDataAndEvent(timestamp, out_objs, camera_item_ptr);
  PERF_BLOCK_END("CameraProcessSubnode publish in DAG");

  if (pb_obj_) PublishPerceptionPbObj(out_objs);
  if (pb_ln_msk_) PublishPerceptionPbLnMsk(mask_color, message);
}

void CameraProcessSubnode::ChassisCallback(
    const apollo::canbus::Chassis &message) {
  chassis_.CopyFrom(message);
}

bool CameraProcessSubnode::MessageToMat(const sensor_msgs::Image &msg,
                                        cv::Mat *img) {
  *img = cv::Mat(msg.height, msg.width, CV_8UC3);
  int pixel_num = msg.width * msg.height;
  if (msg.encoding.compare("yuyv") == 0) {
    unsigned char *yuv = (unsigned char *)&(msg.data[0]);
    yuyv2bgr(yuv, img->data, pixel_num);
  } else {
    cv_bridge::CvImagePtr cv_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    *img = cv_ptr->image;
  }

  return true;
}

bool CameraProcessSubnode::MatToMessage(const cv::Mat &img,
                                        sensor_msgs::Image *msg) {
  if (img.type() == CV_8UC1) {
    sensor_msgs::fillImage(*msg, sensor_msgs::image_encodings::MONO8, img.rows,
                           img.cols, static_cast<unsigned int>(img.step),
                           img.data);
    return true;
  } else if (img.type() == CV_32FC1) {
    cv::Mat uc_img(img.rows, img.cols, CV_8UC1);
    uc_img.setTo(cv::Scalar(0));
    for (int h = 0; h < uc_img.rows; ++h) {
      for (int w = 0; w < uc_img.cols; ++w) {
        if (img.at<float>(h, w) >= ln_msk_threshold_) {
          uc_img.at<unsigned char>(h, w) = 1;
        }
      }
    }

    sensor_msgs::fillImage(*msg, sensor_msgs::image_encodings::MONO8,
                           uc_img.rows, uc_img.cols,
                           static_cast<unsigned int>(uc_img.step), uc_img.data);
    return true;
  } else {
    AERROR << "invalid input Mat type: " << img.type();
    return false;
  }
}
bool CameraProcessSubnode::ScaleBack(
    const Eigen::Matrix<double, 3, 3> &scale_mat,
    const Eigen::Vector2d &point2D,
    Eigen::Vector2d *scaled_point2D) {

  Eigen::Vector3d point2DH;
  Eigen::Vector3d point2d_scale_back;
  point2DH << point2D.x(), point2D.y(), 1.0;
  point2d_scale_back = scale_mat * point2DH;
  if (fabs(point2d_scale_back.z()) < dEpsilon) {
    return false;
  }
  scaled_point2D->x() = point2d_scale_back.x() / point2d_scale_back.z();
  scaled_point2D->y() = point2d_scale_back.y() / point2d_scale_back.z();
  return true;
}

void CameraProcessSubnode::VisualObjToSensorObj(
    const std::vector<std::shared_ptr<VisualObject>> &objects,
    SharedDataPtr<SensorObjects> *sensor_objects, FilterOptions options) {
  if ((*sensor_objects)->sensor_type == SensorType::UNKNOWN_SENSOR_TYPE) {
    (*sensor_objects)->sensor_type = SensorType::CAMERA;
  }
  (*sensor_objects)->sensor_id = device_id_;
  (*sensor_objects)->seq_num = seq_num_;
  if (FLAGS_use_navigation_mode) {
    (*sensor_objects)->sensor2world_pose_static = camera_to_car_;
    (*sensor_objects)->sensor2world_pose = camera_to_car_adj_;
  } else {
    (*sensor_objects)->sensor2world_pose_static = *(options.camera_trans);
    (*sensor_objects)->sensor2world_pose = *(options.camera_trans);
    AINFO << "camera process sensor2world pose is "
          << (*sensor_objects)->sensor2world_pose;
  }
  ((*sensor_objects)->camera_frame_supplement).reset(new CameraFrameSupplement);

  if (!CameraFrameSupplement::state_vars.initialized_) {
    CameraFrameSupplement::state_vars.process_noise *= 10;
    CameraFrameSupplement::state_vars.trans_matrix.block(0, 0, 1, 4) << 1.0f,
        0.0f, 0.33f, 0.0f;
    CameraFrameSupplement::state_vars.trans_matrix.block(1, 0, 1, 4) << 0.0f,
        1.0f, 0.0f, 0.33f;
    ADEBUG << "state trans matrix in CameraFrameSupplement is \n"
           << CameraFrameSupplement::state_vars.trans_matrix << std::endl;
    CameraFrameSupplement::state_vars.initialized_ = true;
  }

  for (auto vobj : objects) {
    std::unique_ptr<Object> obj(new Object());

    obj->id = vobj->id;
    obj->score = vobj->score;
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
    obj->anchor_point = obj->center;
    obj->state_uncertainty = vobj->state_uncertainty;

    if ((*sensor_objects)->sensor_type == SensorType::CAMERA_PSEUDO_NARROW) {
      if (FLAGS_enable_pseudo_narrow) {
        // direction needs to be changed
        obj->direction = vobj->direction.cast<double>();
        // theta needs to be changed
        obj->theta = vobj->theta;
        // obj->center = vobj->center.cast<double>();

        Eigen::Vector3d center;
        center << vobj->center.x(), vobj->center.y(), 1.0;

        Eigen::Vector3d converted_center;
        converted_center = pseudo2camera_mat_ * center;

        if (fabs(converted_center.z()) >= dEpsilon) {
          obj->center << converted_center.x() / converted_center.z(),
                         converted_center.y() / converted_center.z(),
                         vobj->center.z();
        } else {
          obj->center << vobj->center.cast<double>();
        }

        obj->anchor_point = obj->center;
        obj->state_uncertainty = vobj->state_uncertainty;

        (obj->camera_supplement).reset(new CameraSupplement());

        // convert 2D information
        ScaleBack(scale_mat_, vobj->upper_left.cast<double>(),
                  &(obj->camera_supplement->upper_left));
        ScaleBack(scale_mat_, vobj->lower_right.cast<double>(),
                  &(obj->camera_supplement->lower_right));

        obj->camera_supplement->alpha = vobj->alpha;
        // scale back 8 corner points
        obj->camera_supplement->pts8 = vobj->pts8;
        // AINFO << "vobj->pts8.size(): " << vobj->pts8.size();
        // if (vobj->pts8.size() >= 16) {
        //   for (size_t j = 0; j < 8; j++) {
        //     Eigen::Vector2d in_point;
        //     Eigen::Vector2d out_point;
        //     AINFO << "j: " << j;
        //     AINFO << "vobj->pts8[j * 2 + 0]: " << vobj->pts8[j * 2 + 0];
        //     AINFO << "vobj->pts8[j * 2 + 1]: " << vobj->pts8[j * 2 + 1];
        //     in_point.x() = static_cast<double>(vobj->pts8[j * 2 + 0]);
        //     in_point.y() = static_cast<double>(vobj->pts8[j * 2 + 1]);
        //     AINFO << "in_point: " << in_point;
        //     ScaleBack(scale_mat_, in_point, &out_point);
        //     AINFO << "out_point: " << out_point;
        //     obj->camera_supplement->pts8[j * 2 + 0]
        //       = static_cast<float>(out_point.x());
        //     obj->camera_supplement->pts8[j * 2 + 1]
        //      = static_cast<float>(out_point.y());
        //   }
        // }
      }
    } else {
      (obj->camera_supplement).reset(new CameraSupplement());
      // convert 2D information
      obj->camera_supplement->upper_left = vobj->upper_left.cast<double>();
      obj->camera_supplement->lower_right = vobj->lower_right.cast<double>();

      obj->camera_supplement->alpha = vobj->alpha;
      obj->camera_supplement->pts8 = vobj->pts8;
    }
    ((*sensor_objects)->objects).emplace_back(obj.release());
  }
}

void CameraProcessSubnode::PublishDataAndEvent(
    const double timestamp, const SharedDataPtr<SensorObjects> &sensor_objects,
    const SharedDataPtr<CameraItem> &camera_item) {
  const CommonSharedDataKey key(timestamp, device_id_);
  cam_obj_data_->Add(key, sensor_objects);
  cam_shared_data_->Add(key, camera_item);

  for (const EventMeta& event_meta : pub_meta_events_) {
    Event event;
    event.event_id = event_meta.event_id;
    event.timestamp = timestamp;
    event.reserve = device_id_;
    event_manager_->Publish(event);
  }
}

void CameraProcessSubnode::PublishDataAndEventPsuedoNarrow(
    const double timestamp, const SharedDataPtr<SensorObjects> &sensor_objects,
    const SharedDataPtr<CameraItem> &camera_item) {
  std::string device_id = device_id_ + "_pseudo_narrow";
  CommonSharedDataKey key(timestamp, device_id);
  cam_obj_data_pnarrow_->Add(key, sensor_objects);
  cam_shared_data_pnarrow_->Add(key, camera_item);

  for (size_t idx = 0; idx < pub_meta_events_.size(); ++idx) {
    const EventMeta &event_meta = pub_meta_events_[idx];
    Event event;
    event.event_id = event_meta.event_id;
    event.timestamp = timestamp;
    event.reserve = device_id;
    event_manager_->Publish(event);
  }
}

void CameraProcessSubnode::PublishPerceptionPbObj(
    const SharedDataPtr<SensorObjects> &sensor_objects) {
  PerceptionObstacles obstacles;

  // Header
  AdapterManager::FillPerceptionObstaclesHeader("perception_obstacle",
                                                &obstacles);
  common::Header *header = obstacles.mutable_header();
  header->set_lidar_timestamp(0);
  header->set_camera_timestamp(timestamp_ns_);
  header->set_radar_timestamp(0);
  obstacles.set_error_code(sensor_objects->error_code);

  // Serialize each Object
  for (const auto &obj : sensor_objects->objects) {
    PerceptionObstacle *obstacle = obstacles.add_perception_obstacle();
    obj->Serialize(obstacle);
  }

  // Relative speed of objects + latest ego car speed in X
  for (auto obstacle : obstacles.perception_obstacle()) {
    obstacle.mutable_velocity()->set_x(
        obstacle.velocity().x() + chassis_.speed_mps());
  }

  AdapterManager::PublishPerceptionObstacles(obstacles);
  ADEBUG << "PublishPerceptionObstacles: " << obstacles.ShortDebugString();
}

void CameraProcessSubnode::PublishPerceptionPbLnMsk(
    const cv::Mat &mask, const sensor_msgs::Image &message) {
  sensor_msgs::Image lane_mask_msg;
  lane_mask_msg.header = message.header;
  lane_mask_msg.header.frame_id = "lane_mask";
  MatToMessage(mask, &lane_mask_msg);

  AdapterManager::PublishPerceptionLaneMask(lane_mask_msg);
  ADEBUG << "PublishPerceptionLaneMask";
}

}  // namespace perception
}  // namespace apollo
