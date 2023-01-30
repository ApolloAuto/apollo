/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/camera/lib/obstacle/detector/bev_detection/bev_obstacle_detector.h"

#include <functional>
#include <utility>

#include <opencv2/opencv.hpp>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/camera/common/timer.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/inference/inference_factory.h"

namespace apollo {
namespace perception {
namespace camera {

bool BEVObstacleDetector::Init(const ObstacleDetectorInitOptions &options) {
  return true;
}

bool BEVObstacleDetector::Init(const StageConfig &stage_config) {
  if (!Initialize(stage_config)) {
    return false;
  }
  ACHECK(stage_config.has_camera_detector_config());
  LoadExtrinsics(stage_config.camera_detector_config().lidar_extrinsics_file(),
                 &lidar2imu_matrix_rt_);

  std::string model_type = "PaddleNet";

  inference_.reset(inference::CreateInferenceByName(
      model_type, FLAGS_bev_model_file, FLAGS_bev_params_file,
      output_blob_names_, input_blob_names_));

  std::map<std::string, std::vector<int>> shape_map;
  shape_map.emplace(std::pair<std::string, std::vector<int>>(
      input_blob_names_.at(0), images_shape_));
  shape_map.emplace(std::pair<std::string, std::vector<int>>(
      input_blob_names_.at(1), k_shape_));
  shape_map.emplace(std::pair<std::string, std::vector<int>>(
      output_blob_names_.at(0), output_bbox_shape_));
  shape_map.emplace(std::pair<std::string, std::vector<int>>(
      output_blob_names_.at(1), output_score_shape_));
  shape_map.emplace(std::pair<std::string, std::vector<int>>(
      output_blob_names_.at(2), output_label_shape_));

  if (!inference_->Init(shape_map)) {
    return false;
  }
  return true;
}

bool BEVObstacleDetector::Detect(const ObstacleDetectorOptions &options,
                                 CameraFrame *frame) {
  return true;
}

bool BEVObstacleDetector::Process(DataFrame *data_frame) {
  auto input_img_blob = inference_->get_blob(input_blob_names_.at(0));
  auto input_img2lidar_blob = inference_->get_blob(input_blob_names_.at(1));
  auto output_bbox_blob = inference_->get_blob(output_blob_names_.at(0));
  auto output_score_blob = inference_->get_blob(output_blob_names_.at(1));
  auto output_label_blob = inference_->get_blob(output_blob_names_.at(2));

  float *img_data_ptr = input_img_blob->mutable_cpu_data();
  float *img2lidar_data_ptr = input_img2lidar_blob->mutable_cpu_data();

  DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::RGB;
  std::vector<float> images_data;
  std::vector<float> k_data;
  Timer timer;
  float scale = 1.0f;
  int camera_cnt = 6;
  for (int i = 0; i < camera_cnt; ++i) {
    const auto camera_frame_temp = (data_frame + i)->camera_frame;

    std::shared_ptr<base::Image8U> image_temp = nullptr;
    image_temp.reset(
        new base::Image8U(image_height_, image_width_, base::Color::RGB));
    camera_frame_temp->data_provider->GetImage(image_options, image_temp.get());

    cv::Mat image_mat(image_height_, image_width_, CV_8UC3,
                      cv::Scalar(0, 0, 0));
    memcpy(image_mat.data, image_temp->cpu_data(),
           image_temp->total() * sizeof(uint8_t));

    cv::Mat mat_resized_tmp;
    Resize(image_mat, image_height_resized_, image_width_resized_,
           &mat_resized_tmp);
    cv::Mat mat_crop_tmp =
        mat_resized_tmp(cv::Range(130, 450), cv::Range(0, 800));
    Normalize(mean_, std_, scale, &mat_crop_tmp);
    std::vector<float> image_data(1 * 3 * img_height_crop_ * img_width_crop_,
                                  0.0f);
    Mat2Vec(mat_crop_tmp, image_data.data());

    images_data.insert(images_data.end(), image_data.begin(), image_data.end());
  }

  memcpy(img_data_ptr, images_data.data(), images_data.size() * sizeof(float));
  memcpy(img2lidar_data_ptr, k_data_.data(), k_data_.size() * sizeof(float));

  AINFO << "Preprocess: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";

  inference_->Infer();

  AINFO << "Inference: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";

  float threshold = 0.8;

  std::vector<float> out_detections_final;
  std::vector<int64_t> out_labels_final;
  std::vector<float> out_scores_final;

  FilterScore(output_bbox_blob, output_label_blob, output_score_blob, threshold,
              &out_detections_final, &out_labels_final, &out_scores_final);

  AINFO << "images_data size: " << images_data.size();
  AINFO << "k_data size: " << k_data.size();

  if (0 != out_detections_final.size()) {
    std::vector<float> detections_apollo_frame;
    Nuscenes2Apollo(out_detections_final, &detections_apollo_frame);

    GetObjects(detections_apollo_frame, out_labels_final, out_scores_final,
               data_frame->camera_frame);
  }
  return true;
}

void BEVObstacleDetector::Resize(const cv::Mat &img, int resized_h,
                                 int resized_w, cv::Mat *resize_img) {
  cv::resize(img, *resize_img, cv::Size(resized_w, resized_h), 0, 0,
             cv::INTER_LINEAR);
}

void BEVObstacleDetector::Normalize(const std::vector<float> &mean,
                                    const std::vector<float> &std, float scale,
                                    cv::Mat *im) {
  ACHECK(std.size() == 3);
  for (const auto std_value : std) {
    ACHECK(std_value != 0.0);
  }
  ACHECK(scale != 0.0);
  (*im).convertTo(*im, CV_32FC3, scale);
  for (int h = 0; h < im->rows; h++) {
    for (int w = 0; w < im->cols; w++) {
      im->at<cv::Vec3f>(h, w)[0] =
          (im->at<cv::Vec3f>(h, w)[0] - mean[0]) / std[0];
      im->at<cv::Vec3f>(h, w)[1] =
          (im->at<cv::Vec3f>(h, w)[1] - mean[1]) / std[1];
      im->at<cv::Vec3f>(h, w)[2] =
          (im->at<cv::Vec3f>(h, w)[2] - mean[2]) / std[2];
    }
  }
}

void BEVObstacleDetector::Mat2Vec(const cv::Mat &im, float *data) {
  ACHECK(nullptr != data);
  int rh = im.rows;
  int rw = im.cols;
  int rc = im.channels();

  for (int i = 0; i < rc; ++i) {
    cv::extractChannel(im, cv::Mat(rh, rw, CV_32FC1, data + i * rh * rw), i);
  }
}

void BEVObstacleDetector::FilterScore(
    const std::shared_ptr<apollo::perception::base::Blob<float>> &box3d,
    const std::shared_ptr<apollo::perception::base::Blob<float>> &label,
    const std::shared_ptr<apollo::perception::base::Blob<float>> &scores,
    float score_threshold, std::vector<float> *box3d_filtered,
    std::vector<int64_t> *label_preds_filtered,
    std::vector<float> *scores_filtered) {
  const auto bbox_ptr = box3d->cpu_data();
  const auto label_ptr = label->cpu_data();
  const auto score_ptr = scores->cpu_data();

  for (int i = 0; i < scores->count(); ++i) {
    if (score_ptr[i] > score_threshold) {
      box3d_filtered->insert(box3d_filtered->end(),
                             bbox_ptr + num_output_box_feature_ * i,
                             bbox_ptr + num_output_box_feature_ * (i + 1));
      label_preds_filtered->insert(label_preds_filtered->end(),
                                   static_cast<int64_t>(label_ptr[i]));

      scores_filtered->push_back(score_ptr[i]);
    }
  }
}

bool BEVObstacleDetector::LoadExtrinsics(const std::string &yaml_file,
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

void BEVObstacleDetector::GetObjects(const std::vector<float> &detections,
                                     const std::vector<int64_t> &labels,
                                     const std::vector<float> &scores,
                                     camera::CameraFrame *camera_frame) {
  auto objects = &(camera_frame->detected_objects);
  int num_objects = detections.size() / num_output_box_feature_;
  objects->clear();

  for (int i = 0; i < num_objects; ++i) {
    float score = scores.at(i);
    base::ObjectPtr obj = nullptr;
    obj.reset(new base::Object);

    obj->sub_type = GetObjectSubType(labels.at(i));
    obj->type = base::kSubType2TypeMap.at(obj->sub_type);
    obj->type_probs.assign(static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE),
                           0);
    obj->sub_type_probs.assign(
        static_cast<int>(base::ObjectSubType::MAX_OBJECT_TYPE), 0);
    obj->type_probs[static_cast<int>(obj->type)] = score;
    obj->sub_type_probs[static_cast<int>(obj->sub_type)] = score;
    obj->confidence = score;

    FillBBox3d(detections.data() + i * num_output_box_feature_,
               camera_frame->camera2world_pose, camera_frame->camera_extrinsic,
               lidar2imu_matrix_rt_, obj);

    objects->push_back(obj);
  }
}

void BEVObstacleDetector::FillBBox3d(const float *bbox,
                                     const Eigen::Affine3d &cam2world_pose,
                                     const Eigen::Matrix4d &cam2imu_matrix_rt,
                                     const Eigen::Matrix4d &lidar2imu_matrix_rt,
                                     base::ObjectPtr obj) {
  obj->camera_supplement.local_center[0] = bbox[0];
  obj->camera_supplement.local_center[1] = bbox[1];
  obj->camera_supplement.local_center[2] = bbox[2];
  // size: length, width, height of bbox
  obj->size[0] = bbox[4];
  obj->size[1] = bbox[3];
  obj->size[2] = bbox[5];

  obj->camera_supplement.alpha = bbox[6];
  obj->theta = bbox[6];

  obj->direction[0] = cosf(bbox[6]);
  obj->direction[1] = sinf(bbox[6]);
  obj->direction[2] = 0;

  obj->center(0) = static_cast<double>(obj->camera_supplement.local_center[0]);
  obj->center(1) = static_cast<double>(obj->camera_supplement.local_center[1]);
  obj->center(2) = static_cast<double>(obj->camera_supplement.local_center[2]);

  Eigen::Affine3d cam2imu_affine;
  Eigen::Affine3d imu2lidar_affine;
  cam2imu_affine.matrix() = cam2imu_matrix_rt;
  imu2lidar_affine.matrix() = lidar2imu_matrix_rt;

  Eigen::AngleAxisd rotation_vector(-M_PI / 2, Eigen::Vector3d(1, 0, 0));
  Eigen::Matrix4d lidar2cam;
  lidar2cam.setIdentity();
  lidar2cam.block<3, 3>(0, 0) = rotation_vector.matrix();
  Eigen::Affine3d cam2lidar_affine;
  cam2lidar_affine.matrix() = lidar2cam;

  obj->center = cam2world_pose * cam2imu_affine.inverse() * imu2lidar_affine *
                cam2lidar_affine * obj->center;

  Eigen::Matrix3d world2imu_rotation =
      (cam2world_pose.matrix() * cam2imu_affine.matrix().inverse())
          .block<3, 3>(0, 0);
  auto heading = std::atan2(world2imu_rotation(1, 0), world2imu_rotation(0, 0));
  obj->theta += (heading + M_PI / 2);
}
/*
bbox_nuscenes to bbox_apollo: Rotate 90 degrees counterclockwise about the
z-axis
*/
// bbox: x, y, z, w, l, h, yaw, vx, vy
bool BEVObstacleDetector::Nuscenes2Apollo(
    const std::vector<float> &bbox_nuscenes, std::vector<float> *bbox_apollo) {
  ACHECK(nullptr != bbox_apollo);
  int size = bbox_nuscenes.size() / num_output_box_feature_;
  for (int i = 0; i < size; ++i) {
    float x_nuscenes = bbox_nuscenes.at(0 + i * num_output_box_feature_);
    float y_nuscenes = bbox_nuscenes.at(1 + i * num_output_box_feature_);
    float z_nuscenes = bbox_nuscenes.at(2 + i * num_output_box_feature_);

    float w = bbox_nuscenes.at(3 + i * num_output_box_feature_);
    float l = bbox_nuscenes.at(4 + i * num_output_box_feature_);
    float h = bbox_nuscenes.at(5 + i * num_output_box_feature_);

    float heading = bbox_nuscenes.at(6 + i * num_output_box_feature_);

    Eigen::Vector3f center_nuscenes(x_nuscenes, y_nuscenes, z_nuscenes);
    Eigen::AngleAxisd rotation_vector(M_PI / 2, Eigen::Vector3d(1, 0, 0));
    Eigen::Vector3f center_apollo =
        rotation_vector.matrix().cast<float>() * center_nuscenes;

    std::vector<float> bbox_apollo_temp(num_output_box_feature_);

    for (int j = 0; j < 3; ++j) {
      bbox_apollo_temp[j] = center_apollo(j);
    }

    bbox_apollo_temp[3] = w;
    bbox_apollo_temp[4] = l;
    bbox_apollo_temp[5] = h;

    heading -= M_PI / 2;
    heading = std::atan2(sinf(heading), cosf(heading));
    heading = -heading;

    bbox_apollo_temp[6] = heading;
    bbox_apollo->insert(bbox_apollo->end(), bbox_apollo_temp.begin(),
                        bbox_apollo_temp.end());
  }
  return true;
}

base::ObjectSubType BEVObstacleDetector::GetObjectSubType(const int label) {
  switch (label) {
    case 0:
      return base::ObjectSubType::CAR;
    case 1:
      return base::ObjectSubType::TRUCK;
    case 3:
      return base::ObjectSubType::BUS;
    case 6:
      return base::ObjectSubType::MOTORCYCLIST;
    case 7:
      return base::ObjectSubType::CYCLIST;
    case 8:
      return base::ObjectSubType::PEDESTRIAN;
    case 9:
      return base::ObjectSubType::TRAFFICCONE;
    default:
      return base::ObjectSubType::UNKNOWN;
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
