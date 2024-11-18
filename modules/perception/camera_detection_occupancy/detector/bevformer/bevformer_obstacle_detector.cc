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
#include "modules/perception/camera_detection_occupancy/detector/bevformer/bevformer_obstacle_detector.h"

#include <unistd.h>

#include <limits>
#include <map>
#include <memory>

#include <cuda_runtime.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing.h>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/profiler/profiler.h"
#include "modules/common/math/quaternion.h"
#include "modules/perception/camera_detection_occupancy/detector/bevformer/preprocess.h"
#include "modules/perception/camera_detection_occupancy/utils/utils.h"
#include "modules/perception/common/base/point_cloud_util.h"
#include "modules/perception/common/camera/common/util.h"
#include "modules/perception/common/inference/utils/resize.h"
#include "modules/perception/common/lidar/common/lidar_point_label.h"
#include "modules/perception/common/lidar/common/pcl_util.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace camera {
using apollo::perception::base::Blob;
using apollo::perception::lidar::PointSemanticLabel;
using apollo::perception::lidar::TransformToPCLXYZI;
using base::PointF;

const std::map<std::string, base::ObjectSubType> kNuScenesName2SubTypeMap = {
    {"car", base::ObjectSubType::CAR},
    {"truck", base::ObjectSubType::TRUCK},
    {"construction_vehicle", base::ObjectSubType::CAR},
    {"bus", base::ObjectSubType::BUS},
    {"trailer", base::ObjectSubType::CAR},
    {"barrier", base::ObjectSubType::UNKNOWN_UNMOVABLE},
    {"motorcycle", base::ObjectSubType::MOTORCYCLIST},
    {"bicycle", base::ObjectSubType::CYCLIST},
    {"pedestrian", base::ObjectSubType::PEDESTRIAN},
    {"traffic_cone", base::ObjectSubType::TRAFFICCONE},
    {"MAX_OBJECT_TYPE", base::ObjectSubType::MAX_OBJECT_TYPE},
};

const std::map<int, std::string> kIndex2NuScenesName = {
    {0, "car"},          {1, "truck"},   {2, "construction_vehicle"},
    {3, "bus"},          {4, "trailer"}, {5, "barrier"},
    {6, "motorcycle"},   {7, "bicycle"}, {8, "pedestrian"},
    {9, "traffic_cone"},
};

const std::map<std::string, base::ObjectSubType> kApolloName2SubTypeMap = {
    {"smallMot", base::ObjectSubType::CAR},
    {"bigMot", base::ObjectSubType::CAR},
    {"nonMot", base::ObjectSubType::CYCLIST},
    {"pedestrian", base::ObjectSubType::PEDESTRIAN},
    {"TrainedOthers", base::ObjectSubType::TRAFFICCONE},
    {"MAX_OBJECT_TYPE", base::ObjectSubType::MAX_OBJECT_TYPE},
};

const std::map<int, std::string> kIndex2ApolloName = {{0, "smallMot"},
                                                      {1, "bigMot"},
                                                      {2, "nonMot"},
                                                      {3, "pedestrian"},
                                                      {4, "TrainedOthers"}};

void BEVFORMERObstacleDetector::InitImageSize(
    const bevformer::ModelParam &model_param) {
  auto resize = model_param.resize();
  if (resize.width() == 0 && resize.height() == 0) {
    width_ = options_.image_width * resize.fx();
    height_ = options_.image_height * resize.fy();
  } else {
    width_ = resize.width();
    height_ = resize.height();
  }

  AINFO << "height=" << height_ << ", "
        << "width=" << width_;
}

bool BEVFORMERObstacleDetector::InitTypes(
    const bevformer::ModelParam &model_param) {
  for (const auto &class_name : model_param.info().class_names()) {
    if (kNuScenesName2SubTypeMap.find(class_name) !=
        kNuScenesName2SubTypeMap.end()) {
      types_.push_back(kNuScenesName2SubTypeMap.at(class_name));
    } else {
      AERROR << "Unsupported subtype type!" << class_name;
      return false;
    }
  }
  return true;
}

bool BEVFORMERObstacleDetector::Init(
    const ObstacleDetectorInitOptions &options) {
  options_ = options;
  gpu_id_ = options.gpu_id;
  BASE_GPU_CHECK(cudaSetDevice(gpu_id_));
  BASE_GPU_CHECK(cudaStreamCreate(&stream_));

  std::string config_file =
      GetConfigFile(options_.config_path, options_.config_file);
  std::cout << " config file is " << config_file << std::endl;
  if (!cyber::common::GetProtoFromFile(config_file, &model_param_)) {
    AERROR << "Read model param failed!";
    return false;
  }

  InitTypes(model_param_);
  InitImageSize(model_param_);

  common::Normalize normalize = model_param_.normalize();
  mean_bgr_ = {normalize.mean()[0], normalize.mean()[1], normalize.mean()[2]};
  std_bgr_ = {normalize.std()[0], normalize.std()[1], normalize.std()[2]};

  const auto &model_info = model_param_.info();
  std::string model_path = GetModelPath(model_info.name());

  if (!InitNetwork(model_param_.info(), model_path)) {
    AERROR << "Init network failed!";
    return false;
  }
  std::cout << "bevformer model init success from "
            << model_info.proto_file().file() << std::endl;

  auto model_outputs = model_param_.info().outputs();
  num_decoder_layer_ = model_outputs[1].shape()[0];
  num_queries_ = model_outputs[1].shape()[2];
  num_classes_det_ = model_outputs[1].shape()[3];
  code_size_ = model_outputs[2].shape()[3];
  class_blob_start_index_ =
      (num_decoder_layer_ - 1) * num_queries_ * num_classes_det_;
  class_blob_end_index_ =
      (num_decoder_layer_)*num_queries_ * num_classes_det_;
  box_blob_start_index_ =
      (num_decoder_layer_ - 1) * num_queries_ * code_size_;
  AINFO << "number of decoder layer:" << num_decoder_layer_
        << " number of query: " << num_queries_
        << " number of classes: " << num_classes_det_
        << " code size: " << code_size_
        << " class blob start index: " << class_blob_start_index_
        << " class blob end index: " << class_blob_end_index_
        << " box blob start index: " << box_blob_start_index_;

  num_voxels_ = model_outputs[occ_blob_index_].shape()[1];
  num_classes_occ_ = model_outputs[occ_blob_index_].shape()[2];
  AINFO << " number of occ voxels " << num_voxels_
        << " number of occ classes " << num_classes_occ_;
  occ_xmin_ = model_param_.occ_xmin();
  occ_xmax_ = model_param_.occ_xmax();
  occ_ymin_ = model_param_.occ_ymin();
  occ_ymax_ = model_param_.occ_ymax();
  occ_zmin_ = model_param_.occ_zmin();
  occ_zmax_ = model_param_.occ_zmax();
  voxel_size_ = model_param_.voxel_size();
  occ_x_grid_ = (occ_xmax_ - occ_xmin_) / voxel_size_;
  occ_y_grid_ = (occ_ymax_ - occ_ymin_) / voxel_size_;
  occ_z_grid_ = (occ_zmax_ - occ_zmin_) / voxel_size_;


  auto model_inputs = model_param_.info().inputs();
  auto prev_bev_shape = model_inputs[1].shape();
  for (auto size : prev_bev_shape) {
    prev_bev_size_ *= size;
  }
  use_prev_bev_ = new float(0.0);

  float no_pad_image_width = model_param_.no_pad_image_width();
  float no_pad_image_height = model_param_.no_pad_image_height();
  CHECK(no_pad_image_width <= model_inputs[0].shape()[3]);
  CHECK(no_pad_image_height <= model_inputs[0].shape()[4]);
  no_pad_image_shape_[0] = no_pad_image_width;
  no_pad_image_shape_[1] = no_pad_image_height;
  occ_threshold_ = model_param_.occ_threshold();

  trans_wrapper_.reset(new onboard::TransformWrapper());
  // tf_camera_frame_id
  trans_wrapper_->Init("CAM_FRONT");
  return true;
}

void BEVFORMERObstacleDetector::Mat2Vec(const cv::Mat &im, float *data) {
  ACHECK(nullptr != data);
  int rh = im.rows;
  int rw = im.cols;
  int rc = im.channels();

  for (int i = 0; i < rc; ++i) {
    cv::extractChannel(im, cv::Mat(rh, rw, CV_32FC1, data + i * rh * rw), i);
  }
}

bool BEVFORMERObstacleDetector::ImagePreprocessGPU(
    const CameraFrame *frame, base::BlobPtr<float> input_img_blob) {
  ACHECK(frame != nullptr);
  ACHECK(input_img_blob != nullptr);

  DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::RGB;
  base::Image8U image;

  float scale = model_param_.img_scale();

  std::vector<int> img_blob_shape(input_img_blob->shape().begin(),
                                  input_img_blob->shape().end());

  std::vector<int> value(1);
  auto resized_image = std::make_shared<Blob<float>>(value);
  resized_image->Reshape({img_blob_shape[0], img_blob_shape[2],
                          img_blob_shape[3], img_blob_shape[4]});

  PERF_BLOCK("stage bev process image")
  for (int i = 0; i < img_blob_shape[1]; ++i) {
    PERF_BLOCK("stage bev readimage")
    frame->data_provider[i]->GetImage(image_options, &image);
    PERF_BLOCK_END

    int left_pad = 0;
    int right_pad = 0;
    int top_pad = 0;
    int bottom_pad = 0;

    int roi_width = image.cols();
    int roi_height = image.rows();

    // int bg_rows = 0;
    // if (std::ceil(roi_height * scale / 32) * 32 < roi_height * scale) {
    //   bg_rows = (std::ceil(roi_height * scale / 32) + 1) * 32;
    // } else {
    //   bg_rows = std::ceil(roi_height * scale / 32) * 32;
    // }

    // // do down padding on image
    // if (roi_width > roi_height) {
    //   top_pad = 0;
    //   bottom_pad = (bg_rows - image.rows() * scale) / scale;
    // }

    // int dst_height = roi_height + bottom_pad + top_pad;  // 960
    // int dst_width = roi_width + left_pad + right_pad;    // 1600

    // top_pad = 0;
    // bottom_pad = 960 - height_;

    bottom_pad = img_blob_shape[3] / scale - roi_height;

    int dst_height = roi_height + bottom_pad + top_pad;
    int dst_width = roi_width + left_pad + right_pad;
    // AERROR << " dst_height: " << dst_height
    //       << " dst_width: " << dst_width
    //       << " bottom_pad: " << bottom_pad;

    CHECK(dst_height * scale == img_blob_shape[3]);
    CHECK(dst_width * scale == img_blob_shape[4]);

    base::Image8U padding_image_container_ =
        base::Image8U(dst_height, dst_width, base::Color::RGB);

    base::Image8U tmp_image =
        padding_image_container_(base::RectI(0, 0, dst_width, dst_height));
    PERF_BLOCK("stage bev padding")
    inference::ImageZeroPadding(image, &tmp_image, roi_width, left_pad,
                                right_pad, top_pad, bottom_pad, 0, stream_,
                                true);
    PERF_BLOCK_END

    PERF_BLOCK("stage bev resize")
    inference::ResizeGPU(tmp_image, resized_image,
                         frame->data_provider[i]->src_width(), 0, mean_bgr_[0],
                         mean_bgr_[1], mean_bgr_[2], false, std_bgr_[0],
                         std_bgr_[1], std_bgr_[2]);
    PERF_BLOCK_END

    PERF_BLOCK("stage bev memcpy")

    cudaMemcpy(input_img_blob->mutable_gpu_data() + i * img_blob_shape[2] *
                                                        img_blob_shape[3] *
                                                        img_blob_shape[4],
               resized_image->gpu_data(),
               img_blob_shape[2] * img_blob_shape[3] * img_blob_shape[4] *
                   sizeof(float),
               cudaMemcpyHostToDevice);
    PERF_BLOCK_END
  }
  PERF_BLOCK_END
  return true;
}

bool BEVFORMERObstacleDetector::ImagePreprocess(
    const CameraFrame *frame, base::BlobPtr<float> input_img_blob) {
  ACHECK(frame != nullptr);
  ACHECK(input_img_blob != nullptr);

  DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::RGB;

  float scale = 1.0;
  int camera_num = 6;
  base::Image8U image;
  std::vector<float> images_data;

  input_img_blob->Reshape({1, camera_num, 3, 480, 800});
  // Attention: img_data_ptr will change after reshape
  float *img_data_ptr = input_img_blob->mutable_cpu_data();

  PERF_BLOCK("stage bev preprocess")
  for (int i = 0; i < camera_num; ++i) {
    PERF_BLOCK("stage bev readimage")
    frame->data_provider[i]->GetImage(image_options, &image);
    cv::Mat img(image.rows(), image.cols(), CV_8UC3, cv::Scalar(0, 0, 0));
    memcpy(img.data, image.cpu_data(), image.total() * sizeof(uint8_t));
    PERF_BLOCK_END

    // step2 : resize
    PERF_BLOCK("stage bev resize")
    cv::resize(img, img, cv::Size(width_, height_));
    PERF_BLOCK_END

    // step3 : padding
    PERF_BLOCK("stage bev padding")
    int bg_rows = 0;
    if (std::ceil(img.rows / 32) * 32 < img.rows) {
      bg_rows = (std::ceil(img.rows / 32) + 1) * 32;
    } else {
      bg_rows = std::ceil(img.rows / 32) * 32;
    }
    cv::Mat out(bg_rows, img.cols, CV_8UC3, {0, 0, 0});

    img.copyTo(out(cv::Rect(0, 0, img.cols, img.rows)));
    PERF_BLOCK_END

    PERF_BLOCK("stage bev normalize")
    common::Normalize normalize = model_param_.normalize();
    std::vector<float> mean{normalize.mean().begin(), normalize.mean().end()};
    std::vector<float> std{normalize.std().begin(), normalize.std().end()};
    std::reverse(mean.begin(), mean.end());
    std::reverse(std.begin(), std.end());
    BevFormerNormalize(mean, std, scale, &out);
    PERF_BLOCK_END

    std::vector<float> image_data(1 * 3 * out.cols * out.rows, 0.0f);
    Mat2Vec(out, image_data.data());

    images_data.insert(images_data.end(), image_data.begin(), image_data.end());
  }
  memcpy(img_data_ptr, images_data.data(), images_data.size() * sizeof(float));
  PERF_BLOCK_END
  return true;
}

bool BEVFORMERObstacleDetector::ImageExtrinsicPreprocess(
    const CameraFrame *frame, base::BlobPtr<float> input_blob_lidar2img) {
  if (resize_flag_ == false) {
    resized_lidar2img_.resize(96);
    for (int i = 0; i < 96; i++) {
      if (i % 16 < 8) {
        resized_lidar2img_.at(i) =
            frame->k_lidar2img.at(i) * model_param_.img_scale();
      } else {
        resized_lidar2img_.at(i) = frame->k_lidar2img.at(i);
      }
    }
    resize_flag_ = true;
  }
  memcpy(input_blob_lidar2img->mutable_cpu_data(), resized_lidar2img_.data(),
         resized_lidar2img_.size() * sizeof(float));
  return true;
}

void BEVFORMERObstacleDetector::FillCanBus(const CameraFrame *frame,
                                           base::BlobPtr<float> can_bus_blob) {
  apollo::cyber::Time query_time(frame->timestamp);
  apollo::transform::TransformStamped stamped_transform;

  try {
    stamped_transform =
        tf2_buffer_->lookupTransform("world", "localization", query_time);
    // AERROR << "find tf! " << query_time;
  } catch (tf2::TransformException &ex) {
    AERROR << ex.what();
    AERROR << "can't find tf! " << query_time;
    return;
  }

  Eigen::Quaterniond rotation(stamped_transform.transform().rotation().qw(),
                              stamped_transform.transform().rotation().qx(),
                              stamped_transform.transform().rotation().qy(),
                              stamped_transform.transform().rotation().qz());
  std::vector<double> current_can_bus;
  std::vector<double> input_can_bus;
  std::vector<float> input_can_bus_float;
  current_can_bus.push_back(stamped_transform.transform().translation().x());
  current_can_bus.push_back(stamped_transform.transform().translation().y());
  current_can_bus.push_back(stamped_transform.transform().translation().z());
  // note: instead of qw, qx, qy, qz, here use 4*qw as same as training
  current_can_bus.push_back(stamped_transform.transform().rotation().qw());
  current_can_bus.push_back(stamped_transform.transform().rotation().qw());
  current_can_bus.push_back(stamped_transform.transform().rotation().qw());
  current_can_bus.push_back(stamped_transform.transform().rotation().qw());
  for (int ii = 0; ii < 9; ii++) {
    current_can_bus.push_back(0.0);
  }
  Eigen::Vector3d vec(1.0, 0.0, 0.0);
  vec = rotation.toRotationMatrix() * vec;
  double yaw = atan2(vec(1), vec(0));
  yaw = yaw / M_PI * 180;
  if (yaw < 0) {
    yaw += 360;
  }
  current_can_bus.push_back(yaw / 180 * M_PI);
  current_can_bus.push_back(yaw);

  input_can_bus = current_can_bus;
  if (*use_prev_bev_) {
    input_can_bus[0] -= last_can_bus_[0];
    input_can_bus[1] -= last_can_bus_[1];
    input_can_bus[2] -= last_can_bus_[2];
    input_can_bus[17] -= last_can_bus_[17];
  } else {
    input_can_bus[0] = 0.0;
    input_can_bus[1] = 0.0;
    input_can_bus[2] = 0.0;
    input_can_bus[17] = 0.0;
  }

  last_can_bus_ = current_can_bus;
  for (int ii = 0; ii < input_can_bus.size(); ii++) {
    input_can_bus_float.push_back(static_cast<float>(input_can_bus.at(ii)));
  }
  // can_bus_blob:
  // 0-2: ego-car x/y/z offset (m) of current frame and last frame in world
  // coordiante 3-6: ego-car qw of current frame and last frame in world
  // coordiante 7-16: 0 16: yaw 17: (yaw/pi) * 180 diff of current frame and
  // last frame

  for (int ii = 0; ii < 3; ii++) {
    if (input_can_bus.at(ii) >= model_param_.location_dist_threshold()) {
      AERROR << "warning, ego-car location is too large!";
    }
  }

  memcpy(can_bus_blob->mutable_cpu_data(), input_can_bus_float.data(),
         sizeof(float) * input_can_bus_float.size());
}

void BEVFORMERObstacleDetector::GetObjects(
    const CameraFrame *frame, const float *outputs_scores,
    const float *outputs_coords, std::vector<base::ObjectPtr> *objects) {
  int num_objects = 0;
  ACHECK(objects != nullptr);
  objects->clear();
  float score_threshold = model_param_.score_threshold();
  for (int i = class_blob_start_index_; i < class_blob_end_index_; i++) {
    int current_layer_index = i - class_blob_start_index_;
    float score = outputs_scores[i];
    if (score >= score_threshold) {
      num_objects += 1;
      int label = floor(current_layer_index % num_classes_det_);
      int box_index = current_layer_index / num_classes_det_;
      int current_layer_box_index =
          box_blob_start_index_ + box_index * code_size_;
      float cx = outputs_coords[current_layer_box_index];
      float cy = outputs_coords[current_layer_box_index + 1];
      float cz = outputs_coords[current_layer_box_index + 4];
      float w = exp(outputs_coords[current_layer_box_index + 2]);
      float l = exp(outputs_coords[current_layer_box_index + 3]);
      float h = exp(outputs_coords[current_layer_box_index + 5]);
      float rot = atan2(outputs_coords[current_layer_box_index + 6],
                        outputs_coords[current_layer_box_index + 7]);

      base::ObjectPtr obj = nullptr;
      obj.reset(new base::Object);
      obj->sub_type =
        kNuScenesName2SubTypeMap.at(kIndex2NuScenesName.at(label));
      obj->sub_type_probs.assign(
          static_cast<int>(base::ObjectSubType::MAX_OBJECT_TYPE), 0);
      obj->sub_type_probs[static_cast<int>(obj->sub_type)] = score;
      obj->type = base::kSubType2TypeMap.at(obj->sub_type);
      obj->type_probs.assign(
          static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0);
      obj->type_probs[static_cast<int>(obj->type)] = score;
      obj->confidence = score;
      obj->center(0) = cx;
      obj->center(1) = cy;
      obj->center(2) = cz;
      obj->size[0] = l;
      obj->size[1] = w;
      obj->size[2] = h;
      obj->theta = -rot - M_PI / 2;
      if (obj->theta > M_PI) {
        obj->theta = obj->theta - 2 * M_PI;
      }
      if (obj->theta < -M_PI) {
        obj->theta = obj->theta + 2 * M_PI;
      }
      objects->push_back(obj);
      AINFO << "timestamp " << std::to_string(frame->timestamp)
            << " type: " << label << " score: " << score << " center: " << cx
            << " " << cy << " " << cz << " lwh: " << l << " " << w << " " << h
            << " rot " << rot;
    }
  }
  AINFO << " bevformer detect " << num_objects << " objects";
}

bool BEVFORMERObstacleDetector::GetOccResults(CameraFrame *frame,
                                              const float *outputs_occupancy) {
  if (frame == nullptr) {
    return false;
  }
  occ_results_.clear();
  int valid_occ_point = 0;
  for (int voxel_index = 0; voxel_index < num_voxels_; voxel_index++) {
    float max_prob = 0.0;
    int class_pred = num_classes_occ_ + 1;
    for (int class_index = 0; class_index < num_classes_occ_; class_index++) {
      float class_prob =
          outputs_occupancy[voxel_index * num_classes_occ_ + class_index];
      if (class_prob > max_prob) {
        max_prob = class_prob;
        class_pred = class_index;
      }
    }
    if (max_prob >= occ_threshold_) {
      occ_results_.push_back(std::make_pair(voxel_index, class_pred));
      valid_occ_point += 1;
    }
  }
  AINFO << "number of valid occ voxels: " << valid_occ_point;


  if (access(model_param_.occ_save_path().c_str(), 0) == -1) {
    system(("mkdir " + model_param_.occ_save_path()).c_str());
  }
  std::string out_file = model_param_.occ_save_path() + '/' +
                          std::to_string(frame->timestamp) + ".bin";
  std::ofstream out_stream;
  out_stream.open(out_file);
  for (int i = 0; i < occ_results_.size(); i++) {
    out_stream << occ_results_.at(i).first << " " << occ_results_.at(i).second
                << " ";
  }
  out_stream.close();

  return true;
}

bool BEVFORMERObstacleDetector::Detect(CameraFrame *frame) {
  if (frame == nullptr) {
    return false;
  }
  // AERROR << "current time is " << std::to_string(frame->timestamp);

  auto model_inputs = model_param_.info().inputs();
  auto input_blob_img = net_->get_blob(model_inputs[0].name());
  auto input_blob_prev_bev = net_->get_blob(model_inputs[1].name());
  auto input_blob_use_prev_bev = net_->get_blob(model_inputs[2].name());
  auto input_blob_can_bus = net_->get_blob(model_inputs[3].name());
  auto input_blob_lidar2img = net_->get_blob(model_inputs[4].name());
  auto input_blob_no_pad_image_shape = net_->get_blob(model_inputs[5].name());
  auto model_outputs = model_param_.info().outputs();

  PERF_BLOCK("stage bev process")
  memcpy(input_blob_no_pad_image_shape->mutable_cpu_data(),
         no_pad_image_shape_.data(),
         sizeof(float) * no_pad_image_shape_.size());
  ImageExtrinsicPreprocess(frame, input_blob_lidar2img);
  ImagePreprocessGPU(frame, input_blob_img);
  FillCanBus(frame, input_blob_can_bus);
  memcpy(input_blob_use_prev_bev->mutable_cpu_data(), use_prev_bev_,
         sizeof(float));
  if (*use_prev_bev_) {
    auto bev_embed = net_->get_blob(model_outputs[0].name());
    cudaMemcpy(input_blob_prev_bev->mutable_gpu_data(), bev_embed->gpu_data(),
               prev_bev_size_ * sizeof(float), cudaMemcpyHostToDevice);
  }
  PERF_BLOCK_END

  PERF_BLOCK("stage bev infer")
  net_->Infer();
  PERF_BLOCK_END

  PERF_BLOCK("stage bev get objects")
  auto outputs_classes = net_->get_blob(model_outputs[1].name());
  auto outputs_coords = net_->get_blob(model_outputs[2].name());
  const float *outputs_classes_ptr = outputs_classes->cpu_data();
  const float *outputs_coords_ptr = outputs_coords->cpu_data();
  GetObjects(frame, outputs_classes_ptr, outputs_coords_ptr,
              &frame->detected_objects);
  PERF_BLOCK_END

  PERF_BLOCK("stage bev get occ")
  if (model_param_.save_occ_result()) {
    auto outputs_occupancy =
        net_->get_blob(model_outputs[occ_blob_index_].name());
    const float *outputs_occupancy_ptr = outputs_occupancy->cpu_data();
    GetOccResults(frame, outputs_occupancy_ptr);
  }
  PERF_BLOCK_END

  Nuscenes2Apollo(&frame->detected_objects);
  *use_prev_bev_ = 1.0;
  return true;
}

bool BEVFORMERObstacleDetector::Nuscenes2Apollo(
    std::vector<base::ObjectPtr> *objects) {
  ACHECK(objects != nullptr);
  for (auto &obj : *objects) {
    // if (model_param_.info().dataset() == "nuScenes") {
    //   obj->theta += M_PI / 2;
    // }
    obj->direction[0] = cosf(obj->theta);
    obj->direction[1] = sinf(obj->theta);
    obj->direction[2] = 0;

    // if (model_param_.info().dataset() == "nuScenes") {
    //   Eigen::AngleAxisd rotation_vector(M_PI / 2, Eigen::Vector3d(0, 0, 1));
    //   obj->center = rotation_vector.matrix() * obj->center;
    // }
    obj->camera_supplement.local_center[0] = static_cast<float>(obj->center[0]);
    obj->camera_supplement.local_center[1] = static_cast<float>(obj->center[1]);
    obj->camera_supplement.local_center[2] = static_cast<float>(obj->center[2]);
  }
  return true;
}

REGISTER_OBSTACLE_DETECTOR(BEVFORMERObstacleDetector);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
