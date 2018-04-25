/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/obstacle/lidar/segmentation/cnnseg/cnn_segmentation.h"

#include "modules/common/util/file.h"
#include "modules/perception/lib/config_manager/config_manager.h"

namespace apollo {
namespace perception {

using apollo::common::util::GetAbsolutePath;
using std::string;
using std::vector;

bool CNNSegmentation::Init() {
  string config_file;
  string proto_file;
  string weight_file;
  if (!GetConfigs(&config_file, &proto_file, &weight_file)) {
    return false;
  }
  AINFO << "--    config_file: " << config_file;
  AINFO << "--     proto_file: " << proto_file;
  AINFO << "--    weight_file: " << weight_file;

  if (!apollo::common::util::GetProtoFromFile(config_file, &cnnseg_param_)) {
    AERROR << "Failed to load config file of CNNSegmentation.";
  }

  /// set parameters
  auto network_param = cnnseg_param_.network_param();
  auto feature_param = cnnseg_param_.feature_param();

  if (feature_param.has_point_cloud_range()) {
    range_ = static_cast<float>(feature_param.point_cloud_range());
  } else {
    range_ = 60.0;
  }
  if (feature_param.has_width()) {
    width_ = static_cast<int>(feature_param.width());
  } else {
    width_ = 640;
  }
  if (feature_param.has_height()) {
    height_ = static_cast<int>(feature_param.height());
  } else {
    height_ = 640;
  }

/// Instantiate Caffe net
#ifndef USE_CAFFE_GPU
  caffe::Caffe::set_mode(caffe::Caffe::CPU);
#else
  int gpu_id =
      cnnseg_param_.has_gpu_id() ? static_cast<int>(cnnseg_param_.gpu_id()) : 0;
  CHECK_GE(gpu_id, 0);
  caffe::Caffe::SetDevice(gpu_id);
  caffe::Caffe::set_mode(caffe::Caffe::GPU);
  caffe::Caffe::DeviceQuery();
#endif

  caffe_net_.reset(new caffe::Net<float>(proto_file, caffe::TEST));
  caffe_net_->CopyTrainedLayersFrom(weight_file);

#ifndef USE_CAFFE_GPU
  AINFO << "using Caffe CPU mode";
#else
  AINFO << "using Caffe GPU mode";
#endif

  /// set related Caffe blobs
  // center offset prediction
  string instance_pt_blob_name = network_param.has_instance_pt_blob()
                                     ? network_param.instance_pt_blob()
                                     : "instance_pt";
  instance_pt_blob_ = caffe_net_->blob_by_name(instance_pt_blob_name);
  CHECK(instance_pt_blob_ != nullptr) << "`" << instance_pt_blob_name
                                      << "` not exists!";
  // objectness prediction
  string category_pt_blob_name = network_param.has_category_pt_blob()
                                     ? network_param.category_pt_blob()
                                     : "category_score";
  category_pt_blob_ = caffe_net_->blob_by_name(category_pt_blob_name);
  CHECK(category_pt_blob_ != nullptr) << "`" << category_pt_blob_name
                                      << "` not exists!";
  // positiveness (foreground object probability) prediction
  string confidence_pt_blob_name = network_param.has_confidence_pt_blob()
                                       ? network_param.confidence_pt_blob()
                                       : "confidence_score";
  confidence_pt_blob_ = caffe_net_->blob_by_name(confidence_pt_blob_name);
  CHECK(confidence_pt_blob_ != nullptr) << "`" << confidence_pt_blob_name
                                        << "` not exists!";
  // object height prediction
  string height_pt_blob_name = network_param.has_height_pt_blob()
                                   ? network_param.height_pt_blob()
                                   : "height_pt";
  height_pt_blob_ = caffe_net_->blob_by_name(height_pt_blob_name);
  CHECK(height_pt_blob_ != nullptr) << "`" << height_pt_blob_name
                                    << "` not exists!";
  // raw feature data
  string feature_blob_name =
      network_param.has_feature_blob() ? network_param.feature_blob() : "data";
  feature_blob_ = caffe_net_->blob_by_name(feature_blob_name);
  CHECK(feature_blob_ != nullptr) << "`" << feature_blob_name
                                  << "` not exists!";
  // class prediction
  string class_pt_blob_name = network_param.has_class_pt_blob()
                                  ? network_param.class_pt_blob()
                                  : "class_score";
  class_pt_blob_ = caffe_net_->blob_by_name(class_pt_blob_name);
  CHECK(class_pt_blob_ != nullptr) << "`" << class_pt_blob_name
                                   << "` not exists!";

  cluster2d_.reset(new cnnseg::Cluster2D());
  if (!cluster2d_->Init(height_, width_, range_)) {
    AERROR << "Fail to Init cluster2d for CNNSegmentation";
  }

  feature_generator_.reset(new cnnseg::FeatureGenerator<float>());
  if (!feature_generator_->Init(feature_param, feature_blob_.get())) {
    AERROR << "Fail to Init feature generator for CNNSegmentation";
    return false;
  }

  return true;
}

bool CNNSegmentation::Segment(const pcl_util::PointCloudPtr& pc_ptr,
                              const pcl_util::PointIndices& valid_indices,
                              const SegmentationOptions& options,
                              vector<std::shared_ptr<Object>>* objects) {
  objects->clear();
  int num_pts = static_cast<int>(pc_ptr->points.size());
  if (num_pts == 0) {
    AINFO << "None of input points, return directly.";
    return true;
  }

  use_full_cloud_ =
      (cnnseg_param_.has_use_full_cloud() ? cnnseg_param_.use_full_cloud()
                                          : false) &&
      (options.origin_cloud != nullptr);
  PERF_BLOCK_START();

  // generate raw features
  if (use_full_cloud_) {
    feature_generator_->Generate(options.origin_cloud);
  } else {
    feature_generator_->Generate(pc_ptr);
  }
  PERF_BLOCK_END("[CNNSeg] feature generation");

// network forward process
#ifdef USE_CAFFE_GPU
  caffe::Caffe::set_mode(caffe::Caffe::GPU);
#endif
  caffe_net_->Forward();
  PERF_BLOCK_END("[CNNSeg] CNN forward");

  // clutser points and construct segments/objects
  float objectness_thresh = cnnseg_param_.has_objectness_thresh()
                                ? cnnseg_param_.objectness_thresh()
                                : 0.5;
  bool use_all_grids_for_clustering =
      cnnseg_param_.has_use_all_grids_for_clustering()
          ? cnnseg_param_.use_all_grids_for_clustering()
          : false;
  cluster2d_->Cluster(*category_pt_blob_, *instance_pt_blob_, pc_ptr,
                      valid_indices, objectness_thresh,
                      use_all_grids_for_clustering);
  PERF_BLOCK_END("[CNNSeg] clustering");

  cluster2d_->Filter(*confidence_pt_blob_, *height_pt_blob_);

  cluster2d_->Classify(*class_pt_blob_);

  float confidence_thresh = cnnseg_param_.has_confidence_thresh()
                                ? cnnseg_param_.confidence_thresh()
                                : 0.1;
  float height_thresh =
      cnnseg_param_.has_height_thresh() ? cnnseg_param_.height_thresh() : 0.5;
  int min_pts_num = cnnseg_param_.has_min_pts_num()
                        ? static_cast<int>(cnnseg_param_.min_pts_num())
                        : 3;
  cluster2d_->GetObjects(confidence_thresh, height_thresh, min_pts_num,
                         objects);
  PERF_BLOCK_END("[CNNSeg] post-processing");

  return true;
}

bool CNNSegmentation::GetConfigs(string* config_file, string* proto_file,
                                 string* weight_file) {
  ConfigManager* config_manager = ConfigManager::instance();

  const ModelConfig* model_config =
      config_manager->GetModelConfig("CNNSegmentation");
  if (model_config == nullptr) {
    AERROR << "Failed to get model config for CNNSegmentation";
    return false;
  }
  const string& work_root = config_manager->WorkRoot();

  if (!model_config->GetValue("config_file", config_file)) {
    AERROR << "Failed to get value of config_file.";
    return false;
  }
  config_file->assign(GetAbsolutePath(work_root, *config_file));

  if (!model_config->GetValue("proto_file", proto_file)) {
    AERROR << "Failed to get value of proto_file.";
    return false;
  }
  proto_file->assign(GetAbsolutePath(work_root, *proto_file));

  if (!model_config->GetValue("weight_file", weight_file)) {
    AERROR << "Failed to get value of weight_file.";
    return false;
  }
  weight_file->assign(GetAbsolutePath(work_root, *weight_file));

  return true;
}

}  // namespace perception
}  // namespace apollo
