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
#include "modules/perception/lidar/lib/detector/cnn_segmentation/cnn_segmentation.h"

#include <map>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/inference/inference_factory.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/common/lidar_point_label.h"
#include "modules/perception/lidar/common/lidar_timer.h"
#include "modules/perception/lidar/lib/detector/cnn_segmentation/util.h"
#include "modules/perception/lidar/lib/detector/ncut_segmentation/ncut_segmentation.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::cyber::common::GetAbsolutePath;
using apollo::cyber::common::GetProtoFromFile;
using base::AttributePointCloud;
using base::Object;
using base::PointF;

bool CNNSegmentation::Init(const LidarDetectorInitOptions& options) {
  // get configs
  std::string param_file;
  std::string proto_file;
  std::string weight_file;
  std::string engine_file;

  if (!FLAGS_lidar_model_version.empty()) {
    sensor_name_ = FLAGS_lidar_model_version;
  } else {
    sensor_name_ = options.sensor_name;
  }

  CHECK(GetConfigs(&param_file, &proto_file, &weight_file, &engine_file));
  AINFO << "--    param_file: " << param_file;
  AINFO << "--    proto_file: " << proto_file;
  AINFO << "--    weight_file: " << weight_file;
  AINFO << "--    engine_file: " << engine_file;

  // get cnnseg params
  ACHECK(GetProtoFromFile(param_file, &cnnseg_param_))
      << "Failed to parse CNNSegParam config file." << param_file;
  ACHECK(GetProtoFromFile(engine_file, &spp_engine_config_))
      << "Failed to parse SppEngine config file." << engine_file;

  // init feature parameters
  const FeatureParam& feature_param = cnnseg_param_.feature_param();
  range_ = feature_param.point_cloud_range();
  width_ = feature_param.width();
  height_ = feature_param.height();
  min_height_ = feature_param.min_height();
  max_height_ = feature_param.max_height();

  // init inference model
  const NetworkParam& network_param = cnnseg_param_.network_param();
  std::vector<std::string> output_names;
  output_names.push_back(network_param.instance_pt_blob());
  output_names.push_back(network_param.category_pt_blob());
  output_names.push_back(network_param.confidence_pt_blob());
  output_names.push_back(network_param.height_pt_blob());
  output_names.push_back(network_param.heading_pt_blob());
  output_names.push_back(network_param.class_pt_blob());
  std::vector<std::string> input_names;
  input_names.push_back(network_param.feature_blob());
  inference_.reset(inference::CreateInferenceByName(cnnseg_param_.model_type(),
                                                    proto_file, weight_file,
                                                    output_names, input_names));
  CHECK_NOTNULL(inference_.get());

  gpu_id_ = cnnseg_param_.has_gpu_id() ? cnnseg_param_.gpu_id() : -1;
  BASE_GPU_CHECK(cudaSetDevice(gpu_id_));
  inference_->set_gpu_id(gpu_id_);  // inference sets CPU mode when -1

  std::map<std::string, std::vector<int>> input_shapes;
  auto& input_shape = input_shapes[network_param.feature_blob()];
  input_shape = {1, 8, height_, width_};
  if (!feature_param.use_intensity_feature()) {
    input_shape[1] -= 2;
  }
  if (!feature_param.use_constant_feature()) {
    input_shape[1] -= 2;
  }
  ACHECK(inference_->Init(input_shapes)) << "Failed to init inference.";

  // init blobs
  instance_pt_blob_ = inference_->get_blob(network_param.instance_pt_blob());
  CHECK_NOTNULL(instance_pt_blob_.get());
  category_pt_blob_ = inference_->get_blob(network_param.category_pt_blob());
  CHECK_NOTNULL(category_pt_blob_.get());
  confidence_pt_blob_ =
      inference_->get_blob(network_param.confidence_pt_blob());
  CHECK_NOTNULL(confidence_pt_blob_.get());
  height_pt_blob_ = inference_->get_blob(network_param.height_pt_blob());
  CHECK_NOTNULL(height_pt_blob_.get());
  feature_blob_ = inference_->get_blob(network_param.feature_blob());
  CHECK_NOTNULL(feature_blob_.get());
  if (cnnseg_param_.do_classification()) {
    classify_pt_blob_ = inference_->get_blob(network_param.class_pt_blob());
    CHECK_NOTNULL(classify_pt_blob_.get());
  }
  if (cnnseg_param_.do_heading()) {
    heading_pt_blob_ = inference_->get_blob(network_param.heading_pt_blob());
    CHECK_NOTNULL(heading_pt_blob_.get());
  }

  // init feature generator
  feature_generator_.reset(new FeatureGenerator);
  ACHECK(feature_generator_->Init(feature_param, feature_blob_.get()))
      << "Failed to init feature generator.";

  point2grid_.reserve(kDefaultPointCloudSize);

  // init cluster and background segmentation methods
  ACHECK(InitClusterAndBackgroundSegmentation());

  // secondary segmentor
  /*if (cnnseg_param_.fill_recall_with_ncut()) {
     secondary_segmentor.reset(new NCutSegmentation());
     if(!secondary_segmentor->Init(SegmentationInitOptions())) {
         AERROR<<"initialized secondary segmentor fails";
         return false;
     }
  }*/
  return true;
}

bool CNNSegmentation::Init(const StageConfig& stage_config) {
  if (!Initialize(stage_config)) {
    return false;
  }

  cnnseg_config_ = stage_config.cnnseg_config();
  AINFO << "CNNSegmentation: " << cnnseg_config_.DebugString();

  if (!FLAGS_lidar_model_version.empty()) {
    sensor_name_ = FLAGS_lidar_model_version;
  } else {
    sensor_name_ = cnnseg_config_.sensor_name();
  }

  // get cnnseg params
  ACHECK(GetProtoFromFile(cnnseg_config_.param_file(), &cnnseg_param_))
      << "Failed to parse CNNSegParam config file."
      << cnnseg_config_.param_file();
  ACHECK(GetProtoFromFile(cnnseg_config_.engine_file(), &spp_engine_config_))
      << "Failed to parse SppEngine config file."
      << cnnseg_config_.engine_file();

  // init feature parameters
  const FeatureParam& feature_param = cnnseg_param_.feature_param();
  range_ = feature_param.point_cloud_range();
  width_ = feature_param.width();
  height_ = feature_param.height();
  min_height_ = feature_param.min_height();
  max_height_ = feature_param.max_height();

  // init inference model
  const NetworkParam& network_param = cnnseg_param_.network_param();
  std::vector<std::string> output_names;
  output_names.push_back(network_param.instance_pt_blob());
  output_names.push_back(network_param.category_pt_blob());
  output_names.push_back(network_param.confidence_pt_blob());
  output_names.push_back(network_param.height_pt_blob());
  output_names.push_back(network_param.heading_pt_blob());
  output_names.push_back(network_param.class_pt_blob());
  std::vector<std::string> input_names;
  input_names.push_back(network_param.feature_blob());
  inference_.reset(
      inference::CreateInferenceByName(cnnseg_param_.model_type(),
                                       cnnseg_config_.proto_file(),
                                       cnnseg_config_.weight_file(),
                                       output_names,
                                       input_names));
  CHECK_NOTNULL(inference_.get());

  gpu_id_ = cnnseg_param_.has_gpu_id() ? cnnseg_param_.gpu_id() : -1;
  BASE_GPU_CHECK(cudaSetDevice(gpu_id_));
  inference_->set_gpu_id(gpu_id_);  // inference sets CPU mode when -1

  std::map<std::string, std::vector<int>> input_shapes;
  auto& input_shape = input_shapes[network_param.feature_blob()];
  input_shape = {1, 8, height_, width_};
  if (!feature_param.use_intensity_feature()) {
    input_shape[1] -= 2;
  }
  if (!feature_param.use_constant_feature()) {
    input_shape[1] -= 2;
  }
  ACHECK(inference_->Init(input_shapes)) << "Failed to init inference.";

  // init blobs
  instance_pt_blob_ = inference_->get_blob(network_param.instance_pt_blob());
  CHECK_NOTNULL(instance_pt_blob_.get());
  category_pt_blob_ = inference_->get_blob(network_param.category_pt_blob());
  CHECK_NOTNULL(category_pt_blob_.get());
  confidence_pt_blob_ =
      inference_->get_blob(network_param.confidence_pt_blob());
  CHECK_NOTNULL(confidence_pt_blob_.get());
  height_pt_blob_ = inference_->get_blob(network_param.height_pt_blob());
  CHECK_NOTNULL(height_pt_blob_.get());
  feature_blob_ = inference_->get_blob(network_param.feature_blob());
  CHECK_NOTNULL(feature_blob_.get());
  if (cnnseg_param_.do_classification()) {
    classify_pt_blob_ = inference_->get_blob(network_param.class_pt_blob());
    CHECK_NOTNULL(classify_pt_blob_.get());
  }
  if (cnnseg_param_.do_heading()) {
    heading_pt_blob_ = inference_->get_blob(network_param.heading_pt_blob());
    CHECK_NOTNULL(heading_pt_blob_.get());
  }

  // init feature generator
  feature_generator_.reset(new FeatureGenerator);
  ACHECK(feature_generator_->Init(feature_param, feature_blob_.get()))
      << "Failed to init feature generator.";

  point2grid_.reserve(kDefaultPointCloudSize);

  // init cluster and background segmentation methods
  ACHECK(InitClusterAndBackgroundSegmentation());

  return true;
}

bool CNNSegmentation::Process(DataFrame* data_frame) {
  if (data_frame == nullptr)
    return false;

  LidarFrame* lidar_frame = data_frame->lidar_frame;
  if (lidar_frame == nullptr)
    return false;

  LidarDetectorOptions options;
  bool res = Detect(options, lidar_frame);
  return res;
}

bool CNNSegmentation::InitClusterAndBackgroundSegmentation() {
  // init ground detector
  ground_detector_ = BaseGroundDetectorRegisterer::GetInstanceByName(
      cnnseg_param_.ground_detector());
  CHECK_NOTNULL(ground_detector_);
  GroundDetectorInitOptions ground_detector_init_options;
  ACHECK(ground_detector_->Init(ground_detector_init_options))
      << "Failed to init ground detection.";

  // init roi filter
  roi_filter_ =
      BaseROIFilterRegisterer::GetInstanceByName(cnnseg_param_.roi_filter());
  CHECK_NOTNULL(roi_filter_);
  ROIFilterInitOptions roi_filter_init_options;
  ACHECK(roi_filter_->Init(roi_filter_init_options))
      << "Failed to init roi filter.";

  // init spp engine
  SppParams params;
  params.height_gap = spp_engine_config_.height_gap();
  params.confidence_range = cnnseg_param_.confidence_range();

  // init spp data
  auto& spp_data = spp_engine_.GetSppData();
  spp_data.instance_pt_blob = instance_pt_blob_.get();
  spp_data.category_pt_blob = category_pt_blob_.get();
  spp_data.confidence_pt_blob = confidence_pt_blob_.get();

  spp_data.objectness_threshold = cnnseg_param_.objectness_thresh();
  spp_data.confidence_threshold = cnnseg_param_.confidence_thresh();
  spp_data.top_z_threshold = cnnseg_param_.height_thresh();
  spp_data.class_num = static_cast<size_t>(MetaType::MAX_META_TYPE);
  if (height_pt_blob_ != nullptr) {
    spp_data.height_pt_blob = height_pt_blob_.get();
  }
  if (cnnseg_param_.do_classification()) {
    spp_data.classify_pt_blob = classify_pt_blob_.get();
  }
  if (cnnseg_param_.do_heading()) {
    spp_data.heading_pt_blob = heading_pt_blob_.get();
  }
  spp_data.MakeReference(width_, height_, range_);

  // init spp engine
  spp_engine_.Init(width_, height_, range_, params, sensor_name_);

  roi_cloud_ = base::PointFCloudPool::Instance().Get();
  roi_world_cloud_ = base::PointDCloudPool::Instance().Get();

  // init thread worker
  worker_.Bind([&]() {
    Timer timer;
    ROIFilterOptions roi_filter_options;
    AINFO << "before roi filter";
    if (lidar_frame_ref_->hdmap_struct != nullptr &&
        roi_filter_->Filter(roi_filter_options, lidar_frame_ref_)) {
      roi_cloud_->CopyPointCloud(*lidar_frame_ref_->cloud,
                                 lidar_frame_ref_->roi_indices);
      roi_world_cloud_->CopyPointCloud(*lidar_frame_ref_->world_cloud,
                                       lidar_frame_ref_->roi_indices);
    } else {
      AINFO << "Fail to call roi filter, use origin cloud.";
      lidar_frame_ref_->roi_indices.indices.resize(original_cloud_->size());
      // we manually fill roi indices with all cloud point indices
      std::iota(lidar_frame_ref_->roi_indices.indices.begin(),
                lidar_frame_ref_->roi_indices.indices.end(), 0);
      // note roi cloud's memory should be kept here
      *roi_cloud_ = *original_cloud_;
      *roi_world_cloud_ = *original_world_cloud_;
    }
    lidar_frame_ref_->cloud = roi_cloud_;
    lidar_frame_ref_->world_cloud = roi_world_cloud_;

    roi_filter_time_ = timer.toc(true);
    AINFO << "after roi filter";
    GroundDetectorOptions ground_detector_options;
    ground_detector_->Detect(ground_detector_options, lidar_frame_ref_);
    if (lidar_frame_ref_->cloud != original_cloud_) {
      lidar_frame_ref_->cloud = original_cloud_;
    }
    if (lidar_frame_ref_->world_cloud != original_world_cloud_) {
      lidar_frame_ref_->world_cloud = original_world_cloud_;
    }
    ground_detector_time_ = timer.toc(true);
    AINFO << "Roi-filter time: " << roi_filter_time_
          << "\tGround-detector time: " << ground_detector_time_;
    return true;
  });
  worker_.Start();
  return true;
}

void CNNSegmentation::MapPointToGrid(
    const std::shared_ptr<AttributePointCloud<PointF>>& pc_ptr) {
  float inv_res_x = 0.5f * static_cast<float>(width_) / range_;
  // float inv_res_y = 0.5 * static_cast<float>(height_) / range_;
  point2grid_.assign(pc_ptr->size(), -1);
  int pos_x = -1;
  int pos_y = -1;
  for (size_t i = 0; i < pc_ptr->size(); ++i) {
    const auto& pt = pc_ptr->at(i);
    if (pt.z <= min_height_ || pt.z >= max_height_) {
      continue;
    }
    // the coordinates of x and y are exchanged here
    // (row <-> x, column <-> y)
    // int pos_x = F2I(pt.y, range_, inv_res_x);  // col
    // int pos_y = F2I(pt.x, range_, inv_res_y);  // row
    // 2018.6.21, switch to axis rotated projection
    GroupPc2Pixel(pt.x, pt.y, inv_res_x, range_, &pos_x, &pos_y);
    if (pos_y < 0 || pos_y >= height_ || pos_x < 0 || pos_x >= width_) {
      continue;
    }
    point2grid_[i] = pos_y * width_ + pos_x;
  }
}

bool CNNSegmentation::Detect(const LidarDetectorOptions& options,
                              LidarFrame* frame) {
  // check input
  if (frame == nullptr) {
    AERROR << "Input null frame ptr.";
    return false;
  }
  if (frame->cloud == nullptr) {
    AERROR << "Input null frame cloud.";
    return false;
  }
  if (frame->world_cloud == nullptr) {
    AERROR << "Input null frame world cloud.";
    return false;
  }
  if (frame->cloud->size() == 0) {
    AERROR << "Input none points.";
    return false;
  }
  if (frame->cloud->size() != frame->world_cloud->size()) {
    AERROR << "Cloud size and world cloud size not consistent.";
    return false;
  }
  // record input cloud and lidar frame
  original_cloud_ = frame->cloud;
  original_world_cloud_ = frame->world_cloud;
  lidar_frame_ref_ = frame;

  // check output
  frame->segmented_objects.clear();
  worker_.WakeUp();

  // note we should use origninal cloud here, frame->cloud may be exchanged
  Timer timer;
  // map 3d points to 2d image grids
  MapPointToGrid(original_cloud_);
  mapping_time_ = timer.toc(true);

  if (cudaSetDevice(gpu_id_) != cudaSuccess) {
    AERROR << "Failed to set device to " << gpu_id_;
    return false;
  }

  // generate features
  feature_generator_->Generate(original_cloud_, point2grid_);
  feature_time_ = timer.toc(true);

  // model inference
  inference_->Infer();
  infer_time_ = timer.toc(true);

  // processing clustering
  GetObjectsFromSppEngine(&frame->segmented_objects);

  AERROR << "CNNSEG: mapping: " << mapping_time_ << "\t"
        << " feature: " << feature_time_ << "\t"
        << " infer: " << infer_time_ << "\t"
        << " fg-seg: " << fg_seg_time_ << "\t"
        << " join: " << join_time_ << "\t"
        << " collect: " << collect_time_;
  return true;
}

void CNNSegmentation::GetObjectsFromSppEngine(
    std::vector<std::shared_ptr<Object>>* objects) {
  Timer timer;
  spp_engine_.GetSppData().grid_indices = point2grid_.data();
  size_t num_foreground =
      spp_engine_.ProcessForegroundSegmentation(original_cloud_);
  fg_seg_time_ = timer.toc(true);
  // should sync with worker before do background segmentation
  worker_.Join();
  join_time_ = timer.toc(true);
  // copy height from roi cloud to origin cloud,
  // note ground points include other noise points
  // filtered by ground detection post process
  AINFO << "Use origin cloud and copy height";
  for (std::size_t i = 0; i < lidar_frame_ref_->roi_indices.indices.size();
       ++i) {
    const int roi_id = lidar_frame_ref_->roi_indices.indices[i];
    original_cloud_->mutable_points_height()->at(roi_id) =
        roi_cloud_->points_height(i);
    if (roi_cloud_->mutable_points_label()->at(i) ==
        static_cast<uint8_t>(LidarPointLabel::GROUND)) {
      original_cloud_->mutable_points_label()->at(roi_id) =
          roi_cloud_->points_label().at(i);
    }
  }
  memcpy(&original_world_cloud_->mutable_points_height()->at(0),
         &original_cloud_->points_height().at(0),
         sizeof(float) * original_cloud_->size());
  memcpy(&original_world_cloud_->mutable_points_label()->at(0),
         &original_cloud_->points_label().at(0),
         sizeof(uint8_t) * original_cloud_->size());
  if (cnnseg_param_.remove_ground_points()) {
    num_foreground = spp_engine_.RemoveGroundPointsInForegroundCluster(
        original_cloud_, lidar_frame_ref_->roi_indices,
        lidar_frame_ref_->non_ground_indices);
    if (num_foreground == 0) {
      ADEBUG << "No foreground segmentation output";
    }
  }

  const auto& clusters = spp_engine_.clusters();
  objects->clear();
  base::ObjectPool::Instance().BatchGet(clusters.size(), objects);
  size_t valid = 0;

  // prepare for valid point cloud for seconary segmentor
  // after removing pts from primary segmentor, ground and non roi pts
  /*CloudMask mask;
  if (cnnseg_param_.fill_recall_with_ncut()) {
     mask.Set(original_cloud_.size(), 0);
     mask.AddIndicesOfIndices(lidar_frame_ref->roi_indices,
  lidar_frame_ref->non_ground_indices, 1);
  }*/

  for (int i = 0; i < static_cast<int>(clusters.size()); ++i) {
    if (clusters[i]->points.size() <= cnnseg_param_.min_pts_num() &&
        clusters[i]->pixels.size() < cnnseg_param_.min_pts_num()) {
      continue;
    }
    auto& cluster = clusters[i];
    auto& object = objects->at(valid);
    object->lidar_supplement.num_points_in_roi = cluster->points_in_roi;
    object->lidar_supplement.on_use = true;
    object->lidar_supplement.is_background = false;
    // ACHECK(cluster->points.size() == cluster->point_ids.size())
    //  << "cluster points size: " << cluster->points.size()
    //  << "cluster point ids size: " << cluster->point_ids.size();
    object->lidar_supplement.cloud.CopyPointCloud(*original_cloud_,
                                                  cluster->point_ids);
    object->lidar_supplement.cloud_world.CopyPointCloud(*original_world_cloud_,
                                                        cluster->point_ids);

    // for miss detection, try to fill recall with ncut
    /*if (cnnseg_param_.fill_recall_with_ncut()) {
         base::PointIndices ind;
         ind.indices = cluster->point_ids; // ? valid
         mask.RemoveIndices(ind);
    }*/

    // for (auto& id : cluster->point_ids) {
    //  original_cloud_->points_label(id)
    //    = static_cast<uint8_t>(LidarPointLabel::OBJECT);
    //}
    object->confidence = cluster->confidence;
    object->id = static_cast<int>(valid);
    if (cnnseg_param_.do_classification()) {
      object->lidar_supplement.raw_probs.push_back(std::vector<float>(
          static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
      object->lidar_supplement.raw_classification_methods.push_back(Name());
      object->lidar_supplement.raw_probs
          .back()[static_cast<int>(base::ObjectType::UNKNOWN)] =
          cluster->class_prob[static_cast<int>(MetaType::META_UNKNOWN)];
      object->lidar_supplement.raw_probs
          .back()[static_cast<int>(base::ObjectType::PEDESTRIAN)] =
          cluster->class_prob[static_cast<int>(MetaType::META_PEDESTRIAN)];
      object->lidar_supplement.raw_probs
          .back()[static_cast<int>(base::ObjectType::BICYCLE)] =
          cluster->class_prob[static_cast<int>(MetaType::META_NONMOT)];
      object->lidar_supplement.raw_probs
          .back()[static_cast<int>(base::ObjectType::VEHICLE)] =
          cluster->class_prob[static_cast<int>(MetaType::META_SMALLMOT)] +
          cluster->class_prob[static_cast<int>(MetaType::META_BIGMOT)];
      // copy to type
      object->type_probs.assign(
          object->lidar_supplement.raw_probs.back().begin(),
          object->lidar_supplement.raw_probs.back().end());
      object->type = static_cast<base::ObjectType>(
          std::distance(object->type_probs.begin(),
                        std::max_element(object->type_probs.begin(),
                                         object->type_probs.end())));
    }

    if (cnnseg_param_.do_heading()) {
      // object->theta = cluster->yaw;
      // object->direction[0] = cos(cluster->yaw);
      // object->direction[1] = sin(cluster->yaw);
      // 2018.6.21, switch to axis rotated projection
      // should be reverted after retrain model.
      static const float quater_pi = static_cast<float>(M_PI) * 0.25f;
      object->theta = cluster->yaw - quater_pi;
      object->direction[0] = cosf(cluster->yaw - quater_pi);
      object->direction[1] = sinf(cluster->yaw - quater_pi);
      object->direction[2] = 0;
      object->lidar_supplement.is_orientation_ready = true;
    }
    ++valid;
  }
  objects->resize(valid);

  // add additional object seg logic with ncut if cnnseg miss detects
  /*if (cnnseg_param_.fill_recall_with_ncut() && secondary_segmentor) {
      mask.GetValidIndices(lidar_frame_ref_->secondary_indices);
      secondary_segmentor->Segment(SegmentationOptions(), lidar_frame_ref_);
  //segment based on lidar frame ref
  }*/

  collect_time_ = timer.toc(true);
}

bool CNNSegmentation::GetConfigs(std::string* param_file,
                                 std::string* proto_file,
                                 std::string* weight_file,
                                 std::string* engine_file) {
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  ACHECK(config_manager->GetModelConfig("CNNSegmentation", &model_config))
      << "Failed to get model config: CNNSegmentation";

  const std::string& work_root = config_manager->work_root();
  std::string root_path;
  ACHECK(model_config->get_value("root_path", &root_path))
      << "Failed to get value of root_path.";
  std::string config_file;
  config_file = GetAbsolutePath(work_root, root_path);
  config_file = GetAbsolutePath(config_file, sensor_name_);
  config_file = GetAbsolutePath(config_file, "cnnseg.conf");

  CNNSegConfig config;
  ACHECK(apollo::cyber::common::GetProtoFromFile(config_file, &config))
      << "Failed to parse CNNSeg config file";
  *proto_file = GetAbsolutePath(work_root, config.proto_file());
  *weight_file = GetAbsolutePath(work_root, config.weight_file());
  *param_file = GetAbsolutePath(work_root, config.param_file());
  *engine_file = GetAbsolutePath(work_root, config.engine_file());

  return true;
}

PERCEPTION_REGISTER_LIDARDETECTOR(CNNSegmentation);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
