/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/lidar/lib/detector/center_point_detection/center_point_detection.h"

#include <algorithm>
#include <functional>
#include <numeric>
#include <random>

#if GPU_PLATFORM == NVIDIA
  #include <cuda_runtime_api.h>
#elif GPU_PLATFORM == AMD
  #include <hip/hip_runtime.h>
  #include <hip/hip_runtime_api.h>
#endif
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/base/point_cloud_util.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lidar/common/lidar_timer.h"
#include "modules/perception/lidar/common/pcl_util.h"
#include "modules/perception/lidar/lib/detector/center_point_detection/params.h"

namespace apollo {
namespace perception {
namespace lidar {

using base::Object;
using base::PointD;
using base::PointF;

// point cloud range
CenterPointDetection::CenterPointDetection()
    : x_min_range_(Params::kMinXRange),
      x_max_range_(Params::kMaxXRange),
      y_min_range_(Params::kMinYRange),
      y_max_range_(Params::kMaxYRange),
      z_min_range_(Params::kMinZRange),
      z_max_range_(Params::kMaxZRange) {
  if (FLAGS_enable_ground_removal) {
    z_min_range_ =
        std::max(z_min_range_, static_cast<float>(FLAGS_ground_removal_height));
  }
}

bool CenterPointDetection::Init(const LidarDetectorInitOptions &options) {
  /*
  num_point_feature
  */
  paddle::AnalysisConfig config;
  config.EnableUseGpu(1000, FLAGS_gpu_id);
  config.SetModel(FLAGS_center_point_model_file,
                  FLAGS_center_point_params_file);
  if (FLAGS_use_trt) {
    paddle::AnalysisConfig::Precision precision;
    if (FLAGS_trt_precision == 0) {
      precision = paddle_infer::PrecisionType::kFloat32;
    } else if (FLAGS_trt_precision == 1) {
      precision = paddle_infer::PrecisionType::kHalf;
    } else {
      AERROR << "Tensorrt type can only support 0 or 1, but recieved is"
             << FLAGS_trt_precision << "\n";
      return false;
    }
    config.EnableTensorRtEngine(1 << 30, 1, 3, precision, FLAGS_trt_use_static,
                                false);
    // todo: solve EnableTunedTensorRtDynamicShape
    config.CollectShapeRangeInfo(FLAGS_dynamic_shape_file);
    // config.EnableTunedTensorRtDynamicShape(FLAGS_dynamic_shape_file, true);

    if (FLAGS_trt_use_static) {
      config.SetOptimCacheDir(FLAGS_trt_static_dir);
    }
  }
  config.SwitchIrOptim(true);

  predictor_ = paddle_infer::CreatePredictor(config);
  return true;
}

bool CenterPointDetection::Init(const StageConfig& stage_config) {
  if (!Initialize(stage_config)) {
    return false;
  }

  /*
  num_point_feature
  */
  paddle::AnalysisConfig config;
  config.EnableUseGpu(1000, FLAGS_gpu_id);
  config.SetModel(FLAGS_center_point_model_file,
                  FLAGS_center_point_params_file);
  config.EnableMemoryOptim();
  if (FLAGS_use_trt) {
    paddle::AnalysisConfig::Precision precision;
    if (FLAGS_trt_precision == 0) {
      precision = paddle_infer::PrecisionType::kFloat32;
    } else if (FLAGS_trt_precision == 1) {
      precision = paddle_infer::PrecisionType::kHalf;
    } else {
      AERROR << "Tensorrt type can only support 0 or 1, but recieved is"
             << FLAGS_trt_precision << "\n";
      return false;
    }
    config.EnableTensorRtEngine(1 << 30, 1, 3, precision, FLAGS_trt_use_static,
                                false);
    // todo: solve EnableTunedTensorRtDynamicShape
    config.CollectShapeRangeInfo(FLAGS_dynamic_shape_file);
    // config.EnableTunedTensorRtDynamicShape(FLAGS_dynamic_shape_file, true);

    if (FLAGS_trt_use_static) {
      config.SetOptimCacheDir(FLAGS_trt_static_dir);
    }
  }
  config.SwitchIrOptim(true);

  predictor_ = paddle_infer::CreatePredictor(config);
  return true;
}

bool CenterPointDetection::Process(DataFrame* data_frame) {
  if (data_frame == nullptr)
    return false;

  LidarFrame* lidar_frame = data_frame->lidar_frame;
  if (lidar_frame == nullptr)
    return false;

  LidarDetectorOptions options;
  bool res = Detect(options, lidar_frame);
  return res;
}

bool CenterPointDetection::Detect(const LidarDetectorOptions &options,
                                  LidarFrame *frame) {
  // check input
  if (frame == nullptr) {
    AERROR << "Input null frame ptr.";
    return false;
  }
  if (frame->cloud == nullptr) {
    AERROR << "Input null frame cloud.";
    return false;
  }
  if (frame->cloud->size() == 0) {
    AERROR << "Input none points.";
    return false;
  }

  // record input cloud and lidar frame
  original_cloud_ = frame->cloud;
  original_world_cloud_ = frame->world_cloud;
  lidar_frame_ref_ = frame;

  // check output
  frame->segmented_objects.clear();

  if (cudaSetDevice(FLAGS_gpu_id) != cudaSuccess) {
    AERROR << "Failed to set device to gpu " << FLAGS_gpu_id;
    return false;
  }

  Timer timer;

  int num_points;
  cur_cloud_ptr_ = std::shared_ptr<base::PointFCloud>(
      new base::PointFCloud(*original_cloud_));

  // down sample the point cloud through filtering beams
  if (FLAGS_enable_downsample_beams) {
    base::PointFCloudPtr downsample_beams_cloud_ptr(new base::PointFCloud());
    if (DownSamplePointCloudBeams(original_cloud_, downsample_beams_cloud_ptr,
                                  FLAGS_downsample_beams_factor)) {
      cur_cloud_ptr_ = downsample_beams_cloud_ptr;
    } else {
      AWARN << "Down-sample beams factor must be >= 1. Cancel down-sampling."
               " Current factor: "
            << FLAGS_downsample_beams_factor;
    }
  }

  // down sample the point cloud through filtering voxel grid
  if (FLAGS_enable_downsample_pointcloud) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>());
    TransformToPCLXYZI(*cur_cloud_ptr_, pcl_cloud_ptr);
    DownSampleCloudByVoxelGrid(
        pcl_cloud_ptr, filtered_cloud_ptr, FLAGS_downsample_voxel_size_x,
        FLAGS_downsample_voxel_size_y, FLAGS_downsample_voxel_size_z);

    // transform pcl point cloud to apollo point cloud
    base::PointFCloudPtr downsample_voxel_cloud_ptr(new base::PointFCloud());
    TransformFromPCLXYZI(filtered_cloud_ptr, downsample_voxel_cloud_ptr);
    cur_cloud_ptr_ = downsample_voxel_cloud_ptr;
  }
  downsample_time_ = timer.toc(true);

  num_points = cur_cloud_ptr_->size();
  AINFO << "num points before fusing: " << num_points;

  // fuse clouds of preceding frames with current cloud
  cur_cloud_ptr_->mutable_points_timestamp()->assign(cur_cloud_ptr_->size(),
                                                     0.0);
  if (FLAGS_enable_fuse_frames && FLAGS_num_fuse_frames > 1) {
    // before fusing
    while (!prev_world_clouds_.empty() &&
           frame->timestamp - prev_world_clouds_.front()->get_timestamp() >
               FLAGS_fuse_time_interval) {
      prev_world_clouds_.pop_front();
    }
    // transform current cloud to world coordinate and save to a new ptr
    base::PointDCloudPtr cur_world_cloud_ptr =
        std::make_shared<base::PointDCloud>();
    for (size_t i = 0; i < cur_cloud_ptr_->size(); ++i) {
      auto &pt = cur_cloud_ptr_->at(i);
      Eigen::Vector3d trans_point(pt.x, pt.y, pt.z);
      trans_point = lidar_frame_ref_->lidar2world_pose * trans_point;
      PointD world_point;
      world_point.x = trans_point(0);
      world_point.y = trans_point(1);
      world_point.z = trans_point(2);
      world_point.intensity = pt.intensity;
      cur_world_cloud_ptr->push_back(world_point);
    }
    cur_world_cloud_ptr->set_timestamp(frame->timestamp);

    // fusing clouds
    for (auto &prev_world_cloud_ptr : prev_world_clouds_) {
      num_points += prev_world_cloud_ptr->size();
    }
    FuseCloud(cur_cloud_ptr_, prev_world_clouds_);

    // after fusing
    while (static_cast<int>(prev_world_clouds_.size()) >=
           FLAGS_num_fuse_frames - 1) {
      prev_world_clouds_.pop_front();
    }
    prev_world_clouds_.emplace_back(cur_world_cloud_ptr);
  }
  fuse_time_ = timer.toc(true);

  // shuffle points and cut off
  if (FLAGS_enable_shuffle_points) {
    num_points = std::min(num_points, FLAGS_max_num_points);
    std::vector<int> point_indices = GenerateIndices(0, num_points, true);
    base::PointFCloudPtr shuffle_cloud_ptr(
        new base::PointFCloud(*cur_cloud_ptr_, point_indices));
    cur_cloud_ptr_ = shuffle_cloud_ptr;
  }
  shuffle_time_ = timer.toc(true);

  // points_array[x, y, z, i,timestampe, ......]
  std::vector<float> points_data(num_points * FLAGS_num_point_feature);
  CloudToArray(cur_cloud_ptr_, points_data.data(), FLAGS_normalizing_factor);
  cloud_to_array_time_ = timer.toc(true);

  // paddle inference
  std::vector<float> out_detections;
  std::vector<int64_t> out_labels;
  std::vector<float> out_scores;
  std::vector<float> out_detections_final;
  std::vector<int64_t> out_labels_final;

  DoInference(points_data, num_points, &out_detections, &out_labels,
              &out_scores);

  FilterScore(&out_detections, &out_labels, &out_scores, FLAGS_score_threshold,
               &out_detections_final, &out_labels_final);

  GetObjects(&frame->segmented_objects, frame->lidar2world_pose,
             &out_detections_final, &out_labels_final);
  inference_time_ = timer.toc(true);

  AINFO << "CenterPoint: "
        << "\n"
        << "down sample: " << downsample_time_ << "\t"
        << "fuse: " << fuse_time_ << "\t"
        << "shuffle: " << shuffle_time_ << "\t"
        << "cloud_to_array: " << cloud_to_array_time_ << "\t"
        << "inference: " << inference_time_ << "\t";
  return true;
}

// normalizing_factor: Normalize intensity range to [0, 1] by this factor
void CenterPointDetection::CloudToArray(const base::PointFCloudPtr &pc_ptr,
                                        float *out_points_array,
                                        const float normalizing_factor) {
  for (size_t i = 0; i < pc_ptr->size(); ++i) {
    const auto &point = pc_ptr->at(i);
    float x = point.x;
    float y = point.y;
    float z = point.z;
    float intensity = point.intensity;
    if (z < z_min_range_ || z > z_max_range_ || y < y_min_range_ ||
        y > y_max_range_ || x < x_min_range_ || x > x_max_range_) {
      continue;
    }
    out_points_array[i * FLAGS_num_point_feature + 0] = x;
    out_points_array[i * FLAGS_num_point_feature + 1] = y;
    out_points_array[i * FLAGS_num_point_feature + 2] = z;
    out_points_array[i * FLAGS_num_point_feature + 3] =
        intensity / normalizing_factor;
    // delta of timestamp between prev and cur frames
    out_points_array[i * FLAGS_num_point_feature + 4] =
        static_cast<float>(pc_ptr->points_timestamp(i));
  }
}

void CenterPointDetection::FuseCloud(
    const base::PointFCloudPtr &out_cloud_ptr,
    const std::deque<base::PointDCloudPtr> &fuse_clouds) {
  for (auto iter = fuse_clouds.rbegin(); iter != fuse_clouds.rend(); ++iter) {
    double delta_t = lidar_frame_ref_->timestamp - (*iter)->get_timestamp();
    // transform prev world point cloud to current sensor's coordinates
    for (size_t i = 0; i < (*iter)->size(); ++i) {
      auto &point = (*iter)->at(i);
      Eigen::Vector3d trans_point(point.x, point.y, point.z);
      trans_point = lidar_frame_ref_->lidar2world_pose.inverse() * trans_point;
      base::PointF pt;
      pt.x = static_cast<float>(trans_point(0));
      pt.y = static_cast<float>(trans_point(1));
      pt.z = static_cast<float>(trans_point(2));
      pt.intensity = static_cast<float>(point.intensity);
      // delta of time between current and prev frame
      out_cloud_ptr->push_back(pt, delta_t);
    }
  }
}

void CenterPointDetection::DoInference(const std::vector<float> &points_data,
                                       const int in_num_points,
                                       std::vector<float> *out_detections,
                                       std::vector<int64_t> *out_labels,
                                       std::vector<float> *out_scores) {
  // todo: check gpu_id
  std::vector<int> points_shape;
  points_shape.push_back(in_num_points);
  points_shape.push_back(FLAGS_num_point_feature);

  Run(predictor_.get(), points_shape, points_data, out_detections, out_labels,
      out_scores);
}

std::vector<int> CenterPointDetection::GenerateIndices(int start_index,
                                                       int size, bool shuffle) {
  // create a range number array
  std::vector<int> indices(size);
  std::iota(indices.begin(), indices.end(), start_index);

  // shuffle the index array
  if (shuffle) {
    unsigned seed = 0;
    std::shuffle(indices.begin(), indices.end(),
                 std::default_random_engine(seed));
  }
  return indices;
}

void CenterPointDetection::Run(paddle_infer::Predictor *predictor,
                               const std::vector<int> &points_shape,
                               const std::vector<float> &points_data,
                               std::vector<float> *box3d_lidar,
                               std::vector<int64_t> *label_preds,
                               std::vector<float> *scores) {
  auto input_names = predictor->GetInputNames();
  for (const auto &tensor_name : input_names) {
    auto in_tensor = predictor->GetInputHandle(tensor_name);
    if (tensor_name == "data") {
      in_tensor->Reshape(points_shape);
      in_tensor->CopyFromCpu(points_data.data());
    }
  }
  ACHECK(predictor->Run());

  auto output_names = predictor->GetOutputNames();
  for (size_t i = 0; i != output_names.size(); i++) {
    auto output = predictor->GetOutputHandle(output_names[i]);
    std::vector<int> output_shape = output->shape();
    int out_num = std::accumulate(output_shape.begin(), output_shape.end(), 1,
                                  std::multiplies<int>());
    if (i == 0) {
      box3d_lidar->resize(out_num);
      output->CopyToCpu(box3d_lidar->data());
    } else if (i == 1) {
      label_preds->resize(out_num);
      output->CopyToCpu(label_preds->data());
    } else if (i == 2) {
      scores->resize(out_num);
      output->CopyToCpu(scores->data());
    }
  }
}

void CenterPointDetection::GetObjects(
    std::vector<std::shared_ptr<Object>> *objects, const Eigen::Affine3d &pose,
    std::vector<float> *detections, std::vector<int64_t> *labels) {
  int num_objects = detections->size() / num_output_box_feature_;

  objects->clear();
  base::ObjectPool::Instance().BatchGet(num_objects, objects);

  for (int i = 0; i < num_objects; ++i) {
    auto &object = objects->at(i);
    object->id = i;

    // no velocity
    float x = detections->at(i * FLAGS_num_output_box_feature + 0);
    float y = detections->at(i * FLAGS_num_output_box_feature + 1);
    float z = detections->at(i * FLAGS_num_output_box_feature + 2);
    float dx = detections->at(i * FLAGS_num_output_box_feature + 3);
    float dy = detections->at(i * FLAGS_num_output_box_feature + 4);
    float dz = detections->at(i * FLAGS_num_output_box_feature + 5);
    float yaw = detections->at(i * FLAGS_num_output_box_feature + 6);
    // yaw += M_PI / 2;
    yaw = std::atan2(sinf(yaw), cosf(yaw));
    yaw = -yaw;

    // directions
    object->theta = yaw;
    object->direction[0] = cosf(yaw);
    object->direction[1] = sinf(yaw);
    object->direction[2] = 0;
    object->lidar_supplement.is_orientation_ready = true;

    // compute vertexes of bounding box and transform to world coordinate
    object->lidar_supplement.num_points_in_roi = 8;
    object->lidar_supplement.on_use = true;
    object->lidar_supplement.is_background = false;
    float roll = 0, pitch = 0;
    Eigen::Quaternionf quater =
        Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f translation(x, y, z);
    Eigen::Affine3f affine3f = translation * quater.toRotationMatrix();
    for (float vx : std::vector<float>{dx / 2, -dx / 2}) {
      for (float vy : std::vector<float>{dy / 2, -dy / 2}) {
        for (float vz : std::vector<float>{0, dz}) {
          Eigen::Vector3f v3f(vx, vy, vz);
          v3f = affine3f * v3f;
          PointF point;
          point.x = v3f.x();
          point.y = v3f.y();
          point.z = v3f.z();
          object->lidar_supplement.cloud.push_back(point);

          Eigen::Vector3d trans_point(point.x, point.y, point.z);
          trans_point = pose * trans_point;
          PointD world_point;
          world_point.x = trans_point(0);
          world_point.y = trans_point(1);
          world_point.z = trans_point(2);
          object->lidar_supplement.cloud_world.push_back(world_point);
        }
      }
    }

    // classification
    object->lidar_supplement.raw_probs.push_back(std::vector<float>(
        static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
    object->lidar_supplement.raw_classification_methods.push_back(Name());
    object->sub_type = GetObjectSubType(labels->at(i));
    object->type = base::kSubType2TypeMap.at(object->sub_type);
    object->lidar_supplement.raw_probs.back()[static_cast<int>(object->type)] =
        1.0f;
    // copy to type
    object->type_probs.assign(object->lidar_supplement.raw_probs.back().begin(),
                              object->lidar_supplement.raw_probs.back().end());
  }
}

base::ObjectSubType CenterPointDetection::GetObjectSubType(const int label) {
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

void CenterPointDetection::FilterScore(
    const std::vector<float> *box3d_lidar,
    const std::vector<int64_t> *label_preds, const std::vector<float> *scores,
    const float score_threshold, std::vector<float> *box3d_lidar_final,
    std::vector<int64_t> *label_preds_final) {
  for (size_t i = 0; i < scores->size(); i++) {
    if (scores->at(i) > score_threshold) {
      box3d_lidar_final->insert(
          box3d_lidar_final->end(),
          box3d_lidar->begin() + num_output_box_feature_ * i,
          box3d_lidar->begin() + num_output_box_feature_ * (i + 1));
      label_preds_final->insert(label_preds_final->end(),
                                *(label_preds->begin() + i));
    }
  }
}

PERCEPTION_REGISTER_LIDARDETECTOR(CenterPointDetection);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
