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
#include <numeric>
#include <random>

#include <cuda_runtime_api.h>

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
      z_max_range_(Params::kMaxZRange),
      x_voxel_size_(Params::kVoxelXSize),
      y_voxel_size_(Params::kVoxelYSize),
      z_voxel_size_(Params::kVoxelZSize),
      max_num_points_in_voxel_(Params::kMaxNumPointsPerVoxel),
      num_point_feature_(Params::kNumPointFeature),
      max_voxels_(Params::kMaxNumVoxels){
  if (FLAGS_enable_ground_removal) {
    z_min_range_ =
        std::max(z_min_range_, static_cast<float>(FLAGS_ground_removal_height));
  }
}

bool CenterPointDetection::Init(const LidarDetectorInitOptions &options) {
  // whether use tensorrt and set the precision
  paddle::AnalysisConfig::Precision trt_precision =
      paddle_infer::PrecisionType::kFloat32;  // kFloat32, kHalf, kInt8
  predictor_ =
      create_predictor(FLAGS_center_point_model_file, FLAGS_center_point_params_file, use_trt_, trt_precision);
  return true;
}

// todo
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
  AINFO << "num points after fusing: " << num_points;
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

  // point cloud to array
  // points_array[x, y, z, i,timestampe, ......]
  float *points_array = new float[num_points * FLAGS_num_point_feature]();
  CloudToArray(cur_cloud_ptr_, points_array, FLAGS_normalizing_factor);
  cloud_to_array_time_ = timer.toc(true);

  // paddle inference
  std::vector<float> out_detections;
  std::vector<int64_t> out_labels;
  std::vector<float> out_scores;
  std::vector<float> out_detections_final;
  std::vector<int64_t> out_labels_final;

  DoInference(points_array, num_points, &out_detections, &out_labels,
              &out_scores);

  auto itr = out_scores.begin();
  while (itr != out_scores.end()) {
    itr++;
    AERROR << "Score: " << *itr;
  }

  filter_score(&out_detections, &out_labels, &out_scores, 0.5,
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

bool CenterPointDetection::Preprocess(
    const float *in_points_array, const int in_num_points,
    std::vector<int> *voxels_shape, std::vector<float> *voxels_data,
    std::vector<int> *num_points_shape, std::vector<int> *num_points_data,
    std::vector<int> *coords_shape, std::vector<int> *coords_data) {
  // int num_points;
  // todo  change the caluculation to proper position
  int grid_size_x =
      static_cast<int>(round((x_max_range_ - x_min_range_) / x_voxel_size_));
  int grid_size_y =
      static_cast<int>(round((y_max_range_ - y_min_range_) / y_voxel_size_));
  int grid_size_z =
      static_cast<int>(round((z_max_range_ - z_min_range_) / z_voxel_size_));

  float *voxels_ptr =
      new float[max_voxels_ * max_num_points_in_voxel_ * num_point_feature_]();
  int *num_points_ptr = new int[max_voxels_]();
  int *coords_ptr = new int[max_voxels_ * 3]();
  int *voxel_num_ptr = new int[1]();
  hard_voxelize(x_min_range_, y_min_range_, z_min_range_, x_voxel_size_,
                y_voxel_size_, z_voxel_size_, grid_size_x, grid_size_y,
                grid_size_z, max_num_points_in_voxel_, max_voxels_,
                in_points_array, num_point_feature_, in_num_points, voxels_ptr,
                coords_ptr, num_points_ptr, voxel_num_ptr);
  // free(points);
  int batch_idx = 0;
  int *coords_w_batch_idx_ptr = new int[max_voxels_ * (3 + 1)]();
  insert_batch_idx_to_coords(coords_ptr, max_voxels_, batch_idx,
                             coords_w_batch_idx_ptr);

  voxels_data->assign(voxels_ptr, voxels_ptr + voxel_num_ptr[0] *
                                                   max_num_points_in_voxel_ *
                                                   num_point_feature_);
  num_points_data->assign(num_points_ptr, num_points_ptr + voxel_num_ptr[0]);
  coords_data->assign(coords_w_batch_idx_ptr,
                      coords_w_batch_idx_ptr + voxel_num_ptr[0] * (3 + 1));
  voxels_shape->push_back(voxel_num_ptr[0]);
  voxels_shape->push_back(max_num_points_in_voxel_);
  voxels_shape->push_back(num_point_feature_);
  num_points_shape->push_back(voxel_num_ptr[0]);
  coords_shape->push_back(voxel_num_ptr[0]);
  coords_shape->push_back((3 + 1));  // batch_id, z, y, x

  delete[] voxels_ptr;
  delete[] num_points_ptr;
  delete[] coords_ptr;
  delete[] voxel_num_ptr;
  delete[] coords_w_batch_idx_ptr;

  return true;
}

bool CenterPointDetection::hard_voxelize(
    const float point_cloud_range_x_min, const float point_cloud_range_y_min,
    const float point_cloud_range_z_min, const float voxel_size_x,
    const float voxel_size_y, const float voxel_size_z, const int grid_size_x,
    const int grid_size_y, const int grid_size_z,
    const int max_num_points_in_voxel, const int max_voxels,
    const float *points, const int num_point_dim, const int num_points,
    float *voxels, int *coords, int *num_points_per_voxel, int *voxel_num) {
  voxel_num[0] = 0;
  int voxel_idx, grid_idx, curr_num_point;
  int coord_x, coord_y, coord_z;
  int *grid_idx_to_voxel_idx = new int[grid_size_x * grid_size_y * grid_size_z];
  memset(grid_idx_to_voxel_idx, -1,
         sizeof(int) * grid_size_x * grid_size_y * grid_size_z);

  for (int point_idx = 0; point_idx < num_points; ++point_idx) {
    coord_x = floor(
        (points[point_idx * num_point_dim + 0] - point_cloud_range_x_min) /
        voxel_size_x);
    coord_y = floor(
        (points[point_idx * num_point_dim + 1] - point_cloud_range_y_min) /
        voxel_size_y);
    coord_z = floor(
        (points[point_idx * num_point_dim + 2] - point_cloud_range_z_min) /
        voxel_size_z);

    if (coord_x < 0 || coord_x > grid_size_x || coord_x == grid_size_x) {
      continue;
    }
    if (coord_y < 0 || coord_y > grid_size_y || coord_y == grid_size_y) {
      continue;
    }
    if (coord_z < 0 || coord_z > grid_size_z || coord_z == grid_size_z) {
      continue;
    }
    grid_idx =
        coord_z * grid_size_y * grid_size_x + coord_y * grid_size_x + coord_x;
    voxel_idx = grid_idx_to_voxel_idx[grid_idx];
    if (voxel_idx == -1) {
      voxel_idx = voxel_num[0];
      if (voxel_num[0] == max_voxels || voxel_num[0] > max_voxels) {
        continue;
      }
      voxel_num[0]++;
      grid_idx_to_voxel_idx[grid_idx] = voxel_idx;
      coords[voxel_idx * 3 + 0] = coord_z;
      coords[voxel_idx * 3 + 1] = coord_y;
      coords[voxel_idx * 3 + 2] = coord_x;
    }
    curr_num_point = num_points_per_voxel[voxel_idx];
    // max_num_points_in_voxel;
    if (curr_num_point < max_num_points_in_voxel) {
      for (int j = 0; j < num_point_dim; ++j) {
        voxels[voxel_idx * max_num_points_in_voxel * num_point_dim +
               curr_num_point * num_point_dim + j] =
            points[point_idx * num_point_dim + j];
      }
      num_points_per_voxel[voxel_idx] = curr_num_point + 1;
    }
  }
  delete[] grid_idx_to_voxel_idx;
  return true;
}

void CenterPointDetection::DoInference(const float *in_points_array,
                                       const int in_num_points,
                                       std::vector<float> *out_detections,
                                       std::vector<int64_t> *out_labels,
                                       std::vector<float> *out_scores) {
  // todo: check gpu_id
  std::vector<float> voxel_data;
  std::vector<int> voxels_shape;
  std::vector<float> voxels_data;
  std::vector<int> num_points_shape;
  std::vector<int> num_points_data;
  std::vector<int> coords_shape;
  std::vector<int> coords_data;

  Preprocess(in_points_array, in_num_points, &voxels_shape, &voxels_data,
             &num_points_shape, &num_points_data, &coords_shape, &coords_data);

  run(predictor_.get(), voxels_shape, voxels_data, coords_shape, coords_data,
      num_points_shape, num_points_data, out_detections, out_labels,
      out_scores);
}

bool CenterPointDetection::insert_batch_idx_to_coords(
    const int *coords_ptr, const int voxel_num, const int batch_idx,
    int *coords_w_batch_idx_ptr) {
  int num_coord_w_batch_dim = 3 + 1;
  for (int i = 0; i < voxel_num; ++i) {
    coords_w_batch_idx_ptr[i * num_coord_w_batch_dim] = batch_idx;
    memcpy(coords_w_batch_idx_ptr + i * num_coord_w_batch_dim + 1,
           coords_ptr + i * 3, sizeof(int) * 3);
  }
  return true;
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

void CenterPointDetection::run(
    paddle_infer::Predictor *predictor, const std::vector<int> &voxels_shape,
    const std::vector<float> &voxels_data, const std::vector<int> &coords_shape,
    const std::vector<int> &coords_data,
    const std::vector<int> &num_points_shape,
    const std::vector<int> &num_points_data, std::vector<float> *box3d_lidar,
    std::vector<int64_t> *label_preds, std::vector<float> *scores) {

  auto input_names = predictor->GetInputNames();
  for (const auto &tensor_name : input_names) {
    auto in_tensor = predictor->GetInputHandle(tensor_name);
    if (tensor_name == "voxels") {
      in_tensor->Reshape(voxels_shape);
      in_tensor->CopyFromCpu(voxels_data.data());
    } else if (tensor_name == "coordinates") {
      in_tensor->Reshape(coords_shape);
      in_tensor->CopyFromCpu(coords_data.data());
    } else if (tensor_name == "num_points") {
      in_tensor->Reshape(num_points_shape);
      in_tensor->CopyFromCpu(num_points_data.data());
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

    // read params of bounding box
    float x = detections->at(i * FLAGS_num_output_box_feature + 0);
    float y = detections->at(i * FLAGS_num_output_box_feature + 1);
    float z = detections->at(i * FLAGS_num_output_box_feature + 2);
    float dx = detections->at(i * FLAGS_num_output_box_feature + 4);
    float dy = detections->at(i * FLAGS_num_output_box_feature + 3);
    float dz = detections->at(i * FLAGS_num_output_box_feature + 5);
    float yaw = detections->at(i * FLAGS_num_output_box_feature + 8);
    yaw += M_PI / 2;
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
      return base::ObjectSubType::PEDESTRIAN;
    case 2:  // construction vehicle
      return base::ObjectSubType::CYCLIST;
    default:
      return base::ObjectSubType::UNKNOWN;
  }
}

std::shared_ptr<paddle_infer::Predictor> CenterPointDetection::create_predictor(
    const std::string &model_path, const std::string &params_path,
    const bool use_trt, const paddle::AnalysisConfig::Precision &precision) {
  paddle::AnalysisConfig config;
  config.EnableUseGpu(100, 0);
  config.SetModel(model_path, params_path);
  if (use_trt) {
    config.EnableTensorRtEngine(1 << 20, 1, 7, precision, false, false);
    std::map<std::string, std::vector<int>> min_input_shape = {
        {"voxels", {1000, 20, 5}},
        {"coordinates", {1000, 4}},
        {"num_points", {1000}},
        {"atan2_1.tmp_0", {1, 128, 128, 1}},
        {"exp_1.tmp_0", {1, 128, 128, 3}},
        {"reshape2_22.tmp_0", {1, 16384, 1}},
        {"reshape2_30.tmp_0", {1, 16384, 2}},
        {"tmp_45", {1, 16384, 1}},
        {"tmp_48", {1, 16384, 1}},
        {"atan2_4.tmp_0", {1, 128, 128, 1}},
        {"exp_4.tmp_0", {1, 128, 128, 3}},
        {"reshape2_58.tmp_0", {1, 16384, 1}},
        {"reshape2_66.tmp_0", {1, 16384, 2}},
        {"tmp_114", {1, 16384, 1}},
        {"tmp_117", {1, 16384, 1}},
        {"scatter_0.tmp_0", {262144, 64}},
        {"tmp_108", {1, 128, 128}},
        {"tmp_109", {1, 128, 128}},
        {"tmp_131", {1, 128, 128}},
        {"tmp_132", {1, 128, 128}},
        {"tmp_16", {1, 128, 128}},
        {"tmp_17", {1, 128, 128}},
        {"tmp_39", {1, 128, 128}},
        {"tmp_40", {1, 128, 128}},
        {"tmp_62", {1, 128, 128}},
        {"tmp_63", {1, 128, 128}},
        {"tmp_85", {1, 128, 128}},
        {"tmp_86", {1, 128, 128}},
        {"relu_0.tmp_0", {1000, 20, 32}},
        {"tile_0.tmp_0", {1000, 20, 32}},
        {"atan2_3.tmp_0", {1, 128, 128, 1}},
        {"exp_3.tmp_0", {1, 128, 128, 3}},
        {"reshape2_46.tmp_0", {1, 16384, 1}},
        {"reshape2_54.tmp_0", {1, 16384, 2}},
        {"tmp_91", {1, 16384, 1}},
        {"tmp_94", {1, 16384, 1}},
        {"atan2_0.tmp_0", {1, 128, 128, 1}},
        {"exp_0.tmp_0", {1, 128, 128, 3}},
        {"reshape2_10.tmp_0", {1, 16384, 1}},
        {"reshape2_18.tmp_0", {1, 16384, 2}},
        {"tmp_22", {1, 16384, 1}},
        {"tmp_25", {1, 16384, 1}},
        {"atan2_2.tmp_0", {1, 128, 128, 1}},
        {"exp_2.tmp_0", {1, 128, 128, 3}},
        {"reshape2_34.tmp_0", {1, 16384, 1}},
        {"reshape2_42.tmp_0", {1, 16384, 2}},
        {"tmp_68", {1, 16384, 1}},
        {"tmp_71", {1, 16384, 1}},
        {"_generated_var_110", {1}},
        {"_generated_var_138", {1}},
        {"_generated_var_166", {1}},
        {"_generated_var_26", {1}},
        {"_generated_var_54", {1}},
        {"_generated_var_82", {1}},
        {"cast_0.tmp_0", {1000}},
        {"full_like_0.tmp_0", {1000, 20, 2}},
        {"tmp_11", {1000, 20, 1}},
        {"atan2_5.tmp_0", {1, 128, 128, 1}},
        {"exp_5.tmp_0", {1, 128, 128, 3}},
        {"reshape2_70.tmp_0", {1, 16384, 1}},
        {"reshape2_78.tmp_0", {1, 16384, 2}},
        {"tmp_137", {1, 16384, 1}},
        {"tmp_140", {1, 16384, 1}}};
    std::map<std::string, std::vector<int>> max_input_shape = {
        {"voxels", {60000, 20, 5}},
        {"coordinates", {60000, 4}},
        {"num_points", {60000}},
        {"atan2_1.tmp_0", {1, 128, 128, 1}},
        {"exp_1.tmp_0", {1, 128, 128, 3}},
        {"reshape2_22.tmp_0", {1, 16384, 1}},
        {"reshape2_30.tmp_0", {1, 16384, 2}},
        {"tmp_45", {1, 16384, 1}},
        {"tmp_48", {1, 16384, 1}},
        {"atan2_4.tmp_0", {1, 128, 128, 1}},
        {"exp_4.tmp_0", {1, 128, 128, 3}},
        {"reshape2_58.tmp_0", {1, 16384, 1}},
        {"reshape2_66.tmp_0", {1, 16384, 2}},
        {"tmp_114", {1, 16384, 1}},
        {"tmp_117", {1, 16384, 1}},
        {"scatter_0.tmp_0", {262144, 64}},
        {"tmp_108", {1, 128, 128}},
        {"tmp_109", {1, 128, 128}},
        {"tmp_131", {1, 128, 128}},
        {"tmp_132", {1, 128, 128}},
        {"tmp_16", {1, 128, 128}},
        {"tmp_17", {1, 128, 128}},
        {"tmp_39", {1, 128, 128}},
        {"tmp_40", {1, 128, 128}},
        {"tmp_62", {1, 128, 128}},
        {"tmp_63", {1, 128, 128}},
        {"tmp_85", {1, 128, 128}},
        {"tmp_86", {1, 128, 128}},
        {"relu_0.tmp_0", {60000, 20, 32}},
        {"tile_0.tmp_0", {60000, 20, 32}},
        {"atan2_3.tmp_0", {1, 128, 128, 1}},
        {"exp_3.tmp_0", {1, 128, 128, 3}},
        {"reshape2_46.tmp_0", {1, 16384, 1}},
        {"reshape2_54.tmp_0", {1, 16384, 2}},
        {"tmp_91", {1, 16384, 1}},
        {"tmp_94", {1, 16384, 1}},
        {"atan2_0.tmp_0", {1, 128, 128, 1}},
        {"exp_0.tmp_0", {1, 128, 128, 3}},
        {"reshape2_10.tmp_0", {1, 16384, 1}},
        {"reshape2_18.tmp_0", {1, 16384, 2}},
        {"tmp_22", {1, 16384, 1}},
        {"tmp_25", {1, 16384, 1}},
        {"atan2_2.tmp_0", {1, 128, 128, 1}},
        {"exp_2.tmp_0", {1, 128, 128, 3}},
        {"reshape2_34.tmp_0", {1, 16384, 1}},
        {"reshape2_42.tmp_0", {1, 16384, 2}},
        {"tmp_68", {1, 16384, 1}},
        {"tmp_71", {1, 16384, 1}},
        {"_generated_var_110", {1000}},
        {"_generated_var_138", {1000}},
        {"_generated_var_166", {1000}},
        {"_generated_var_26", {1000}},
        {"_generated_var_54", {1000}},
        {"_generated_var_82", {1000}},
        {"cast_0.tmp_0", {60000}},
        {"full_like_0.tmp_0", {60000, 20, 2}},
        {"tmp_11", {60000, 20, 1}},
        {"atan2_5.tmp_0", {1, 128, 128, 1}},
        {"exp_5.tmp_0", {1, 128, 128, 3}},
        {"reshape2_70.tmp_0", {1, 16384, 1}},
        {"reshape2_78.tmp_0", {1, 16384, 2}},
        {"tmp_137", {1, 16384, 1}},
        {"tmp_140", {1, 16384, 1}}};
    std::map<std::string, std::vector<int>> opt_input_shape = {
        {"voxels", {60000, 20, 5}},
        {"coordinates", {60000, 4}},
        {"num_points", {60000}},
        {"atan2_1.tmp_0", {1, 128, 128, 1}},
        {"exp_1.tmp_0", {1, 128, 128, 3}},
        {"reshape2_22.tmp_0", {1, 16384, 1}},
        {"reshape2_30.tmp_0", {1, 16384, 2}},
        {"tmp_45", {1, 16384, 1}},
        {"tmp_48", {1, 16384, 1}},
        {"atan2_4.tmp_0", {1, 128, 128, 1}},
        {"exp_4.tmp_0", {1, 128, 128, 3}},
        {"reshape2_58.tmp_0", {1, 16384, 1}},
        {"reshape2_66.tmp_0", {1, 16384, 2}},
        {"tmp_114", {1, 16384, 1}},
        {"tmp_117", {1, 16384, 1}},
        {"scatter_0.tmp_0", {262144, 64}},
        {"tmp_108", {1, 128, 128}},
        {"tmp_109", {1, 128, 128}},
        {"tmp_131", {1, 128, 128}},
        {"tmp_132", {1, 128, 128}},
        {"tmp_16", {1, 128, 128}},
        {"tmp_17", {1, 128, 128}},
        {"tmp_39", {1, 128, 128}},
        {"tmp_40", {1, 128, 128}},
        {"tmp_62", {1, 128, 128}},
        {"tmp_63", {1, 128, 128}},
        {"tmp_85", {1, 128, 128}},
        {"tmp_86", {1, 128, 128}},
        {"relu_0.tmp_0", {60000, 20, 32}},
        {"tile_0.tmp_0", {60000, 20, 32}},
        {"atan2_3.tmp_0", {1, 128, 128, 1}},
        {"exp_3.tmp_0", {1, 128, 128, 3}},
        {"reshape2_46.tmp_0", {1, 16384, 1}},
        {"reshape2_54.tmp_0", {1, 16384, 2}},
        {"tmp_91", {1, 16384, 1}},
        {"tmp_94", {1, 16384, 1}},
        {"atan2_0.tmp_0", {1, 128, 128, 1}},
        {"exp_0.tmp_0", {1, 128, 128, 3}},
        {"reshape2_10.tmp_0", {1, 16384, 1}},
        {"reshape2_18.tmp_0", {1, 16384, 2}},
        {"tmp_22", {1, 16384, 1}},
        {"tmp_25", {1, 16384, 1}},
        {"atan2_2.tmp_0", {1, 128, 128, 1}},
        {"exp_2.tmp_0", {1, 128, 128, 3}},
        {"reshape2_34.tmp_0", {1, 16384, 1}},
        {"reshape2_42.tmp_0", {1, 16384, 2}},
        {"tmp_68", {1, 16384, 1}},
        {"tmp_71", {1, 16384, 1}},
        {"_generated_var_110", {83}},
        {"_generated_var_138", {83}},
        {"_generated_var_166", {83}},
        {"_generated_var_26", {83}},
        {"_generated_var_54", {83}},
        {"_generated_var_82", {83}},
        {"cast_0.tmp_0", {60000}},
        {"full_like_0.tmp_0", {60000, 20, 2}},
        {"tmp_11", {60000, 20, 1}},
        {"atan2_5.tmp_0", {1, 128, 128, 1}},
        {"exp_5.tmp_0", {1, 128, 128, 3}},
        {"reshape2_70.tmp_0", {1, 16384, 1}},
        {"reshape2_78.tmp_0", {1, 16384, 2}},
        {"tmp_137", {1, 16384, 1}},
        {"tmp_140", {1, 16384, 1}}};
    config.SetTRTDynamicShapeInfo(min_input_shape, max_input_shape,
                                  opt_input_shape);
  }
  return paddle_infer::CreatePredictor(config);
}

void CenterPointDetection::filter_score(
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