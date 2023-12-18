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

#include "modules/perception/lidar_detection/detector/center_point_detection/center_point_detection.h"

#include <algorithm>
#include <functional>
#include <numeric>
#include <random>
#include <utility>

#if GPU_PLATFORM == NVIDIA
#include <cuda_runtime_api.h>
#elif GPU_PLATFORM == AMD
#include <hip/hip_runtime.h>
#include <hip/hip_runtime_api.h>
#endif

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/common/base/object_pool_types.h"
#include "modules/perception/common/base/point_cloud_util.h"
#include "modules/perception/common/inference/inference_factory.h"
#include "modules/perception/common/inference/model_util.h"
#include "modules/perception/common/lidar/common/cloud_mask.h"
#include "modules/perception/common/lidar/common/lidar_timer.h"
#include "modules/perception/common/lidar/common/pcl_util.h"
#include "modules/perception/common/util.h"
#include "modules/perception/lidar_detection/detector/center_point_detection/params.h"

#include "cyber/profiler/profiler.h"

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
      z_max_range_(Params::kMaxZRange) {}

bool CenterPointDetection::Init(const LidarDetectorInitOptions &options) {
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);
  ACHECK(apollo::cyber::common::GetProtoFromFile(config_file, &model_param_));

  const auto &model_info = model_param_.info();
  std::string model_path = GetModelPath(model_info.name());
  // Network files
  std::string proto_file =
      GetModelFile(model_path, model_info.proto_file().file());
  std::string weight_file =
      GetModelFile(model_path, model_info.weight_file().file());
  // Network input and output names
  input_blob_names_ = inference::GetBlobNames(model_info.inputs());
  output_blob_names_ = inference::GetBlobNames(model_info.outputs());

  inference_.reset(inference::CreateInferenceByName(
      model_info.framework(), proto_file, weight_file, output_blob_names_,
      input_blob_names_));

  std::map<std::string, std::vector<int>> shape_map;
  inference::AddShape(&shape_map, model_info.inputs());
  inference::AddShape(&shape_map, model_info.outputs());

  if (!inference_->Init(shape_map)) {
    return false;
  }

  if (model_param_.preprocess().enable_ground_removal()) {
    z_min_range_ = std::max(
        z_min_range_,
        static_cast<float>(model_param_.preprocess().ground_removal_height()));
  }
  nms_strategy_ = model_param_.nms_strategy();
  diff_class_iou_ = model_param_.diff_class_iou();
  diff_class_nms_ = model_param_.diff_class_nms();
  if (diff_class_nms_) {
    nms_strategy_table_ = {
      {base::ObjectType::BICYCLE, {base::ObjectType::PEDESTRIAN,
       base::ObjectType::UNKNOWN}},
      {base::ObjectType::PEDESTRIAN, {base::ObjectType::UNKNOWN}},
      {base::ObjectType::VEHICLE, {base::ObjectType::BICYCLE,
       base::ObjectType::PEDESTRIAN, base::ObjectType::UNKNOWN}}
    };
  }
  return true;
}

float get_3Dbox_iou_len(float center1, float len1, float center2, float len2) {
    float x11 = center1 - len1 / 2;
    float x12 = center2 - len2 / 2;
    float x21 = center1 + len1 / 2;
    float x22 = center2 + len2 / 2;
    if (std::min(x22, x21) - std::max(x12, x11) < 0) {
        return 0;
    }
    return std::min(x22, x21) - std::max(x12, x11);
}

float get_3dbox_iou(base::ObjectPtr obj1, base::ObjectPtr obj2,
                    int index1, int index2) {
    auto center1 = obj1->center;
    auto center2 = obj2->center;
    auto size1 = obj1->size;
    auto size2 = obj2->size;
    float x_len = get_3Dbox_iou_len(center1(0), size1(0), center2(0), size2(0));
    float y_len = get_3Dbox_iou_len(center1(1), size1(1), center2(1), size2(1));
    float z_len = get_3Dbox_iou_len(center1(2), size1(2), center2(2), size2(2));
    float v1 = size1(0) * size1(1) * size1(2);
    float v2 = size2(0) * size2(1) * size2(2);
    float vo = x_len * y_len * z_len;
    ADEBUG << "center point IOU: (" << index1 << ", " << index2
           << ") v1:" << v1 << " v2:" << v2 << " vo:" << vo;
    return vo / (v1 + v2 - vo);
}

bool nms_by_strategy(std::shared_ptr<base::Object> obj1,
            std::shared_ptr<base::Object> obj2,
            std::map<base::ObjectType, std::vector<base::ObjectType>> table) {
    if (obj1.get()->type == obj2.get()->type) {
        return obj1.get()->confidence > obj2.get()->confidence;
    }
    std::vector<base::ObjectType> obj_type_array = table[obj1.get()->type];
    if (std::find(obj_type_array.begin(), obj_type_array.end(),
                  obj2->type) != obj_type_array.end()) {
        return true;
    } else {
        return false;
    }
    return true;
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

  if (cudaSetDevice(model_param_.preprocess().gpu_id()) != cudaSuccess) {
    AERROR << "Failed to set device to gpu "
           << model_param_.preprocess().gpu_id();
    return false;
  }

  Timer timer;

  int num_points;
  cur_cloud_ptr_ = std::shared_ptr<base::PointFCloud>(
      new base::PointFCloud(*original_cloud_));

  if (model_param_.preprocess().enable_roi_outside_removal()) {
    cur_cloud_ptr_->CopyPointCloud(*original_cloud_, frame->roi_indices);
  }

  // down sample the point cloud through filtering beams
  if (model_param_.preprocess().enable_downsample_beams()) {
    base::PointFCloudPtr downsample_beams_cloud_ptr(new base::PointFCloud());
    if (DownSamplePointCloudBeams(
            cur_cloud_ptr_, downsample_beams_cloud_ptr,
            model_param_.preprocess().downsample_beams_factor())) {
      cur_cloud_ptr_ = downsample_beams_cloud_ptr;
    } else {
      AWARN << "Down-sample beams factor must be >= 1. Cancel down-sampling."
               " Current factor: "
            << model_param_.preprocess().downsample_beams_factor();
    }
  }

  // down sample the point cloud through filtering voxel grid
  if (model_param_.preprocess().enable_downsample_pointcloud()) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>());
    TransformToPCLXYZI(*cur_cloud_ptr_, pcl_cloud_ptr);
    DownSampleCloudByVoxelGrid(
        pcl_cloud_ptr, filtered_cloud_ptr,
        model_param_.preprocess().downsample_voxel_size_x(),
        model_param_.preprocess().downsample_voxel_size_y(),
        model_param_.preprocess().downsample_voxel_size_z());

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
  if (model_param_.preprocess().enable_fuse_frames() &&
      model_param_.preprocess().num_fuse_frames() > 1) {
    // before fusing
    while (!prev_world_clouds_.empty() &&
           frame->timestamp - prev_world_clouds_.front()->get_timestamp() >
               model_param_.preprocess().fuse_time_interval()) {
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
           model_param_.preprocess().num_fuse_frames() - 1) {
      prev_world_clouds_.pop_front();
    }
    prev_world_clouds_.emplace_back(cur_world_cloud_ptr);
  }
  fuse_time_ = timer.toc(true);

  // shuffle points and cut off
  if (model_param_.preprocess().enable_shuffle_points()) {
    num_points =
        std::min(num_points, model_param_.preprocess().max_num_points());
    std::vector<int> point_indices = GenerateIndices(0, num_points, true);
    base::PointFCloudPtr shuffle_cloud_ptr(
        new base::PointFCloud(*cur_cloud_ptr_, point_indices));
    cur_cloud_ptr_ = shuffle_cloud_ptr;
  }
  shuffle_time_ = timer.toc(true);

  // points_array[x, y, z, i,timestampe, ......]
  std::vector<float> points_data(num_points *
                                 model_param_.preprocess().num_point_feature());

  CloudToArray(cur_cloud_ptr_, points_data.data(),
               model_param_.preprocess().normalizing_factor());
  cloud_to_array_time_ = timer.toc(true);

  auto input_data_blob = inference_->get_blob(input_blob_names_.at(0));
  auto output_bbox_blob = inference_->get_blob(output_blob_names_.at(0));
  auto output_score_blob = inference_->get_blob(output_blob_names_.at(1));
  auto output_label_blob = inference_->get_blob(output_blob_names_.at(2));

  std::vector<int> points_shape{num_points,
                                model_param_.preprocess().num_point_feature()};
  input_data_blob->Reshape(points_shape);
  float *data_ptr = input_data_blob->mutable_cpu_data();
  memcpy(data_ptr, points_data.data(), points_data.size() * sizeof(float));

  inference_->Infer();

  inference_time_ = timer.toc(true);

  // paddle inference
  std::vector<float> out_detections;
  std::vector<int64_t> out_labels;
  std::vector<float> out_scores;

  FilterScore(output_bbox_blob, output_label_blob, output_score_blob,
              model_param_.postprocess().score_threshold(), &out_detections,
              &out_labels, &out_scores);

  GetObjects(frame->lidar2world_pose, out_detections, out_labels, out_scores,
             &frame->segmented_objects);

  if (model_param_.filter_by_points()) {
    FilterObjectsbyPoints(&frame->segmented_objects);
  }

  FilterForegroundPoints(&frame->segmented_objects);

  std::stringstream sstr;
  sstr << "[CenterPointDetection BeforeNMS] "
       << std::to_string(frame->timestamp)
       << " objs: " << frame->segmented_objects.size() << std::endl;
  for (auto obj : frame->segmented_objects) {
      sstr << "id = " << obj->id << ": " << obj->center(0) << ", "
           << obj->center(1) << ", " << obj->center(2) << ", "
           << obj->size(0) << ", " << obj->size(1) << ", "
           << obj->size(2) << ", " << obj->theta << ", "
           << static_cast<int>(obj->type) << std::endl;
  }
  ADEBUG << sstr.str();

  PERF_BLOCK("class_nms")
  if (diff_class_nms_) {
      FilterObjectsbyClassNMS(&frame->segmented_objects);
  }
  PERF_BLOCK_END
  nms_time_ = timer.toc(true);

  SetPointsInROI(&frame->segmented_objects);

  postprocess_time_ = timer.toc(true);

  AINFO << "CenterPoint: "
        << "\n"
        << "down sample: " << downsample_time_ << "\t"
        << "fuse: " << fuse_time_ << "\t"
        << "shuffle: " << shuffle_time_ << "\t"
        << "cloud_to_array: " << cloud_to_array_time_ << "\t"
        << "inference: " << inference_time_ << "\t"
        << "postprocess: " << postprocess_time_ << "\t"
        << "nms: " << nms_time_ << "\t";

  std::stringstream ssstr;
  ssstr << "[CenterPointDetection AfterNMS] "
        << std::to_string(frame->timestamp)
        << " objs: " << frame->segmented_objects.size() << std::endl;
  for (auto obj : frame->segmented_objects) {
      ssstr << "id = " << obj->id << ": " << obj->center(0) << ", "
            << obj->center(1) << ", " << obj->center(2) << ", "
            << obj->size(0) << ", " << obj->size(1) << ", "
            << obj->size(2) << ", " << obj->theta << ", "
            << static_cast<int>(obj->type) << std::endl;
  }
  ADEBUG << ssstr.str();

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
    out_points_array[i * model_param_.preprocess().num_point_feature() + 0] = x;
    out_points_array[i * model_param_.preprocess().num_point_feature() + 1] = y;
    out_points_array[i * model_param_.preprocess().num_point_feature() + 2] = z;
    out_points_array[i * model_param_.preprocess().num_point_feature() + 3] =
        intensity / normalizing_factor;
    // delta of timestamp between prev and cur frames
    if (5 == model_param_.preprocess().num_point_feature()) {
      out_points_array[i * model_param_.preprocess().num_point_feature() + 4] =
          static_cast<float>(pc_ptr->points_timestamp(i));
    }
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

void CenterPointDetection::GetBoxCorner(int num_objects,
                                        const std::vector<float> &detections,
                                        std::vector<float> &box_corner,
                                        std::vector<float> &box_rectangular) {
  for (int i = 0; i < num_objects; ++i) {
    float x =
        detections[i * model_param_.postprocess().num_output_box_feature()];
    float y =
        detections[i * model_param_.postprocess().num_output_box_feature() + 1];
    float w =
        detections[i * model_param_.postprocess().num_output_box_feature() + 3];
    float l =
        detections[i * model_param_.postprocess().num_output_box_feature() + 4];
    float a =
        detections[i * model_param_.postprocess().num_output_box_feature() + 6];
    if (model_param_.quantize() > 0) {
      w = ceil(w / model_param_.quantize()) * model_param_.quantize();
      l = ceil(l / model_param_.quantize()) * model_param_.quantize();
    }
    if (model_param_.postprocess().width_enlarge_value() > 0) {
      w = w + model_param_.postprocess().width_enlarge_value();
    }
    if (model_param_.postprocess().length_enlarge_value() > 0) {
      l = l + model_param_.postprocess().length_enlarge_value();
    }

    float cos_a = cos(a);
    float sin_a = sin(a);
    float hw = w * 0.5;
    float hl = l * 0.5;

    float left_up_x = (-hw) * cos_a + (-hl) * sin_a + x;
    float left_up_y = (-hw) * (-sin_a) + (-hl) * cos_a + y;
    float right_up_x = (-hw) * cos_a + (hl)*sin_a + x;
    float right_up_y = (-hw) * (-sin_a) + (hl)*cos_a + y;
    float right_down_x = (hw)*cos_a + (hl)*sin_a + x;
    float right_down_y = (hw) * (-sin_a) + (hl)*cos_a + y;
    float left_down_x = (hw)*cos_a + (-hl) * sin_a + x;
    float left_down_y = (hw) * (-sin_a) + (-hl) * cos_a + y;

    box_corner[i * 8 + 0] = left_up_x;
    box_corner[i * 8 + 1] = left_up_y;
    box_corner[i * 8 + 2] = right_up_x;
    box_corner[i * 8 + 3] = right_up_y;
    box_corner[i * 8 + 4] = right_down_x;
    box_corner[i * 8 + 5] = right_down_y;
    box_corner[i * 8 + 6] = left_down_x;
    box_corner[i * 8 + 7] = left_down_y;

    box_rectangular[i * 4] = std::min(
        std::min(std::min(left_up_x, right_up_x), right_down_x), left_down_x);
    box_rectangular[i * 4 + 1] = std::min(
        std::min(std::min(left_up_y, right_up_y), right_down_y), left_down_y);
    box_rectangular[i * 4 + 2] = std::max(
        std::max(std::max(left_up_x, right_up_x), right_down_x), left_down_x);
    box_rectangular[i * 4 + 3] = std::max(
        std::max(std::max(left_up_y, right_up_y), right_down_y), left_down_y);
  }
}

void CenterPointDetection::GetBoxIndices(
    int num_objects, const std::vector<float> &detections,
    const std::vector<float> &box_corner,
    const std::vector<float> &box_rectangular,
    std::vector<std::shared_ptr<Object>> *objects) {
  for (size_t point_idx = 0; point_idx < original_cloud_->size(); ++point_idx) {
    const auto &point = original_cloud_->at(point_idx);
    float px = point.x;
    float py = point.y;
    float pz = point.z;

    int box_count = 0;

    for (int box_idx = 0; box_idx < num_objects; box_idx++) {
      if (box_count >= model_param_.point2box_max_num()) {
        break;
      }
      if (px < box_rectangular[box_idx * 4 + 0] ||
          px > box_rectangular[box_idx * 4 + 2]) {
        continue;
      }
      if (py < box_rectangular[box_idx * 4 + 1] ||
          py > box_rectangular[box_idx * 4 + 3]) {
        continue;
      }

      float z =
          detections[box_idx *
                         model_param_.postprocess().num_output_box_feature() +
                     2];
      float h =
          detections[box_idx *
                         model_param_.postprocess().num_output_box_feature() +
                     5];
      if (pz < (z - h / 2 -
                model_param_.postprocess().bottom_enlarge_height()) ||
          pz > (z + h / 2 + model_param_.postprocess().top_enlarge_height())) {
        continue;
      }

      float x1 = box_corner[box_idx * 8 + 0];
      float x2 = box_corner[box_idx * 8 + 2];
      float x3 = box_corner[box_idx * 8 + 4];
      float x4 = box_corner[box_idx * 8 + 6];
      float y1 = box_corner[box_idx * 8 + 1];
      float y2 = box_corner[box_idx * 8 + 3];
      float y3 = box_corner[box_idx * 8 + 5];
      float y4 = box_corner[box_idx * 8 + 7];

      // double angl1 = (px - x1) * (x2 - x1) + (py - y1) * (y2 - y1);
      // double angl2 = (px - x2) * (x3 - x2) + (py - y2) * (y3 - y2);
      // double angl3 = (px - x3) * (x4 - x3) + (py - y3) * (y4 - y3);
      // double angl4 = (px - x4) * (x1 - x4) + (py - y4) * (y1 - y4);

      double angl1 = (px - x1) * (y2 - y1) - (py - y1) * (x2 - x1);
      double angl2 = (px - x2) * (y3 - y2) - (py - y2) * (x3 - x2);
      double angl3 = (px - x3) * (y4 - y3) - (py - y3) * (x4 - x3);
      double angl4 = (px - x4) * (y1 - y4) - (py - y4) * (x1 - x4);

      if ((angl1 <= 0 && angl2 <= 0 && angl3 <= 0 && angl4 <= 0) ||
          (angl1 >= 0 && angl2 >= 0 && angl3 >= 0 && angl4 >= 0)) {
        auto &object = objects->at(box_idx);
        const auto &world_point = original_world_cloud_->at(point_idx);
        object->lidar_supplement.point_ids.push_back(point_idx);
        object->lidar_supplement.cloud.push_back(point);
        object->lidar_supplement.cloud_world.push_back(world_point);
      }
    }
  }
}

void CenterPointDetection::GetObjects(
    const Eigen::Affine3d &pose, const std::vector<float> &detections,
    const std::vector<int64_t> &labels, const std::vector<float> &scores,
    std::vector<std::shared_ptr<Object>> *objects) {
  int num_objects =
      detections.size() / model_param_.postprocess().num_output_box_feature();

  objects->clear();
  base::ObjectPool::Instance().BatchGet(num_objects, objects);

  for (int i = 0; i < num_objects; ++i) {
    auto &object = objects->at(i);
    object->id = i;

    // no velocity
    float x = detections.at(
        i * model_param_.postprocess().num_output_box_feature() + 0);
    float y = detections.at(
        i * model_param_.postprocess().num_output_box_feature() + 1);
    float z = detections.at(
        i * model_param_.postprocess().num_output_box_feature() + 2);
    float dx = detections.at(
        i * model_param_.postprocess().num_output_box_feature() + 4);
    float dy = detections.at(
        i * model_param_.postprocess().num_output_box_feature() + 3);
    float dz = detections.at(
        i * model_param_.postprocess().num_output_box_feature() + 5);
    float yaw = detections.at(
        i * model_param_.postprocess().num_output_box_feature() + 6);
    yaw = -yaw - M_PI / 2;

    object->size(0) = dx;
    object->size(1) = dy;
    object->size(2) = dz;
    object->center(0) = x;
    object->center(1) = y;
    object->center(2) = z;

    // directions
    object->theta = yaw;
    object->direction[0] = cosf(yaw);
    object->direction[1] = sinf(yaw);
    object->direction[2] = 0;
    object->lidar_supplement.is_orientation_ready = true;
    object->confidence = scores.at(i);

    // compute vertexes of bounding box and transform to world coordinate
    object->lidar_supplement.on_use = true;
    object->lidar_supplement.is_background = false;

    // classification
    object->lidar_supplement.raw_probs.push_back(std::vector<float>(
        static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
    object->lidar_supplement.raw_classification_methods.push_back(Name());
    object->sub_type = GetObjectSubType(labels.at(i));
    object->type = base::kSubType2TypeMap.at(object->sub_type);
    object->lidar_supplement.raw_probs.back()[static_cast<int>(object->type)] =
        1.0f;
    // copy to type
    object->type_probs.assign(object->lidar_supplement.raw_probs.back().begin(),
                              object->lidar_supplement.raw_probs.back().end());
  }

  std::vector<float> box_corner(num_objects * 8);
  std::vector<float> box_rectangular(num_objects * 4);
  GetBoxCorner(num_objects, detections, box_corner, box_rectangular);
  GetBoxIndices(num_objects, detections, box_corner, box_rectangular, objects);
  AINFO << "[CenterPoint] we get " << num_objects << " objs";
}

// for nuscenes model
// base::ObjectSubType CenterPointDetection::GetObjectSubType(const int label) {
//   switch (label) {
//     case 0:
//       return base::ObjectSubType::CAR;
//     case 1:
//       return base::ObjectSubType::TRUCK;
//     case 3:
//       return base::ObjectSubType::BUS;
//     case 6:
//       return base::ObjectSubType::MOTORCYCLIST;
//     case 7:
//       return base::ObjectSubType::CYCLIST;
//     case 8:
//       return base::ObjectSubType::PEDESTRIAN;
//     case 9:
//       return base::ObjectSubType::TRAFFICCONE;
//     default:
//       return base::ObjectSubType::UNKNOWN;
//   }
// }

// for kitti model
// base::ObjectSubType CenterPointDetection::GetObjectSubType(const int label) {
//   switch (label) {
//     case 0:
//       return base::ObjectSubType::CAR;
//     case 1:
//       return base::ObjectSubType::CYCLIST;
//     case 2:
//       return base::ObjectSubType::PEDESTRIAN;
//     default:
//       return base::ObjectSubType::UNKNOWN;
//   }
// }

// for apollo model
base::ObjectSubType CenterPointDetection::GetObjectSubType(const int label) {
  switch (label) {
    case 0:
      return base::ObjectSubType::SMALLMOT;
    case 1:
      return base::ObjectSubType::BIGMOT;
    case 2:
      return base::ObjectSubType::NONMOT;
    case 3:
      return base::ObjectSubType::PEDESTRIAN;
    default:
      return base::ObjectSubType::UNKNOWN;
  }
}

void CenterPointDetection::FilterScore(
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
      box3d_filtered->insert(
          box3d_filtered->end(),
          bbox_ptr + model_param_.postprocess().num_output_box_feature() * i,
          bbox_ptr +
              model_param_.postprocess().num_output_box_feature() * (i + 1));
      label_preds_filtered->insert(label_preds_filtered->end(),
                                   static_cast<int64_t>(label_ptr[i]));

      scores_filtered->push_back(score_ptr[i]);
    }
  }
}

void CenterPointDetection::FilterObjectsbyPoints(
    std::vector<std::shared_ptr<Object>> *objects) {
  size_t valid_num = 0;
  for (uint32_t i = 0; i < objects->size(); i++) {
    auto &object = objects->at(i);
    uint32_t num_points_thresholds = model_param_.min_points_threshold();
    if (object->lidar_supplement.cloud.size() >= num_points_thresholds) {
      objects->at(valid_num) = object;
      valid_num++;
    }
  }
  objects->resize(valid_num);
}

void CenterPointDetection::FilterForegroundPoints(
    std::vector<std::shared_ptr<Object>> *objects) {
  CloudMask mask;
  mask.Set(original_cloud_->size(), 0);
  mask.AddIndicesOfIndices(lidar_frame_ref_->roi_indices,
                           lidar_frame_ref_->non_ground_indices, 1);
  for (uint32_t i = 0; i < objects->size(); ++i) {
    auto &object = objects->at(i);
    base::PointIndices ind;
    ind.indices = object->lidar_supplement.point_ids;
    mask.RemoveIndices(ind);
  }
  mask.GetValidIndices(&lidar_frame_ref_->secondary_indices);
}

void CenterPointDetection::FilterObjectsbyClassNMS(
    std::vector<std::shared_ptr<base::Object>> *objects) {
    std::vector<bool> delete_array(objects->size(), false);
    // different class nms
    for (size_t i = 0; i < objects->size(); i++) {
        auto &obj_i = objects->at(i);
        if (!obj_i) {
            continue;
        }
        for (size_t j = i + 1; j < objects->size(); j++) {
            auto &obj_j = objects->at(j);
            if (!obj_j) {
                continue;
            }
            if (get_3dbox_iou(obj_i, obj_j, i, j) <= diff_class_iou_) {
                continue;
            }
            // should NMS
            if (nms_strategy_) {
                if (nms_by_strategy(obj_i, obj_j, nms_strategy_table_)) {
                    delete_array[j] = true;
                } else {
                    delete_array[i] = true;
                }
            } else {
                // reserve by score
                if (objects->at(i)->confidence > objects->at(j)->confidence) {
                    delete_array[j] = true;
                } else {
                    delete_array[i] = true;
                }
                // size_t index = (objects->at(i)->confidence
                // >= objects->at(j)->confidence ? i : j);
                // delete_array[index] = true;
            }
        }
    }
    size_t valid_num = 0;
    for (size_t i = 0; i < objects->size(); i++) {
        auto &object = objects->at(i);
        if (!delete_array[i]) {
            objects->at(valid_num) = object;
            valid_num++;
        }
    }
    objects->resize(valid_num);
}

void CenterPointDetection::SetPointsInROI(
    std::vector<std::shared_ptr<Object>> *objects) {
  CloudMask roi_mask_;
  roi_mask_.Set(original_cloud_->size(), 0);
  roi_mask_.AddIndices(lidar_frame_ref_->roi_indices, 1);
  for (uint32_t i = 0; i < objects->size(); ++i) {
    auto &object = objects->at(i);
    object->lidar_supplement.num_points_in_roi =
        roi_mask_.ValidIndicesCount(object->lidar_supplement.point_ids);
  }
}

PERCEPTION_REGISTER_LIDARDETECTOR(CenterPointDetection);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
