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

#include "modules/perception/pointcloud_ground_detection/ground_detector/spatio_temporal_ground_detector/spatio_temporal_ground_detector.h"

#include "cyber/common/file.h"

#include "modules/perception/common/util.h"
#include "modules/perception/common/algorithm/point_cloud_processing/common.h"
#include "modules/perception/common/lidar/common/lidar_log.h"
#include "modules/perception/common/lidar/common/lidar_point_label.h"
#include "modules/perception/common/algorithm/geometry/basic.h"
#include "modules/perception/pointcloud_ground_detection/ground_detector/proto/spatio_temporal_ground_detector_config.pb.h"

#include "modules/perception/common/algorithm/i_lib/geometry/i_plane.h"
#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::cyber::common::GetProtoFromFile;

bool SpatioTemporalGroundDetector::Init(
    const GroundDetectorInitOptions& options) {
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);

  // get config params
  SpatioTemporalGroundDetectorConfig config_params;
  ACHECK(GetProtoFromFile(config_file, &config_params))
      << "Failed to parse SpatioTemporalGroundDetectorConfig config file.";
  ground_thres_ = config_params.ground_thres();
  use_roi_ = config_params.use_roi();
  use_semantic_ground_ = config_params.use_semantic_ground();
  single_ground_detect_ = config_params.single_ground_detect();
  use_ground_service_ = config_params.use_ground_service();
  near_range_dist_ = config_params.near_range_dist();
  near_range_ground_thres_ = config_params.near_range_ground_thres();
  middle_range_dist_ = config_params.middle_range_dist();
  middle_range_ground_thres_ = config_params.middle_range_ground_thres();
  parsing_height_buffer_ = config_params.parsing_height_buffer();
  debug_output_ = config_params.debug_output();

  param_ = new algorithm::PlaneFitGroundDetectorParam;
  param_->roi_region_rad_x = config_params.roi_rad_x();
  param_->roi_region_rad_y = config_params.roi_rad_y();
  param_->roi_region_rad_z = config_params.roi_rad_z();
  param_->nr_grids_coarse = config_params.small_grid_size();
  param_->nr_grids_fine = config_params.big_grid_size();
  grid_size_ = config_params.small_grid_size();

  param_->roi_near_rad = config_params.near_range();
  near_range_ = config_params.near_range();
  param_->sample_region_z_lower = config_params.near_z_min();
  ori_sample_z_lower_ = config_params.near_z_min();
  param_->sample_region_z_upper = config_params.near_z_max();
  ori_sample_z_upper_ = config_params.near_z_max();
  param_->planefit_filter_threshold = config_params.z_compare_thres();
  param_->planefit_orien_threshold = config_params.planefit_orien_threshold();
  param_->planefit_dist_threshold_near =
      config_params.planefit_dist_thres_near();
  param_->planefit_dist_threshold_far = config_params.planefit_dist_thres_far();
  param_->candidate_filter_threshold = config_params.smooth_z_thres();
  param_->nr_inliers_min_threshold = config_params.inliers_min_threshold();
  param_->nr_smooth_iter = config_params.nr_smooth_iter();
  param_->use_semantic_info = config_params.use_semantic_ground();
  param_->use_math_optimize = config_params.use_math_optimize();
  param_->debug_output = config_params.debug_output();
  param_->single_frame_detect = config_params.single_ground_detect();
  param_->semantic_ground_value = static_cast<int>(PointSemanticLabel::GROUND);

  pfdetector_ = new algorithm::PlaneFitGroundDetector(*param_);
  pfdetector_->Init();

  point_attribute_.resize(default_point_size_);
  point_indices_temp_.resize(default_point_size_);
  data_.resize(default_point_size_ * 3);
  semantic_data_.resize(default_point_size_);
  ground_height_signed_.resize(default_point_size_);

  ground_service_content_.Init(
      config_params.roi_rad_x(), config_params.roi_rad_y(),
      config_params.small_grid_size(), config_params.small_grid_size());
  return true;
}

bool SpatioTemporalGroundDetector::Detect(const GroundDetectorOptions& options,
                                          LidarFrame* frame) {
  // check input
  if (frame == nullptr) {
    AERROR << "Input null frame ptr.";
    return false;
  }
  if (frame->cloud.get() == nullptr || frame->world_cloud.get() == nullptr) {
    AERROR << "Input null frame cloud.";
    return false;
  }
  if (frame->cloud->empty() || frame->world_cloud->empty()) {
    AERROR << "Input none points.";
    return false;
  }

  unsigned int data_id = 0;
  unsigned int semantic_data_id = 0;
  unsigned int valid_point_num = 0;
  unsigned int valid_point_num_cur = 0;
  size_t i = 0;
  size_t num_points = 0;
  size_t num_points_all = 0;
  int index = 0;
  unsigned int nr_points_element = 3;
  float z_dis = 0.0f;

  cloud_center_(0) = frame->lidar2world_pose(0, 3);
  cloud_center_(1) = frame->lidar2world_pose(1, 3);
  cloud_center_(2) = frame->lidar2world_pose(2, 3);

  // check output
  frame->non_ground_indices.indices.clear();

  if (frame->roi_indices.indices.empty()) {
    use_roi_ = false;
  }

  if (use_roi_) {
    num_points = frame->roi_indices.indices.size();
  } else {
    num_points = frame->world_cloud->size();
  }

  // reallocate memory if points num > the preallocated size
  num_points_all = num_points;

  if (num_points_all > default_point_size_) {
    default_point_size_ = num_points * 2;
    point_attribute_.resize(default_point_size_);
    point_indices_temp_.resize(default_point_size_);
    data_.resize(default_point_size_ * 3);
    semantic_data_.resize(default_point_size_);
    ground_height_signed_.resize(default_point_size_);
  }

  // copy point data, filtering lower points under ground
  // and use semantic filter
  // compute near-by semantic-ground average value for non-semantic data
  size_t near_ground_count = 0;
  float near_ground_z = 0.0;
  if (use_roi_) {
    for (i = 0; i < num_points; ++i) {
      index = frame->roi_indices.indices[i];
      // if (use_semantic_ground_ && static_cast<PointSemanticLabel>(
      //         frame->cloud->points_semantic_label(index) & 15) !=
      //               PointSemanticLabel::GROUND) {
      //     point_attribute_[i] = std::make_pair(index, -1);
      //     continue;
      // }
      point_attribute_[i] = std::make_pair(index, valid_point_num);
      point_indices_temp_[valid_point_num++] = index;
      bool ground_flag = IsSemanticLabelEqual(PointSemanticLabel::GROUND,
          frame->cloud->points_semantic_label(index));
      if (single_ground_detect_) {
          const auto& pt = frame->cloud->at(index);
          data_[data_id++] = static_cast<float>(pt.x);
          data_[data_id++] = static_cast<float>(pt.y);
          data_[data_id++] = static_cast<float>(pt.z);
          semantic_data_[semantic_data_id++] = static_cast<int>(
              GetSemanticLabel(frame->cloud->points_semantic_label(index)));
          if (ground_flag &&
              fabs(pt.x) <= near_range_ && fabs(pt.y) <= near_range_) {
              near_ground_count++;
              near_ground_z += pt.z;
          }
      } else {
          const auto& pt = frame->world_cloud->at(index);
          data_[data_id++] = static_cast<float>(pt.x - cloud_center_(0));
          data_[data_id++] = static_cast<float>(pt.y - cloud_center_(1));
          data_[data_id++] = static_cast<float>(pt.z - cloud_center_(2));
          semantic_data_[semantic_data_id++] = static_cast<int>(
              GetSemanticLabel(frame->cloud->points_semantic_label(index)));
          if (ground_flag &&
              fabs(pt.x - cloud_center_(0)) <= near_range_ &&
              fabs(pt.y - cloud_center_(1)) <= near_range_) {
              near_ground_count++;
              near_ground_z += pt.z;
          }
      }
    }
  } else {
    for (i = 0; i < num_points; ++i) {
      // if (use_semantic_ground_ && static_cast<PointSemanticLabel>(
      //         frame->cloud->points_semantic_label(i) & 15) !=
      //             PointSemanticLabel::GROUND) {
      //     point_attribute_[i] = std::make_pair(i, -1);
      //     continue;
      // }
      point_attribute_[i] = std::make_pair(i, valid_point_num);
      point_indices_temp_[valid_point_num++] = static_cast<int>(i);
      bool ground_flag = IsSemanticLabelEqual(PointSemanticLabel::GROUND,
          frame->cloud->points_semantic_label(i));
      if (single_ground_detect_) {
          const auto& pt = frame->cloud->at(i);
          data_[data_id++] = static_cast<float>(pt.x);
          data_[data_id++] = static_cast<float>(pt.y);
          data_[data_id++] = static_cast<float>(pt.z);
          semantic_data_[semantic_data_id++] = static_cast<int>(
              GetSemanticLabel(frame->cloud->points_semantic_label(index)));
          if (ground_flag &&
              fabs(pt.x) <= near_range_ && fabs(pt.y) <= near_range_) {
              near_ground_count++;
              near_ground_z += pt.z;
          }
      } else {
          const auto& pt = frame->world_cloud->at(i);
          data_[data_id++] = static_cast<float>(pt.x - cloud_center_(0));
          data_[data_id++] = static_cast<float>(pt.y - cloud_center_(1));
          data_[data_id++] = static_cast<float>(pt.z - cloud_center_(2));
          semantic_data_[semantic_data_id++] = static_cast<int>(
              GetSemanticLabel(frame->cloud->points_semantic_label(index)));
          if (ground_flag &&
              fabs(pt.x - cloud_center_(0)) <= near_range_ &&
              fabs(pt.y - cloud_center_(1)) <= near_range_) {
              near_ground_count++;
              near_ground_z += pt.z;
          }
      }
    }
  }

  CHECK_GE(num_points, valid_point_num);
  CHECK_EQ(data_id, valid_point_num * 3);
  CHECK_EQ(semantic_data_id, valid_point_num);
  base::PointIndices& non_ground_indices = frame->non_ground_indices;

  AINFO << "spatial temporal seg: use roi " << use_roi_ << " roi points "
        << num_points << " and input of ground detector: " << valid_point_num;

  pfdetector_->ResetParams(ori_sample_z_lower_, ori_sample_z_upper_);
  if (use_semantic_ground_) {
      if (near_ground_count > 0) {
          pfdetector_->UpdateParams(near_ground_z * 1.0f / near_ground_count,
              parsing_height_buffer_, frame->timestamp);
      } else {
          pfdetector_->UpdateParams(frame->parsing_ground_height,
              parsing_height_buffer_, frame->timestamp);
      }
  }
  if (!pfdetector_->Detect(data_.data(),  semantic_data_.data(),
    ground_height_signed_.data(), valid_point_num, nr_points_element)) {
    ADEBUG << "failed to call ground detector!";
    non_ground_indices.indices.insert(
        non_ground_indices.indices.end(), point_indices_temp_.begin(),
        point_indices_temp_.begin() + valid_point_num);
    return false;
  }

  // valid_point_num_cur = valid_point_num;
  // for (i = 0; i < valid_point_num_cur; ++i) {
  //   z_dis = ground_height_signed_.data()[i];
  //   frame->cloud->mutable_points_height()->at(point_indices_temp_[i]) =
  //       z_dis;
  //   frame->world_cloud->mutable_points_height()->at(point_indices_temp_[i])=
  //       z_dis;
  //   if (algorithm::IAbs(z_dis) > ground_thres_) {
  //     non_ground_indices.indices.push_back(i);
  //   } else {
  //     frame->cloud->mutable_points_label()->at(point_indices_temp_[i]) =
  //         static_cast<uint8_t>(LidarPointLabel::GROUND);
  //     frame->world_cloud->mutable_points_label()->at(point_indices_temp_[i])
  //           = static_cast<uint8_t>(LidarPointLabel::GROUND);
  //   }
  // }
  size_t ground_z_value_count = 0;
  float ground_z_value = 0.0;
  if (origin_ground_z_array_.size() >= ground_z_average_frame) {
      origin_ground_z_array_.pop_front();
  }
  auto valid_index = [&](int row, int col) {
      if (row >= 0 && row < grid_size_ && col >= 0 && col < grid_size_) {
        return true;
      }
      return false;
  };
  for (i = 0; i < num_points; ++i) {
      index = frame->roi_indices.indices[i];
      // in valid_num
      size_t pc_index = point_attribute_[i].first;
      size_t count = point_attribute_[i].second;

      float pc[3];
      if (single_ground_detect_) {
          const auto& pt = frame->cloud->at(pc_index);
          pc[0] = pt.x;
          pc[1] = pt.y;
          pc[2] = pt.z;
      } else {
          const auto& pt = frame->world_cloud->at(pc_index);
          pc[0] = pt.x - cloud_center_(0);
          pc[1] = pt.y - cloud_center_(1);
          pc[2] = pt.z - cloud_center_(2);
      }
      // impossible equal -1, but reserve temporarily
      // in case of other corner case
      if (count == -1) {
          int cur_row, cur_col = 0;
          std::vector<std::pair<int, int>> neighbors;
          pfdetector_->Pc2Voxel(pc[0], pc[1], pc[2], &cur_row, &cur_col);
          neighbors.push_back(std::make_pair(cur_row, cur_col));
          neighbors.push_back(std::make_pair(cur_row - 1, cur_col));
          neighbors.push_back(std::make_pair(cur_row + 1, cur_col));
          neighbors.push_back(std::make_pair(cur_row, cur_col - 1));
          neighbors.push_back(std::make_pair(cur_row, cur_col + 1));
          float min_z = std::numeric_limits<float>::max();
          for (size_t k = 0; k < neighbors.size(); k++) {
              int row = neighbors[k].first;
              int col = neighbors[k].second;
              if (!valid_index(row, col)) {
                  continue;
              }
              const algorithm::GroundPlaneLiDAR* plane =
                  pfdetector_->GetGroundPlane(row, col);
              if (plane->IsValid()) {
                  z_dis = algorithm::IPlaneToPointSignedDistanceWUnitNorm(
                      plane->params, pc);
                  min_z = std::fabs(z_dis) < min_z ? std::fabs(z_dis) : min_z;
              }
          }
          z_dis = min_z;
      } else {
          z_dis = ground_height_signed_.data()[count];
      }
      frame->cloud->mutable_points_height()->at(pc_index) = z_dis;
      frame->world_cloud->mutable_points_height()->at(pc_index) = z_dis;

      const auto& ppp = frame->cloud->at(pc_index);
      Eigen::Vector3d pc_novatel =
        frame->lidar2novatel_extrinsics * Eigen::Vector3d(ppp.x, ppp.y, ppp.z);
      float threshold = ground_thres_;
      if (pc_novatel(1) > 0 && pc_novatel(1) < near_range_dist_ &&
          pc_novatel(0) > FLAGS_x_back - 0.1 &&
          pc_novatel(0) < FLAGS_x_front + 0.1) {
          threshold = near_range_ground_thres_;
      }
      if (pc_novatel(1) >= near_range_dist_ &&
          pc_novatel(1) < middle_range_dist_ &&
          pc_novatel(0) > FLAGS_x_back - 0.1 &&
          pc_novatel(0) < FLAGS_x_front + 0.1) {
          threshold = middle_range_ground_thres_;
      }

      // if (algorithm::IAbs(z_dis) > threshold) {
      if (z_dis > threshold) {
          non_ground_indices.indices.push_back(i);
      } else {
          frame->cloud->mutable_points_label()->at(pc_index) =
              static_cast<uint8_t>(LidarPointLabel::GROUND);
          frame->world_cloud->mutable_points_label()->at(pc_index) =
              static_cast<uint8_t>(LidarPointLabel::GROUND);
          if (sqrt(ppp.x * ppp.x + ppp.y * ppp.y) < 8 && fabs(z_dis) < 0.1) {
              ++ground_z_value_count;
              ground_z_value += ppp.z;
          }
      }
  }
  AINFO << "succeed to call ground detector with non ground points "
        << non_ground_indices.indices.size();

  float oriHeight = ground_z_value /
      (static_cast<float>(ground_z_value_count) * 1.0f);
  origin_ground_z_array_.push_back(oriHeight);
  float ori_height = 0.0;
  for (auto value : origin_ground_z_array_) {
    ori_height += value;
  }
  frame->original_ground_z = ori_height /
      (static_cast<float>(origin_ground_z_array_.size()) * 1.0f);
  AINFO << "This frame " << std::to_string(frame->timestamp)
        <<" origin ground height is " << frame->original_ground_z;

  if (debug_output_) {
      std::ofstream out1;
      out1.open("semantic/" + std::to_string(frame->timestamp) + ".txt");
      if (out1.is_open()) {
          for (size_t i = 0; i < frame->cloud->size(); i++) {
              int parsing_index = static_cast<int>(GetSemanticLabel(
                  frame->cloud->points_semantic_label(i)));
              int sbr_index = static_cast<int>(GetMotionLabel(
                  frame->cloud->points_semantic_label(i)));
              int label_index = static_cast<uint8_t>(
                  frame->cloud->points_label(i));
              if (single_ground_detect_) {
                  const auto& pt = frame->cloud->at(i);
                  out1 << pt.x << ", " << pt.y << ", " << pt.z << ", "
                       << pt.intensity << ", " << parsing_index << ", "
                       << sbr_index << ", " << label_index << std::endl;
              } else {
                  const auto& pt = frame->world_cloud->at(i);
                  out1 << pt.x - cloud_center_(0) << ", "
                       << pt.y - cloud_center_(1) << ", "
                       << pt.z - cloud_center_(2) << ", " << pt.intensity
                       << ", " << parsing_index << ", " << sbr_index << ", "
                       << label_index << std::endl;
              }
          }
      }
      out1.close();
  }

  if (use_ground_service_) {
    auto ground_service = SceneManager::Instance().Service("GroundService");
    if (ground_service != nullptr) {
      ground_service_content_.grid_center_ = cloud_center_;
      ground_service_content_.grid_.Reset();
      GroundNode* node_ptr = ground_service_content_.grid_.DataPtr();
      unsigned int rows = pfdetector_->GetGridDimY();
      unsigned int cols = pfdetector_->GetGridDimX();
      unsigned int index = 0;
      for (unsigned int r = 0; r < rows; ++r) {
        for (unsigned int c = 0; c < cols; ++c) {
          const algorithm::GroundPlaneLiDAR* plane =
              pfdetector_->GetGroundPlane(r, c);
          if (plane->IsValid()) {
            index = r * cols + c;
            GroundNode* node = node_ptr + index;
            node->params(0) = plane->params[0];
            node->params(1) = plane->params[1];
            node->params(2) = plane->params[2];
            node->params(3) = plane->params[3];
            node->confidence = 1.0;
          }
        }
      }
      ground_service->UpdateServiceContent(ground_service_content_);
    } else {
      AINFO << "Failed to find ground service and cannot update.";
    }
  }
  return true;
}

PERCEPTION_REGISTER_GROUNDDETECTOR(SpatioTemporalGroundDetector);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
