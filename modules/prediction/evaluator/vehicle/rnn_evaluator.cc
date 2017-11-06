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

#include "modules/prediction/evaluator/vehicle/rnn_evaluator.h"

#include <utility>
#include <vector>
#include <cmath>
#include <memory>
#include <string>
#include <mutex>

#include "modules/common/util/file.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_map.h"

namespace apollo {
namespace prediction {

using apollo::hdmap::LaneInfo;

RNNEvaluator::RNNEvaluator() { LoadModel(FLAGS_evaluator_vehicle_rnn_file); }

void RNNEvaluator::Evaluate(Obstacle* obstacle_ptr) {
  Clear();
  CHECK_NOTNULL(obstacle_ptr);

  int id = obstacle_ptr->id();
  if (!obstacle_ptr->latest_feature().IsInitialized()) {
    AERROR << "Obstacle [" << id << "] has no latest feature.";
    return;
  }

  Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
  CHECK_NOTNULL(latest_feature_ptr);
  if (!latest_feature_ptr->has_lane() ||
      !latest_feature_ptr->lane().has_lane_graph()) {
    ADEBUG << "Obstacle [" << id << "] has no lane graph.";
    return;
  }

  LaneGraph* lane_graph_ptr =
      latest_feature_ptr->mutable_lane()->mutable_lane_graph();
  CHECK_NOTNULL(lane_graph_ptr);
  if (lane_graph_ptr->lane_sequence_size() == 0) {
    AERROR << "Obstacle [" << id << "] has no lane sequences.";
    return;
  }

  Eigen::MatrixXf obstacle_feature_mat;
  std::unordered_map<int, Eigen::MatrixXf> lane_feature_mats;
  if (ExtractFeatureValues(obstacle_ptr,
          &obstacle_feature_mat, &lane_feature_mats) != 0) {
    AWARN << "Fail to extract feature from obstacle";
    return;
  }
  if (obstacle_feature_mat.rows() != 1 ||
      obstacle_feature_mat.size() != dim_obstacle_feature_) {
    AWARN << "Dim of obstacle feature is wrong!";
    return;
  }

  Eigen::MatrixXf prob_mat;
  std::vector<Eigen::MatrixXf> states;
  if (!obstacle_ptr->rnn_enabled()) {
    obstacle_ptr->InitRNNStates();
  }
  obstacle_ptr->GetRNNStates(&states);

  for (int i = 0; i < lane_graph_ptr->lane_sequence_size(); ++i) {
    LaneSequence* lane_sequence_ptr = lane_graph_ptr->mutable_lane_sequence(i);
    int seq_id = lane_sequence_ptr->lane_sequence_id();
    if (lane_feature_mats.find(seq_id) != lane_feature_mats.end()) {
      ADEBUG << "Fail to access seq-" << seq_id << " feature!";
      continue;
    }
    const Eigen::MatrixXf& lane_feature_mat = lane_feature_mats[seq_id];
    if (lane_feature_mat.cols() != dim_lane_point_feature_) {
      AWARN << "Lane feature dim of seq-" << seq_id << " is wrong!";
      continue;
    }
    // TODO(all) solve the following
    model_ptr_->SetState(states);
    model_ptr_->Run({obstacle_feature_mat, lane_feature_mat}, &prob_mat);
    if (std::isnan(prob_mat(0, 0)) || std::isinf(prob_mat(0, 0))) {
      AWARN << "Fail to compute probability.";
      continue;
    }
    lane_sequence_ptr->set_probability(prob_mat(0, 0));
  }
  model_ptr_->State(&states);
  obstacle_ptr->SetRNNStates(states);
}

void RNNEvaluator::Clear() {}

void RNNEvaluator::LoadModel(const std::string& model_file) {
  NetParameter net_parameter;
  CHECK(common::util::GetProtoFromFile(model_file, &net_parameter))
      << "Unable to load model file: " << model_file << ".";

  AINFO << "Succeeded in loading the model file: " << model_file << ".";
  model_ptr_ = network::RnnModel::instance();
  model_ptr_->LoadModel(net_parameter);
}

int RNNEvaluator::ExtractFeatureValues(
    Obstacle* obstacle, Eigen::MatrixXf* const obstacle_feature_mat,
    std::unordered_map<int, Eigen::MatrixXf>* const lane_feature_mats) {
  std::vector<float> obstacle_features;
  std::vector<float> lane_features;
  if (SetupObstacleFeature(obstacle, &obstacle_features) != 0) {
    ADEBUG << "Reset rnn state";
    obstacle->InitRNNStates();
  }
  if (static_cast<int>(obstacle_features.size()) != dim_obstacle_feature_) {
    AERROR << "Fail to setup obstacle feature!";
    return -1;
  }
  obstacle_feature_mat->resize(1, obstacle_features.size());
  for (size_t i = 0; i < obstacle_features.size(); ++i) {
    (*obstacle_feature_mat)(0, i) = obstacle_features[i];
  }

  Feature* feature = obstacle->mutable_latest_feature();
  if (!feature->has_lane() || !feature->lane().has_lane_graph()) {
    AERROR << "Fail to access lane graph!";
    return -1;
  }
  int routes = feature->lane().lane_graph().lane_sequence_size();
  for (int i = 0; i < routes; ++i) {
    LaneSequence lane_seq = feature->lane().lane_graph().lane_sequence(i);
    int seq_id = lane_seq.lane_sequence_id();
    if (SetupLaneFeature(*feature, lane_seq, &lane_features) != 0) {
      AWARN << "Fail to setup lane sequence feature!";
      continue;
    }
    int dim = dim_lane_point_feature_;
    Eigen::MatrixXf mat(lane_features.size() / dim, dim);
    for (int j = 0; j < static_cast<int>(lane_features.size()); ++j) {
      mat(j / dim, j % dim) = lane_features[j];
    }
    (*lane_feature_mats)[seq_id] = std::move(mat);
  }
  return 0;
}

int RNNEvaluator::SetupObstacleFeature(
    Obstacle* obstacle, std::vector<float>* const feature_values) {
  feature_values->clear();
  feature_values->reserve(dim_obstacle_feature_);

  float heading = 0.0;
  float speed = 0.0;
  float lane_l = 0.0;
  float theta = 0.0;
  float dist_lb = 1.0;
  float dist_rb = 1.0;
  int lane_type = 3;  // NO_TURN TODO(all) double check
  if (obstacle->history_size() < 1) {
    AWARN << "Size of feature less than 1!";
    return -1;
  }

  bool success_setup = false;
  int ret = 0;
  size_t num = obstacle->history_size() > 3 ? 3 : obstacle->history_size();
  for (size_t i = 0; i < num; ++i) {
    Feature* fea = obstacle->mutable_feature(i);
    if (fea == nullptr) {
      ADEBUG << "Fail to get " << i << "-th feature from obstacle";
      continue;
    }
    if (!fea->has_lane() || !fea->lane().has_lane_feature() ||
        !fea->lane().lane_feature().has_lane_id()) {
      ADEBUG << "Fail to access lane feature from " << i << "-the feature";
      continue;
    }
    LaneFeature* p_lane_fea = fea->mutable_lane()->mutable_lane_feature();
    lane_l = p_lane_fea->lane_l();
    theta = p_lane_fea->angle_diff();
    dist_lb = p_lane_fea->dist_to_left_boundary();
    dist_rb = p_lane_fea->dist_to_right_boundary();
    lane_type = p_lane_fea->lane_turn_type();

    if (!fea->has_speed() || !fea->has_theta()) {
      ADEBUG << "Fail to access speed from " << i << "-the feature";
      continue;
    }
    speed = fea->speed();
    heading = fea->theta();
    success_setup = true;
    ADEBUG << "Success to setup obstacle feature!";

    Feature* fea_pre = nullptr;
    if (i + 1 < obstacle->history_size()) {
      fea_pre = obstacle->mutable_feature(i + 1);
    }
    if (fea_pre != nullptr) {
      if (fea_pre->lane().has_lane_feature() &&
          fea_pre->lane().lane_feature().has_lane_id()) {
        std::string lane_id_pre = fea_pre->lane().lane_feature().lane_id();
        if (lane_id_pre != p_lane_fea->lane_id() &&
            IsCutinInHistory(p_lane_fea->lane_id(), lane_id_pre)) {
          ADEBUG << "Obstacle [" << fea->id() << "] cut in from " << lane_id_pre
                 << " to " << p_lane_fea->lane_id() << ", reset";
          ret = -1;
        }
      }
    }
    break;
  }

  if (!success_setup) {
    return -1;
  }
  feature_values->push_back(heading);
  feature_values->push_back(speed);
  feature_values->push_back(lane_l);
  feature_values->push_back(dist_lb);
  feature_values->push_back(dist_rb);
  feature_values->push_back(theta);
  feature_values->push_back(lane_type == 0 ? 1.0 : 0.0);
  feature_values->push_back(lane_type == 1 ? 1.0 : 0.0);
  feature_values->push_back(lane_type == 2 ? 1.0 : 0.0);
  feature_values->push_back(lane_type == 3 ? 1.0 : 0.0);
  return ret;
}

int RNNEvaluator::SetupLaneFeature(const Feature& feature,
                                   const LaneSequence& lane_sequence,
                                   std::vector<float>* const feature_values) {
  feature_values->clear();
  feature_values->reserve(
      dim_lane_point_feature_ * length_lane_point_sequence_);
  LanePoint* p_lane_point = nullptr;
  int counter = 0;
  for (int seg_i = 0; seg_i < lane_sequence.lane_segment_size(); ++seg_i) {
    if (counter > length_lane_point_sequence_) {
      break;
    }
    LaneSegment lane_seg = lane_sequence.lane_segment(seg_i);
    for (int pt_i = 0; pt_i < lane_seg.lane_point_size(); ++pt_i) {
      p_lane_point = lane_seg.mutable_lane_point(pt_i);
      if (p_lane_point->has_relative_s() &&
          p_lane_point->relative_s() < FLAGS_rnn_min_lane_relatice_s) {
        continue;
      }
      if (!feature.has_position() || !p_lane_point->has_position()) {
        ADEBUG << "Feature or lane_point has no position!";
        continue;
      }
      float diff_x = p_lane_point->position().x() - feature.position().x();
      float diff_y = p_lane_point->position().y() - feature.position().y();
      float angle = std::atan2(diff_y, diff_x);
      feature_values->push_back(p_lane_point->heading());
      feature_values->push_back(p_lane_point->angle_diff());
      feature_values->push_back(
          p_lane_point->relative_l() - feature.lane().lane_feature().lane_l());
      feature_values->push_back(angle);
      ++counter;
      if (counter > length_lane_point_sequence_) {
        ADEBUG << "Full the lane point sequence";
        break;
      }
    }
  }
  ADEBUG << "Lane sequence feature size: " << counter;
  if (counter < dim_lane_point_feature_) {
    ADEBUG << "Fail to setup lane feature!";
    return -1;
  }
  return 0;
}

bool RNNEvaluator::IsCutinInHistory(const std::string& curr_lane_id,
                                    const std::string& prev_lane_id) {
  PredictionMap* p_map = PredictionMap::instance();
  std::shared_ptr<const LaneInfo> curr_lane_info =
      p_map->LaneById(curr_lane_id);
  std::shared_ptr<const LaneInfo> prev_lane_info =
      p_map->LaneById(prev_lane_id);
  if (!p_map->IsSuccessorLane(curr_lane_info, prev_lane_info)) {
    return true;
  }
  return false;
}

}  // namespace prediction
}  // namespace apollo
