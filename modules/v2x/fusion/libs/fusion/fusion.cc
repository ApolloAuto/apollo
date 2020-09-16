/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/v2x/fusion/libs/fusion/fusion.h"

#include <cmath>
#include <utility>

namespace apollo {
namespace v2x {
namespace ft {

Fusion::Fusion() {
  ft_config_manager_ptr_ = FTConfigManager::Instance();

  score_params_ = ft_config_manager_ptr_->fusion_params_.params.score_params();
  switch (score_params_.confidence_level()) {
    case fusion::ConfidenceLevel::C90P:
      m_matched_dis_limit_ = std::sqrt(4.605);
      break;
    case fusion::ConfidenceLevel::C95P:
      m_matched_dis_limit_ = std::sqrt(5.991);
      break;
    case fusion::ConfidenceLevel::C975P:
      m_matched_dis_limit_ = std::sqrt(7.378);
      break;
    case fusion::ConfidenceLevel::C99P:
      m_matched_dis_limit_ = std::sqrt(9.210);
      break;
    default:
      break;
  }
}

bool Fusion::Init() {
  last_timestamp_ = -1.0;
  return true;
}

bool Fusion::Proc(
    const std::vector<std::vector<base::Object>> &input_objectlists,
    double timestamp) {
  fusion_result_.clear();
  updated_objects_.clear();
  for (unsigned int i = 0; i < input_objectlists.size(); ++i) {
    CombineNewResource(input_objectlists[i]);
  }
  return true;
}

bool Fusion::CombineNewResource(const std::vector<base::Object> &new_objects) {
  return CombineNewResource(new_objects, &updated_objects_, &fusion_result_);
}

bool Fusion::CombineNewResource(
    const std::vector<base::Object> &new_objects,
    std::vector<base::Object> *fused_objects,
    std::vector<std::vector<base::Object>> *fusion_result) {
  if (new_objects.empty()) {
    return false;
  }
  if (fused_objects->size() < 1) {
    fused_objects->assign(new_objects.begin(), new_objects.end());
    for (unsigned int j = 0; j < new_objects.size(); ++j) {
      std::vector<base::Object> matched_objects;
      matched_objects.push_back(new_objects[j]);
      fusion_result->push_back(matched_objects);
    }
    return true;
  }
  int u_num = fused_objects->size();
  int v_num = new_objects.size();
  Eigen::MatrixXf association_mat(u_num, v_num);
  ComputeAssociateMatrix(*fused_objects, new_objects, &association_mat);
  std::vector<std::pair<int, int>> match_cps;
  if (u_num > v_num) {
    km_matcher_.GetKMResult(association_mat.transpose(), &match_cps, true);
  } else {
    km_matcher_.GetKMResult(association_mat, &match_cps, false);
  }
  for (auto it = match_cps.begin(); it != match_cps.end(); it++) {
    if (it->second != -1) {
      if (it->first == -1) {
        fused_objects->push_back(new_objects[it->second]);
        std::vector<base::Object> matched_objects;
        matched_objects.push_back(fused_objects->back());
        fusion_result->push_back(matched_objects);
      } else {
        (*fusion_result)[it->first].push_back(new_objects[it->second]);
      }
    }
  }
  return true;
}

bool Fusion::GetV2xFusionObjects(
    const std::vector<std::vector<base::Object>> &fusion_result,
    std::vector<base::Object> *fused_objects) {
  for (const auto &objects : fusion_result) {
    if (objects.size() == 1) {
      fused_objects->push_back(objects.at(0));
      if (objects.at(0).frame_id == "V2X") {
        fused_objects->back().v2x_type = base::V2xType::BLIND_ZONE;
      } else {
        fused_objects->back().v2x_type = base::V2xType::UNKNOWN;
      }
    } else {
      fused_objects->push_back(objects.at(0));
      host_vehicle_ = false;
      zom_vehicle_ = false;
      for (const auto &object : objects) {
        if (object.v2x_type == base::V2xType::HOST_VEHICLE) {
          host_vehicle_ = true;
        } else if (object.v2x_type == base::V2xType::ZOMBIES_CAR) {
          zom_vehicle_ = true;
        }
      }
      if (zom_vehicle_ == true) {
        fused_objects->back().v2x_type = base::V2xType::ZOMBIES_CAR;
      }
      if (host_vehicle_ == true) {
        fused_objects->back().v2x_type = base::V2xType::HOST_VEHICLE;
      }
    }
  }
  return true;
}

double Fusion::CheckOdistance(const base::Object &in1_ptr,
                              const base::Object &in2_ptr) {
  double xi = in1_ptr.position.x();
  double yi = in1_ptr.position.y();
  double xj = in2_ptr.position.x();
  double yj = in2_ptr.position.y();
  double distance = std::hypot(xi - xj, yi - yj);
  return distance;
}

bool Fusion::CheckDisScore(const base::Object &in1_ptr,
                           const base::Object &in2_ptr, double *score) {
  double dis = CheckOdistance(in1_ptr, in2_ptr);
  *score = 2.5 * std::max(0.0, score_params_.max_match_distance() - dis);
  return true;
}

bool Fusion::CheckTypeScore(const base::Object &in1_ptr,
                            const base::Object &in2_ptr, double *score) {
  double same_prob = 0;
  if (in1_ptr.sub_type == in2_ptr.sub_type) {
    same_prob =
        1 - (1 - in1_ptr.sub_type_probs[0]) * (1 - in2_ptr.sub_type_probs[0]);
  } else if (in1_ptr.type == in2_ptr.type ||
             in1_ptr.type == v2x::ft::base::ObjectType::UNKNOWN ||
             in2_ptr.type == v2x::ft::base::ObjectType::UNKNOWN) {
    same_prob =
        (1 - in1_ptr.sub_type_probs.at(0)) * in2_ptr.sub_type_probs.at(0) +
        (1 - in2_ptr.sub_type_probs.at(0)) * in1_ptr.sub_type_probs.at(0);
    same_prob *= score_params_.prob_scale();
  }
  *score *= same_prob;
  return true;
}

bool Fusion::ComputeAssociateMatrix(
    const std::vector<base::Object> &in1_objects,  // fused
    const std::vector<base::Object> &in2_objects,  // new
    Eigen::MatrixXf *association_mat) {
  for (unsigned int i = 0; i < in1_objects.size(); ++i) {
    for (unsigned int j = 0; j < in2_objects.size(); ++j) {
      const base::Object &obj1_ptr = in1_objects[i];
      const base::Object &obj2_ptr = in2_objects[j];
      double score = 0;
      if (!CheckDisScore(obj1_ptr, obj2_ptr, &score)) {
        AERROR << "V2X Fusion: check dis score failed";
      }
      if (score_params_.check_type() &&
          !CheckTypeScore(obj1_ptr, obj2_ptr, &score)) {
        AERROR << "V2X Fusion: check type failed";
      }
      (*association_mat)(i, j) =
          (score >= score_params_.min_score()) ? score : 0;
    }
  }
  return true;
}

int Fusion::DeleteRedundant(std::vector<base::Object> *objects) {
  std::vector<unsigned int> to_be_deleted;
  for (unsigned int i = 0; i < objects->size(); ++i) {
    for (unsigned int j = i + 1; j < objects->size(); ++j) {
      double distance = CheckOdistance(objects->at(i), objects->at(j));
      if (distance < 1) {
        to_be_deleted.push_back(j);
      }
    }
  }
  for (auto iter = to_be_deleted.rbegin(); iter != to_be_deleted.rend();
       ++iter) {
    objects->erase(objects->begin() + *iter);
  }
  return to_be_deleted.size();
}
}  // namespace ft
}  // namespace v2x
}  // namespace apollo
