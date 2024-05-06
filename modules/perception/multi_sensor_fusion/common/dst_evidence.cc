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
#include "modules/perception/multi_sensor_fusion/common/dst_evidence.h"

#include <algorithm>
#include <bitset>
#include <numeric>

#include <boost/format.hpp>

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace fusion {

bool DstManager::AddApp(const std::string &app_name,
                        const std::vector<uint64_t> &fod_subsets,
                        const std::vector<std::string> &fod_subset_names) {
  if (dst_common_data_.find(app_name) != dst_common_data_.end()) {
    AWARN << boost::format("Dst %s was added!") % app_name;
  }
  DstCommonData dst_data;
  dst_data.fod_subsets_ = fod_subsets;
  BuildSubsetsIndMap(&dst_data);
  if (dst_data.subsets_ind_map_.size() != dst_data.fod_subsets_.size()) {
    AERROR << boost::format(
                  "Dst %s: The input fod subsets"
                  " have repetitive elements.") %
                  app_name;
    return false;
  }
  FodCheck(&dst_data);
  ComputeCardinalities(&dst_data);
  if (!ComputeRelations(&dst_data)) {
    return false;
  }
  dst_data.init_ = true;
  BuildNamesMap(fod_subset_names, &dst_data);

  std::lock_guard<std::mutex> lock(map_mutex_);
  dst_common_data_[app_name] = dst_data;
  return true;
}

bool DstManager::IsAppAdded(const std::string &app_name) {
  auto iter = dst_common_data_.find(app_name);
  if (iter == dst_common_data_.end()) {
    return false;
  }
  return iter->second.init_;
}

DstCommonDataPtr DstManager::GetAppDataPtr(const std::string &app_name) {
  if (!IsAppAdded(app_name)) {
    AERROR << "app_name is not available";
    return nullptr;
  }
  auto iter = dst_common_data_.find(app_name);
  if (iter != dst_common_data_.end()) {
    return &iter->second;
  }
  return nullptr;
}

size_t DstManager::FodSubsetToInd(const std::string &app_name,
                                  const uint64_t &fod_subset) {
  auto iter0 = dst_common_data_.find(app_name);
  ACHECK(iter0 != dst_common_data_.end());
  auto iter = iter0->second.subsets_ind_map_.find(fod_subset);
  ACHECK(iter != iter0->second.subsets_ind_map_.end());
  return iter->second;
}

uint64_t DstManager::IndToFodSubset(const std::string &app_name,
                                    const size_t &ind) {
  auto iter = dst_common_data_.find(app_name);
  ACHECK(iter != dst_common_data_.end());
  return iter->second.fod_subsets_[ind];
}

void DstManager::BuildSubsetsIndMap(DstCommonData *dst_data) {
  dst_data->subsets_ind_map_.clear();
  for (size_t i = 0; i < dst_data->fod_subsets_.size(); ++i) {
    dst_data->subsets_ind_map_[dst_data->fod_subsets_[i]] = i;
  }
}

void DstManager::FodCheck(DstCommonData *dst_data) {
  uint64_t fod = 0;
  for (auto fod_subset : dst_data->fod_subsets_) {
    fod |= fod_subset;
  }
  dst_data->fod_loc_ = dst_data->fod_subsets_.size();
  auto find_res = dst_data->subsets_ind_map_.insert(
      std::make_pair(fod, dst_data->fod_subsets_.size()));
  if (find_res.second) {
    dst_data->fod_subsets_.push_back(fod);
  } else {
    dst_data->fod_loc_ = find_res.first->second;
  }
}

void DstManager::ComputeCardinalities(DstCommonData *dst_data) {
  auto count_set_bits = [](uint64_t fod_subset) {
    size_t count = 0;
    while (fod_subset) {
      fod_subset &= (fod_subset - 1);
      ++count;
    }
    return count;
  };
  dst_data->fod_subset_cardinalities_.reserve(dst_data->fod_subsets_.size());
  for (auto fod_subset : dst_data->fod_subsets_) {
    dst_data->fod_subset_cardinalities_.push_back(count_set_bits(fod_subset));
  }
}

bool DstManager::ComputeRelations(DstCommonData *dst_data) {
  auto reserve_space = [](std::vector<std::vector<size_t>> &relations,
                          size_t size) {
    relations.clear();
    relations.resize(size);
    for (auto &relation : relations) {
      relation.reserve(size);
    }
  };
  reserve_space(dst_data->subset_relations_, dst_data->fod_subsets_.size());
  reserve_space(dst_data->inter_relations_, dst_data->fod_subsets_.size());
  // reserve space for combination_relations
  dst_data->combination_relations_.clear();
  dst_data->combination_relations_.resize(dst_data->fod_subsets_.size());
  for (auto &combination_relation : dst_data->combination_relations_) {
    combination_relation.reserve(2 * dst_data->fod_subsets_.size());
  }
  for (size_t i = 0; i < dst_data->fod_subsets_.size(); ++i) {
    uint64_t fod_subset = dst_data->fod_subsets_[i];
    auto &subset_inds = dst_data->subset_relations_[i];
    auto &inter_inds = dst_data->inter_relations_[i];

    for (size_t j = 0; j < dst_data->fod_subsets_.size(); ++j) {
      if ((fod_subset | dst_data->fod_subsets_[j]) == fod_subset) {
        subset_inds.push_back(j);
      }
      uint64_t inter_res = fod_subset & dst_data->fod_subsets_[j];
      if (inter_res) {
        inter_inds.push_back(j);
        auto find_res = dst_data->subsets_ind_map_.find(inter_res);
        if (find_res == dst_data->subsets_ind_map_.end()) {
          AERROR << boost::format(
              "Dst: The input set "
              "of fod subsets has no closure under the operation "
              "intersection");
          return false;
        }
        dst_data->combination_relations_[find_res->second].push_back(
            std::make_pair(i, j));
      }
    }
  }
  return true;
}

void DstManager::BuildNamesMap(const std::vector<std::string> &fod_subset_names,
                               DstCommonData *dst_data) {
  // reset and reserve space
  dst_data->fod_subset_names_.clear();
  dst_data->fod_subset_names_.resize(dst_data->fod_subsets_.size());
  for (size_t i = 0; i < dst_data->fod_subsets_.size(); ++i) {
    dst_data->fod_subset_names_[i] =
        std::bitset<64>(dst_data->fod_subsets_[i]).to_string();
  }
  // set fod to unknown
  dst_data->fod_subset_names_[dst_data->fod_loc_] = "unknown";
  for (size_t i = 0;
       i < std::min(fod_subset_names.size(), dst_data->fod_subsets_.size());
       ++i) {
    dst_data->fod_subset_names_[i] = fod_subset_names[i];
  }
}

Dst::Dst(const std::string &app_name) : app_name_(app_name) {
  if (DstManager::Instance()->IsAppAdded(app_name)) {
    dst_data_ptr_ = DstManager::Instance()->GetAppDataPtr(app_name);
    // default BBA provide no more evidence
    bba_vec_.resize(dst_data_ptr_->fod_subsets_.size(), 0.0);
    bba_vec_[dst_data_ptr_->fod_loc_] = 1.0;
  }
}

void Dst::SelfCheck() const {
  ACHECK(DstManager::Instance()->IsAppAdded(app_name_));
  if (dst_data_ptr_ == nullptr) {
    dst_data_ptr_ = DstManager::Instance()->GetAppDataPtr(app_name_);
    CHECK_NOTNULL(dst_data_ptr_);
    bba_vec_.resize(dst_data_ptr_->fod_subsets_.size(), 0.0);
    bba_vec_[dst_data_ptr_->fod_loc_] = 1.0;
  }
}

double Dst::GetSubsetBfmass(uint64_t fod_subset) const {
  SelfCheck();
  size_t idx = DstManager::Instance()->FodSubsetToInd(app_name_, fod_subset);
  return bba_vec_[idx];
}

double Dst::GetIndBfmass(size_t ind) const {
  SelfCheck();
  return bba_vec_[ind];
}

bool Dst::SetBbaVec(const std::vector<double> &bba_vec) {
  SelfCheck();
  if (bba_vec.size() != dst_data_ptr_->fod_subsets_.size()) {
    AERROR << boost::format("input bba_vec size: %d !=  Dst subsets size: %d") %
                  bba_vec.size() % dst_data_ptr_->fod_subsets_.size();
    return false;
  }
  // check belief mass valid
  for (auto belief_mass : bba_vec) {
    if (belief_mass < 0.0) {
      AWARN << boost::format(" belief mass: %lf is not valid") % belief_mass;
      return false;
    }
  }
  // reset
  // *this = Dst(app_name_);
  bba_vec_ = bba_vec;
  Normalize();
  return true;
}

bool Dst::SetBba(const std::map<uint64_t, double> &bba_map) {
  SelfCheck();
  std::vector<double> bba_vec(dst_data_ptr_->fod_subsets_.size(), 0.0);
  const auto &subsets_ind_map = dst_data_ptr_->subsets_ind_map_;
  for (auto bba_map_iter : bba_map) {
    uint64_t fod_subset = bba_map_iter.first;
    double belief_mass = bba_map_iter.second;
    auto find_res = subsets_ind_map.find(fod_subset);
    if (find_res == subsets_ind_map.end()) {
      AERROR << "the input bba map has invalid fod subset";
      return false;
    }
    if (belief_mass < 0.0) {
      AWARN << boost::format("belief mass: %lf is not valid. Dst name: %s") %
                   belief_mass % app_name_;
      return false;
    }
    bba_vec[find_res->second] = belief_mass;
  }
  // reset
  // *this = Dst(app_name_);
  bba_vec_ = bba_vec;
  Normalize();
  return true;
}

std::string Dst::PrintBba() const {
  SelfCheck();
  static constexpr size_t total_res_size = 10000;
  static constexpr size_t row_res_size = 1000;
  static auto print_row = [](const std::string &row_header,
                             const std::vector<double> &data) {
    std::string row_str = (boost::format("%19s ") % row_header).str();
    row_str.reserve(row_res_size);
    for (auto flt : data) {
      row_str += (boost::format("%20.6lf") % (flt * 100)).str();
    }
    row_str += "\n";
    return row_str;
  };
  std::string res;
  res.reserve(total_res_size);
  res += "\n";
  // output table header
  std::string header = (boost::format("%20s") % " ").str();
  header.reserve(row_res_size);
  const std::vector<std::string> &fod_subset_names =
      dst_data_ptr_->fod_subset_names_;
  for (const auto &fod_subset_name : fod_subset_names) {
    header += (boost::format("%20s") % fod_subset_name).str();
  }
  res += header + "\n";
  res += print_row("belief_mass", bba_vec_);
  // res += print_row("support", support_vec_);
  // res += print_row("uncertainty", uncertainty_vec_);
  // res += print_row("probability", probability_vec_);
  return res;
}

void Dst::ComputeSptPlsUct() const {
  SelfCheck();
  auto resize_space = [](std::vector<double> &vec, size_t size) {
    vec.clear();
    vec.resize(size, 0.0);
  };
  size_t size = bba_vec_.size();
  resize_space(support_vec_, size);
  resize_space(plausibility_vec_, size);
  resize_space(uncertainty_vec_, size);
  const std::vector<std::vector<size_t>> &subset_relations =
      dst_data_ptr_->subset_relations_;
  const std::vector<std::vector<size_t>> &inter_relations =
      dst_data_ptr_->inter_relations_;
  for (size_t i = 0; i < size; ++i) {
    double &spt = support_vec_[i];
    double &pls = plausibility_vec_[i];
    double &uct = uncertainty_vec_[i];
    const auto &subset_inds = subset_relations[i];
    const auto &inter_inds = inter_relations[i];
    // AINFO << boost::format("inter_size: (%d %d)") % i % inter_inds.size();
    for (auto subset_ind : subset_inds) {
      spt += bba_vec_[subset_ind];
    }
    for (auto inter_ind : inter_inds) {
      pls += bba_vec_[inter_ind];
    }
    // AINFO << boost::format("pls: (%d %lf)") % i % pls;
    uct = pls - spt;
  }
}

// use combination_relations to compute all the probability at one time
void Dst::ComputeProbability() const {
  SelfCheck();
  probability_vec_.clear();
  probability_vec_.resize(bba_vec_.size(), 0.0);
  const auto &combination_relations = dst_data_ptr_->combination_relations_;
  const std::vector<size_t> &fod_subset_cardinalities =
      dst_data_ptr_->fod_subset_cardinalities_;
  for (size_t i = 0; i < combination_relations.size(); ++i) {
    const auto &combination_pairs = combination_relations[i];
    double intersection_card = static_cast<double>(fod_subset_cardinalities[i]);
    for (auto combination_pair : combination_pairs) {
      size_t a_ind = combination_pair.first;
      size_t b_ind = combination_pair.second;
      probability_vec_[a_ind] +=
          intersection_card /
          static_cast<double>(fod_subset_cardinalities[b_ind]) *
          bba_vec_[b_ind];
    }
  }
}

void Dst::Normalize() {
  SelfCheck();
  double mass_sum = std::accumulate(bba_vec_.begin(), bba_vec_.end(), 0.0);
  if (mass_sum == 0.0) {
    ADEBUG << "mass_sum equal 0!!";
  }
  for (auto &belief_mass : bba_vec_) {
    belief_mass /= mass_sum;
  }
}

Dst operator+(const Dst &lhs, const Dst &rhs) {
  CHECK_EQ(lhs.app_name_, rhs.app_name_)
      << boost::format("lhs Dst(%s) is not equal to rhs Dst(%s)") %
             lhs.app_name_ % rhs.app_name_;
  lhs.SelfCheck();
  rhs.SelfCheck();
  Dst res(lhs.app_name_);
  std::vector<double> &resbba_vec_ = res.bba_vec_;
  const auto &combination_relations = lhs.dst_data_ptr_->combination_relations_;
  for (size_t i = 0; i < resbba_vec_.size(); ++i) {
    const auto &combination_pairs = combination_relations[i];
    // AINFO << "pairs size: " << combination_pairs.size();
    double &belief_mass = resbba_vec_[i];
    belief_mass = 0.0;
    for (auto combination_pair : combination_pairs) {
      // AINFO << boost::format("(%d %d)") % combination_pair.first
      //     % combination_pair.second;
      belief_mass += lhs.GetIndBfmass(combination_pair.first) *
                     rhs.GetIndBfmass(combination_pair.second);
    }
    // AINFO << boost::format("belief_mass: %lf") % belief_mass;
  }
  res.Normalize();
  return res;
}

Dst operator*(const Dst &dst, double w) {
  dst.SelfCheck();
  Dst res(dst.app_name_);
  // check w
  if (w < 0.0 || w > 1.0) {
    AERROR << boost::format(
                  "the weight of bba %lf is not valid, return default bba") %
                  w;
    return res;
  }
  size_t fod_loc = dst.dst_data_ptr_->fod_loc_;
  std::vector<double> &resbba_vec_ = res.bba_vec_;
  const std::vector<double> &bba_vec = dst.bba_vec_;
  for (size_t i = 0; i < resbba_vec_.size(); ++i) {
    if (i == fod_loc) {
      resbba_vec_[i] = 1.0 - w + w * bba_vec[i];
    } else {
      resbba_vec_[i] = w * bba_vec[i];
    }
  }
  return res;
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
