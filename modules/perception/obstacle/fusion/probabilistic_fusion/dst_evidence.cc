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

#include "modules/perception/obstacle/fusion/probabilistic_fusion/dst_evidence.h"

#include <algorithm>
#include <bitset>
#include <utility>
#include "boost/format.hpp"
#include "modules/common/log.h"

namespace apollo {
namespace perception {

// BBAManager
bool BBAManager::init(const std::vector<uint64_t>& fod_subsets,
                      const std::vector<std::string>& fod_subset_names) {
  if (_init) {
    AERROR << boost::format("BBAManager(%s) was initialized!") % _manager_name;
    return false;
  }
  _fod_subsets = fod_subsets;
  build_subsets_ind_map();
  if (_subsets_ind_map.size() != _fod_subsets.size()) {
    AERROR << boost::format(
                  "BBAManager(%s): The input fod subsets"
                  " have repetitive elements.") %
                  _manager_name;
    return false;
  }
  fod_check();
  compute_cardinalities();
  if (!compute_relations()) {
    return false;
  }
  _init = true;
  build_names_map(fod_subset_names);
  return true;
}
bool BBAManager::fod_subset_exsits(uint64_t fod_subset) const {
  if (_subsets_ind_map.find(fod_subset) == _subsets_ind_map.end()) {
    return false;
  }
  return true;
}
bool BBAManager::ind_exsits(size_t ind) const {
  return ind < _fod_subsets.size();
}
size_t BBAManager::fod_subset_to_ind(uint64_t fod_subset) const {
  auto iter = _subsets_ind_map.find(fod_subset);
  CHECK(iter != _subsets_ind_map.end());
  return iter->second;
}
uint64_t BBAManager::ind_to_fod_subset(size_t ind) const {
  return _fod_subsets[ind];
}
void BBAManager::build_subsets_ind_map() {
  _subsets_ind_map.clear();
  for (size_t i = 0; i < _fod_subsets.size(); ++i) {
    _subsets_ind_map[_fod_subsets[i]] = i;
  }
}
void BBAManager::fod_check() {
  uint64_t fod = 0;
  for (auto fod_subset : _fod_subsets) {
    fod |= fod_subset;
  }
  _fod_loc = _fod_subsets.size();
  auto find_res =
      _subsets_ind_map.insert(std::make_pair(fod, _fod_subsets.size()));
  if (find_res.second) {
    _fod_subsets.push_back(fod);
  } else {
    _fod_loc = find_res.first->second;
  }
}
void BBAManager::compute_cardinalities() {
  auto count_set_bits = [](uint64_t fod_subset) {
    size_t count = 0;
    while (fod_subset) {
      fod_subset &= (fod_subset - 1);
      ++count;
    }
    return count;
  };
  _fod_subset_cardinalities.reserve(_fod_subsets.size());
  for (auto fod_subset : _fod_subsets) {
    _fod_subset_cardinalities.push_back(count_set_bits(fod_subset));
  }
}
bool BBAManager::compute_relations() {
  auto reserve_space = [](std::vector<std::vector<size_t>>& relations,
                          size_t size) {
    relations.clear();
    relations.resize(size);
    for (auto& relation : relations) {
      relation.reserve(size);
    }
  };
  reserve_space(_subset_relations, _fod_subsets.size());
  reserve_space(_inter_relations, _fod_subsets.size());
  // reserve space for combination_relations
  _combination_relations.clear();
  _combination_relations.resize(_fod_subsets.size());
  for (auto& combination_relation : _combination_relations) {
    combination_relation.reserve(2 * _fod_subsets.size());
  }
  for (size_t i = 0; i < _fod_subsets.size(); ++i) {
    uint64_t fod_subset = _fod_subsets[i];
    auto& subset_inds = _subset_relations[i];
    auto& inter_inds = _inter_relations[i];

    for (size_t j = 0; j < _fod_subsets.size(); ++j) {
      if ((fod_subset | _fod_subsets[j]) == fod_subset) {
        subset_inds.push_back(j);
      }
      uint64_t inter_res = fod_subset & _fod_subsets[j];
      if (inter_res) {
        inter_inds.push_back(j);
        auto find_res = _subsets_ind_map.find(inter_res);
        if (find_res == _subsets_ind_map.end()) {
          AERROR << boost::format(
                        "BBAManager(%s): The input set "
                        "of fod subsets has no closure under the operation "
                        "intersection") %
                        _manager_name;
          return false;
        }
        _combination_relations[find_res->second].push_back(
            std::make_pair(i, j));
      }
    }
  }
  return true;
}
void BBAManager::build_names_map(
    const std::vector<std::string>& fod_subset_names) {
  // reset and reserve space
  _fod_subset_names.clear();
  _fod_subset_names.resize(_fod_subsets.size());
  for (size_t i = 0; i < _fod_subsets.size(); ++i) {
    _fod_subset_names[i] = std::bitset<64>(_fod_subsets[i]).to_string();
  }
  // set fod to unkown
  _fod_subset_names[_fod_loc] = "unkown";
  for (size_t i = 0; i < std::min(fod_subset_names.size(), _fod_subsets.size());
       ++i) {
    _fod_subset_names[i] = fod_subset_names[i];
  }
}
// BBA
BBA::BBA(const BBAManager* bba_manager_ptr) {
  _bba_manager_ptr = bba_manager_ptr;
  if (_bba_manager_ptr == nullptr) {
    _bba_manager_ptr = &BBAManager::instance();
  }
  if (!_bba_manager_ptr->initialized()) {
    // LOG_WARN << "create a BBA before BBAManager initialization";
    return;
  }
  // default BBA provide no more evidence
  _bba_vec.resize(_bba_manager_ptr->get_subsets_size(), 0.0);
  _bba_vec[_bba_manager_ptr->get_fod_ind()] = 1.0;
}
double BBA::get_subset_bfmass(uint64_t fod_subset) const {
  return _bba_vec[_bba_manager_ptr->fod_subset_to_ind(fod_subset)];
}
double BBA::get_ind_bfmass(size_t ind) const { return _bba_vec[ind]; }
bool BBA::set_bba_vec(const std::vector<double>& bba_vec) {
  if (bba_vec.size() != _bba_manager_ptr->get_subsets_size()) {
    AERROR << boost::format(
                  "input bba_vec size: %d !=  bba_manager subsets size: %d") %
                  bba_vec.size() % _bba_manager_ptr->get_subsets_size();
    return false;
  }
  // check belief mass valid
  for (auto belief_mass : bba_vec) {
    if (belief_mass < 0.0) {
      AERROR << boost::format("belief mass: %lf is not valid") % belief_mass;
      return false;
    }
  }
  // reset
  *this = BBA(_bba_manager_ptr);
  _bba_vec = bba_vec;
  normalize();
  return true;
}
bool BBA::set_bba(const std::map<uint64_t, double>& bba_map) {
  std::vector<double> bba_vec(_bba_manager_ptr->get_subsets_size(), 0.0);
  const auto& subsets_ind_map = _bba_manager_ptr->get_subsets_ind_map();
  for (auto bba_map_iter : bba_map) {
    uint64_t fod_subset = bba_map_iter.first;
    double belief_mass = bba_map_iter.second;
    auto find_res = subsets_ind_map.find(fod_subset);
    if (find_res == subsets_ind_map.end()) {
      AERROR << "the input bba map has invalid fod subset";
      return false;
    }
    if (belief_mass < 0.0) {
      AERROR << boost::format(
                    "belief mass: %lf is not valid. Manager name: %s") %
                    belief_mass % _bba_manager_ptr->get_manager_name();
      return false;
    }
    bba_vec[find_res->second] = belief_mass;
  }
  // reset
  *this = BBA(_bba_manager_ptr);
  _bba_vec = bba_vec;
  normalize();
  return true;
}

std::string BBA::print_bba() const {
  static constexpr size_t total_res_size = 10000;
  static constexpr size_t row_res_size = 1000;
  static auto print_row = [](const std::string& row_header,
                             const std::vector<double>& data) {
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
  const std::vector<std::string>& fod_subset_names =
      _bba_manager_ptr->get_fod_subset_names();
  for (const auto& fod_subset_name : fod_subset_names) {
    header += (boost::format("%20s") % fod_subset_name).str();
  }
  res = res + header + "\n";
  res += print_row("belief_mass", _bba_vec);
  res += print_row("support", _support_vec);
  res += print_row("uncertainty", _uncertainty_vec);
  res += print_row("probability", _probability_vec);
  return res;
}

void BBA::compute_spt_pls_uct() const {
  auto resize_space = [](std::vector<double>& vec, size_t size) {
    vec.clear();
    vec.resize(size, 0.0);
  };
  size_t size = _bba_vec.size();
  resize_space(_support_vec, size);
  resize_space(_plausibility_vec, size);
  resize_space(_uncertainty_vec, size);
  const std::vector<std::vector<size_t>>& subset_relations =
      _bba_manager_ptr->get_subset_relations();
  const std::vector<std::vector<size_t>>& inter_relations =
      _bba_manager_ptr->get_inter_relations();
  for (size_t i = 0; i < size; ++i) {
    double& spt = _support_vec[i];
    double& pls = _plausibility_vec[i];
    double& uct = _uncertainty_vec[i];
    const auto& subset_inds = subset_relations[i];
    const auto& inter_inds = inter_relations[i];
    // LOG_INFO << boost::format("inter_size: (%d %d)") % i % inter_inds.size();
    for (auto subset_ind : subset_inds) {
      spt += _bba_vec[subset_ind];
    }
    for (auto inter_ind : inter_inds) {
      pls += _bba_vec[inter_ind];
    }
    // LOG_INFO << boost::format("pls: (%d %lf)") % i % pls;
    uct = pls - spt;
  }
}

// TODO(zhangweide): use combination_relations to
// compute all the probability at one time, this way maybe not effective
void BBA::compute_probability() const {
  _probability_vec.clear();
  _probability_vec.resize(_bba_vec.size(), 0.0);
  const auto& combination_relations =
      _bba_manager_ptr->get_combination_relations();
  const std::vector<size_t>& fod_subset_cardinalities =
      _bba_manager_ptr->get_cardinalities();
  for (size_t i = 0; i < combination_relations.size(); ++i) {
    const auto& combination_pairs = combination_relations[i];
    double intersection_card = fod_subset_cardinalities[i];
    for (auto combination_pair : combination_pairs) {
      size_t a_ind = combination_pair.first;
      size_t b_ind = combination_pair.second;
      _probability_vec[a_ind] +=
          intersection_card / fod_subset_cardinalities[b_ind] * _bba_vec[b_ind];
    }
  }
}

void BBA::normalize() {
  double mass_sum = std::accumulate(_bba_vec.begin(), _bba_vec.end(), 0.0);
  if (mass_sum == 0.0) {
    AERROR << "mass_sum equal 0!!";
  }
  for (auto& belief_mass : _bba_vec) {
    belief_mass /= mass_sum;
  }
}

BBA operator+(const BBA& lhs, const BBA& rhs) {
  const BBAManager* lhs_manager_ptr = lhs.get_bba_manager_ptr();
  const BBAManager* rhs_manager_ptr = rhs.get_bba_manager_ptr();
  CHECK_EQ(lhs_manager_ptr, rhs_manager_ptr)
      << boost::format("lhs manager(%s) is not equal to rhs manager(%s)") %
             lhs_manager_ptr->get_manager_name() %
             rhs_manager_ptr->get_manager_name();
  const BBAManager* manager_ptr = lhs_manager_ptr;
  BBA res(manager_ptr);
  std::vector<double>& res_bba_vec = res._bba_vec;
  const auto& combination_relations = manager_ptr->get_combination_relations();
  for (size_t i = 0; i < res_bba_vec.size(); ++i) {
    const auto& combination_pairs = combination_relations[i];
    // LOG_INFO << "pairs size: " << combination_pairs.size();
    double& belief_mass = res_bba_vec[i];
    belief_mass = 0.0;
    for (auto combination_pair : combination_pairs) {
      // LOG_INFO << boost::format("(%d %d)") % combination_pair.first
      //     % combination_pair.second;
      belief_mass += lhs.get_ind_bfmass(combination_pair.first) *
                     rhs.get_ind_bfmass(combination_pair.second);
    }
    // LOG_INFO << boost::format("belief_mass: %lf") % belief_mass;
  }
  res.normalize();
  return res;
}

BBA operator*(const BBA& bba, double w) {
  const BBAManager* manager_ptr = bba.get_bba_manager_ptr();
  BBA res(manager_ptr);
  // check w
  if (w < 0.0 || w > 1.0) {
    AERROR << boost::format(
                  "the weight of bba %lf is not valid, return default bba") %
                  w;
    return res;
  }
  size_t fod_loc = manager_ptr->get_fod_ind();
  std::vector<double>& res_bba_vec = res._bba_vec;
  const std::vector<double>& bba_vec = bba.get_bba_vec();
  for (size_t i = 0; i < res_bba_vec.size(); ++i) {
    if (i == fod_loc) {
      res_bba_vec[i] = 1.0 - w + w * bba_vec[i];
    } else {
      res_bba_vec[i] = w * bba_vec[i];
    }
  }
  return res;
}

}  // namespace perception
}  // namespace apollo
