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

#ifndef MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_DST_EVIDENCE_H_  // NOLINT
#define MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_DST_EVIDENCE_H_  // NOLINT

#include <stdint.h>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <utility>
namespace apollo {
namespace perception {

class BBAManager;
class BBA;
// @brief: A singleton class to mange the set of fod subset and the
// intersection relationship between them. Note:
// 1. this class uses bits of uint64_t to represent the subsets of fod,
// so the max number of hypotheses are 64.
// 2. the input set of fod subsets must have closure under the operation
// intersection.
class BBAManager {
 public:
  static BBAManager& instance(const std::string manager_name = "default") {
    static std::map<std::string, std::shared_ptr<BBAManager>> bba_managers_ptrs;
    std::shared_ptr<BBAManager>& manager_ptr = bba_managers_ptrs[manager_name];
    if (manager_ptr == nullptr) {
      manager_ptr.reset(new BBAManager(manager_name));
    }
    return *manager_ptr;
  }
  // getter
  const std::string get_manager_name() const { return _manager_name; }
  const std::vector<uint64_t>& get_fod_subsets() const { return _fod_subsets; }
  const std::vector<size_t>& get_cardinalities() const {
    return _fod_subset_cardinalities;
  }
  const std::vector<std::string>& get_fod_subset_names() const {
    return _fod_subset_names;
  }
  size_t get_subsets_size() const { return _fod_subsets.size(); }
  const std::vector<std::vector<std::pair<size_t, size_t>>>&
  get_combination_relations() const {
    return _combination_relations;
  }
  const std::vector<std::vector<size_t>>& get_subset_relations() const {
    return _subset_relations;
  }
  const std::vector<std::vector<size_t>>& get_inter_relations() const {
    return _inter_relations;
  }
  const std::map<uint64_t, size_t>& get_subsets_ind_map() const {
    return _subsets_ind_map;
  }
  size_t get_fod_ind() const { return _fod_loc; }
  uint64_t get_fod() const { return _fod_subsets[_fod_loc]; }
  std::string get_subset_name(uint64_t fod_subset) const {
    return _fod_subset_names[fod_subset_to_ind(fod_subset)];
  }

  bool init(const std::vector<uint64_t>& fod_subsets,
            const std::vector<std::string>& fod_subset_names =
                std::vector<std::string>());
  bool fod_subset_exsits(uint64_t fod_subset) const;
  bool ind_exsits(size_t ind) const;
  size_t fod_subset_to_ind(uint64_t fod_subset) const;
  uint64_t ind_to_fod_subset(size_t ind) const;
  bool initialized() const { return _init; }

 private:
  std::string _manager_name;
  // ensure initialize BBAManager once
  bool _init = false;
  size_t _fod_loc = 0;
  std::vector<uint64_t> _fod_subsets;
  // for transforming to probability effectively
  std::vector<size_t> _fod_subset_cardinalities;
  std::vector<std::string> _fod_subset_names;

  // sets relations
  // for combining two bbas effectively.
  std::vector<std::vector<std::pair<size_t, size_t>>> _combination_relations;
  // for computing support vector effectively
  std::vector<std::vector<size_t>> _subset_relations;
  // for computing plausibility vector effectively
  std::vector<std::vector<size_t>> _inter_relations;
  std::map<uint64_t, size_t> _subsets_ind_map;

  void build_subsets_ind_map();
  // fod check, put fod in fod_subsets to ensure BBA's validity after
  // default construction.
  void fod_check();
  // compute the cardinality of fod_subset which means counting set bits in
  // an integer
  void compute_cardinalities();
  bool compute_relations();
  void build_names_map(const std::vector<std::string>& fod_subset_names);

  explicit BBAManager(std::string manager_name) {
    _manager_name = manager_name;
  }

  explicit BBAManager(const BBAManager& other);
  BBAManager& operator=(const BBAManager& other);

  friend class BBAManagerTest;
};

// @brief: Implementation of BBA, mainly include the operations of
// combination and discount with a reliability factor. Note, all the
// BBAs must obey the BBAManager, that is to say, the set of fod subsets
// is always the same as BBAManager. As a result, we only need save a
// fixed length normalized vector, the meaning of the vector is explained
// by BBAManager.
class BBA {
 public:
  // the bba constructor will use the default bba manager if input the
  // nullptr
  explicit BBA(const BBAManager* bba_manager_ptr = nullptr);
  ~BBA() {}
  // getter
  const BBAManager* get_bba_manager_ptr() const { return _bba_manager_ptr; }
  const std::vector<double>& get_bba_vec() const { return _bba_vec; }
  const size_t get_bba_size() const { return _bba_vec.size(); }
  double get_subset_bfmass(uint64_t fod_subset) const;
  double get_ind_bfmass(size_t ind) const;
  const std::vector<double>& get_support_vec() const { return _support_vec; }
  const std::vector<double>& get_plausibility_vec() const {
    return _plausibility_vec;
  }
  const std::vector<double>& get_uncertainty_vec() const {
    return _uncertainty_vec;
  }
  const std::vector<double>& get_probability_vec() const {
    return _probability_vec;
  }

  // setter
  bool set_bba_vec(const std::vector<double>& bba_vec);
  // strictly require the fod in bba_map is valid
  bool set_bba(const std::map<uint64_t, double>& bba_map);

  std::string print_bba() const;

  void compute_spt_pls_uct() const;
  void compute_probability() const;

  friend BBA operator+(const BBA& lhs, const BBA& rhs);
  friend BBA operator*(const BBA& bba, double w);

 private:
  const BBAManager* _bba_manager_ptr;
  std::vector<double> _bba_vec;
  // the construction of following vectors is manual.
  mutable std::vector<double> _support_vec;
  mutable std::vector<double> _plausibility_vec;
  mutable std::vector<double> _uncertainty_vec;
  mutable std::vector<double> _probability_vec;

  // normalize itself
  void normalize();
};
}  // namespace perception
}  // namespace apollo

#endif
