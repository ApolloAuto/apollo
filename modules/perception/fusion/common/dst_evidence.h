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
#pragma once

#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace apollo {
namespace perception {
namespace fusion {

struct DstCommonData {
  // ensure initialize DSTEvidence once
  bool init_ = false;
  // fods
  size_t fod_loc_ = 0;
  std::vector<uint64_t> fod_subsets_;
  // for transforming to probability effectively
  std::vector<size_t> fod_subset_cardinalities_;
  std::vector<std::string> fod_subset_names_;
  // for combining two bbas effectively.
  std::vector<std::vector<std::pair<size_t, size_t>>> combination_relations_;
  // for computing support vector effectively
  std::vector<std::vector<size_t>> subset_relations_;
  // for computing plausibility vector effectively
  std::vector<std::vector<size_t>> inter_relations_;
  std::map<uint64_t, size_t> subsets_ind_map_;
};

typedef DstCommonData* DstCommonDataPtr;

// @brief: A singleton class to mange the set of fod subset and the
// intersection relationship between them.
class DstManager {
 public:
  static DstManager* Instance() {
    static DstManager dst_manager;
    return &dst_manager;
  }
  // brief: app initialization
  // param [in]: app_name
  // param [in]: fod_subsets, hypotheses sets
  // param [in]: fod_subset_names
  bool AddApp(const std::string& app_name,
              const std::vector<uint64_t>& fod_subsets,
              const std::vector<std::string>& fod_subset_names =
                  std::vector<std::string>());
  bool IsAppAdded(const std::string& app_name);

  DstCommonDataPtr GetAppDataPtr(const std::string& app_name);
  size_t FodSubsetToInd(const std::string& app_name,
                        const uint64_t& fod_subset);
  uint64_t IndToFodSubset(const std::string& app_name, const size_t& ind);

 private:
  DstManager() {}
  void BuildSubsetsIndMap(DstCommonData* dst_data);
  // fod check, put fod in fod_subsets to ensure BBA's validity after
  // default construction.
  void FodCheck(DstCommonData* dst_data);
  // compute the cardinality of fod_subset which means counting set bits in
  // an integer
  void ComputeCardinalities(DstCommonData* st_data);
  bool ComputeRelations(DstCommonData* dst_data);
  void BuildNamesMap(const std::vector<std::string>& fod_subset_names,
                     DstCommonData* dst_data);

 private:
  // Dst data map
  std::map<std::string, DstCommonData> dst_common_data_;

  std::mutex map_mutex_;
};

class Dst {
 public:
  explicit Dst(const std::string& app_name);

  // setter
  bool SetBbaVec(const std::vector<double>& bba_vec);
  // strictly require the fod in bba_map is valid
  bool SetBba(const std::map<uint64_t, double>& bba_map);

  void ComputeSptPlsUct() const;
  void ComputeProbability() const;
  // getter
  const std::vector<double>& GetBbaVec() const { return bba_vec_; }
  const size_t GetBbaSize() const { return bba_vec_.size(); }
  double GetSubsetBfmass(uint64_t fod_subset) const;
  double GetIndBfmass(size_t ind) const;
  const std::vector<double>& GetSupportVec() const { return support_vec_; }
  const std::vector<double>& GetPlausibilityVec() const {
    return plausibility_vec_;
  }
  const std::vector<double>& GetUncertaintyVec() const {
    return uncertainty_vec_;
  }
  const std::vector<double>& GetProbabilityVec() const {
    return probability_vec_;
  }
  std::string PrintBba() const;

  friend Dst operator+(const Dst& lhs, const Dst& rhs);
  friend Dst operator*(const Dst& dst_evidence, double w);
  std::string Name() const { return app_name_; }

 private:
  void Normalize();
  void SelfCheck() const;

 private:
  std::string app_name_;
  // the construction of following vectors is manual.
  mutable DstCommonDataPtr dst_data_ptr_ = nullptr;
  mutable std::vector<double> bba_vec_;
  mutable std::vector<double> support_vec_;
  mutable std::vector<double> plausibility_vec_;
  mutable std::vector<double> uncertainty_vec_;
  mutable std::vector<double> probability_vec_;
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
