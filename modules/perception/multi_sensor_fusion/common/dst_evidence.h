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

  /**
   * @brief Whether app added
   *
   * @param app_name
   * @return true
   * @return false
   */
  bool IsAppAdded(const std::string& app_name);

  /**
   * @brief Get the app data ptr object
   *
   * @param app_name
   * @return DstCommonDataPtr
   */
  DstCommonDataPtr GetAppDataPtr(const std::string& app_name);

  /**
   * @brief
   *
   * @param app_name
   * @param fod_subset
   * @return size_t
   */
  size_t FodSubsetToInd(const std::string& app_name,
                        const uint64_t& fod_subset);

  /**
   * @brief
   *
   * @param app_name
   * @param ind
   * @return uint64_t
   */
  uint64_t IndToFodSubset(const std::string& app_name, const size_t& ind);

 private:
  DstManager() {}

  /**
   * @brief
   *
   * @param dst_data
   */
  void BuildSubsetsIndMap(DstCommonData* dst_data);

  /**
   * @brief fod check, put fod in fod_subsets to ensure BBA's validity after
   * default construction.
   * @param dst_data
   */
  void FodCheck(DstCommonData* dst_data);

  /**
   * @brief compute the cardinality of fod_subset which means counting set bits in
   * an integer
   * @param st_data
   */
  void ComputeCardinalities(DstCommonData* st_data);

  /**
   * @brief
   *
   * @param dst_data
   * @return true
   * @return false
   */
  bool ComputeRelations(DstCommonData* dst_data);

  /**
   * @brief
   *
   * @param fod_subset_names
   * @param dst_data
   */
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

  /**
   * @brief Set the Bba Vec object
   *
   * @param bba_vec
   * @return true
   * @return false
   */
  bool SetBbaVec(const std::vector<double>& bba_vec);

  /**
   * @brief Set the Bba object, strictly require the fod in bba_map is valid
   *
   * @param bba_map
   * @return true
   * @return false
   */
  bool SetBba(const std::map<uint64_t, double>& bba_map);

  /**
   * @brief
   *
   */
  void ComputeSptPlsUct() const;

  /**
   * @brief
   *
   */
  void ComputeProbability() const;

  /**
   * @brief Get the Bba Vec object
   *
   * @return const std::vector<double>&
   */
  const std::vector<double>& GetBbaVec() const { return bba_vec_; }

  /**
   * @brief Get the Bba Size object
   *
   * @return const size_t
   */
  const size_t GetBbaSize() const { return bba_vec_.size(); }

  /**
   * @brief Get the Subset Bfmass object
   *
   * @param fod_subset
   * @return double
   */
  double GetSubsetBfmass(uint64_t fod_subset) const;

  /**
   * @brief Get the Ind Bfmass object
   *
   * @param ind
   * @return double
   */
  double GetIndBfmass(size_t ind) const;

  /**
   * @brief Get the Support Vec object
   *
   * @return const std::vector<double>&
   */
  const std::vector<double>& GetSupportVec() const { return support_vec_; }

  /**
   * @brief Get the Plausibility Vec object
   *
   * @return const std::vector<double>&
   */
  const std::vector<double>& GetPlausibilityVec() const {
    return plausibility_vec_;
  }

  /**
   * @brief Get the Uncertainty Vec object
   *
   * @return const std::vector<double>&
   */
  const std::vector<double>& GetUncertaintyVec() const {
    return uncertainty_vec_;
  }

  /**
   * @brief Get the Probability Vec object
   *
   * @return const std::vector<double>&
   */
  const std::vector<double>& GetProbabilityVec() const {
    return probability_vec_;
  }

  /**
   * @brief
   *
   * @return std::string
   */
  std::string PrintBba() const;

  /**
   * @brief
   *
   * @param lhs
   * @param rhs
   * @return Dst
   */
  friend Dst operator+(const Dst& lhs, const Dst& rhs);

  /**
   * @brief
   *
   * @param dst_evidence
   * @param w
   * @return Dst
   */
  friend Dst operator*(const Dst& dst_evidence, double w);

  /**
   * @brief
   *
   * @return std::string
   */
  std::string Name() const { return app_name_; }

 private:
  /**
   * @brief
   *
   */
  void Normalize();

  /**
   * @brief
   *
   */
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
