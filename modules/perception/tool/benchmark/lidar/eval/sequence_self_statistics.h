/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#include <vector>
#include "modules/perception/tool/benchmark/lidar/base/sequence_maintainer.h"

namespace apollo {
namespace perception {
namespace benchmark {

template <typename KeyType>
class SequenceSelfStatistics {
 public:
  SequenceSelfStatistics();
  ~SequenceSelfStatistics() = default;

  void reset();

  bool add_objects(const std::vector<ObjectPtr>& objects, KeyType key);
  void get_classification_type_change_rates(
      std::vector<std::vector<double>>* rate_per_class, double* rate) const;

 protected:
  void add_statistics(SequenceType<KeyType>* sequence);

 protected:
  SequenceMaintainer<KeyType> _sequences;

  std::vector<std::vector<unsigned int>> _type_change_counts;
};

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
