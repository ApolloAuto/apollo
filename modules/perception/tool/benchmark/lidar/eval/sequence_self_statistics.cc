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

#include "modules/perception/tool/benchmark/lidar/eval/sequence_self_statistics.h"
#include "modules/perception/tool/benchmark/lidar/eval/meta_statistics.h"

namespace apollo {
namespace perception {
namespace benchmark {

template <typename KeyType>
SequenceSelfStatistics<KeyType>::SequenceSelfStatistics() {
  reset();
}

template <typename KeyType>
void SequenceSelfStatistics<KeyType>::reset() {
  _type_change_counts.resize(
      MetaStatistics::get_type_dim(),
      std::vector<unsigned int>(MetaStatistics::get_type_dim(), 0));
}

template <typename KeyType>
bool SequenceSelfStatistics<KeyType>::add_objects(
    const std::vector<ObjectPtr>& objects, KeyType key) {
  if (!_sequences.add_data(objects, key)) {
    return false;
  }
  for (const auto& obj : objects) {
    auto seq = _sequences.get_sequence(obj->track_id);
    if (seq == nullptr) {
      std::cerr << "Sequence pointer is nullptr." << std::endl;
      return false;
    }
    add_statistics(seq);
  }
  return true;
}

template <typename KeyType>
void SequenceSelfStatistics<KeyType>::add_statistics(
    SequenceType<KeyType>* sequence) {
  if (sequence->size() <= 1) {
    return;
  }
  auto iter = sequence->rbegin();
  auto& cur = iter->second;
  ++iter;
  auto& pre = iter->second;
  ++_type_change_counts[MetaStatistics::get_type_index(pre->type)]
                       [MetaStatistics::get_type_index(cur->type)];
}

template <typename KeyType>
void SequenceSelfStatistics<KeyType>::get_classification_type_change_rates(
    std::vector<std::vector<double>>* rate_per_class, double* rate) const {
  rate_per_class->clear();
  rate_per_class->resize(
      MetaStatistics::get_type_dim(),
      std::vector<double>(MetaStatistics::get_type_dim(), 0.0));
  unsigned int total_sum = 0;
  unsigned int change_sum = 0;
  for (unsigned int i = 0; i < MetaStatistics::get_type_dim(); ++i) {
    unsigned int sum = std::accumulate(_type_change_counts[i].begin(),
                                       _type_change_counts[i].end(), 0);
    total_sum += sum;
    change_sum += sum - _type_change_counts[i][i];
    for (unsigned int j = 0; j < MetaStatistics::get_type_dim(); ++j) {
      rate_per_class->at(i)[j] =
          static_cast<double>(_type_change_counts[i][j]) / sum;
    }
  }
  *rate = static_cast<double>(change_sum) / total_sum;
}

// specified for timestamp key
template class SequenceSelfStatistics<double>;
// specified for id key
template class SequenceSelfStatistics<unsigned int>;

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
