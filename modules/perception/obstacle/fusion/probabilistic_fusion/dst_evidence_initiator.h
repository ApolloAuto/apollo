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

#ifndef MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_DST_EVIDENCE_INITIATOR_H_  // NOLINT
#define MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_DST_EVIDENCE_INITIATOR_H_  // NOLINT

#include <map>
#include <string>
#include <vector>

#include "modules/perception/obstacle/base/types.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/dst_evidence.h"

namespace apollo {
namespace perception {

// @brief: A singleton class to manager the mapping relations between
// obstacles types and hypotheses types in DST evidence theory. This
// class also do the initialization work of BBAManager
class DSTInitiator {
 public:
  // for classifying
  enum {
    PEDESTRIAN = (1 << 0),
    BICYCLE = (1 << 1),
    VEHICLE = (1 << 2),
    OTHERS_MOVABLE = (1 << 3),
    OTHERS_UNMOVABLE = (1 << 4)
  };
  enum {
    OTHERS = (OTHERS_MOVABLE | OTHERS_UNMOVABLE),
    UNKNOWN = (PEDESTRIAN | BICYCLE | VEHICLE | OTHERS)
  };

  static DSTInitiator& instance() {
    static DSTInitiator dst_initiator;
    return dst_initiator;
  }
  bool typ_to_hyp(size_t object_type, uint64_t *hypothesis_type) const {
    auto find_res = _typ_to_hyp_map.find(object_type);
    if (find_res == _typ_to_hyp_map.end()) {
      return false;
    }
    *hypothesis_type = find_res->second;
    return true;
  }
  uint64_t typ_to_hyp(size_t object_type) const {
    auto find_res = _typ_to_hyp_map.find(object_type);
    // CHECK(find_res != _typ_to_hyp_map.end());
    return find_res->second;
  }
  bool hyp_to_typ(uint64_t hypothesis_type, size_t *object_type) const {
    auto find_res = _hyp_to_typ_map.find(hypothesis_type);
    if (find_res == _hyp_to_typ_map.end()) {
      return false;
    }
    *object_type = find_res->second;
    return true;
  }
  size_t hyp_to_typ(uint64_t hypothesis_type) const {
    auto find_res = _hyp_to_typ_map.find(hypothesis_type);
    // CHECK(find_res != _hyp_to_typ_map.end());
    return find_res->second;
  }
  bool initialize_bba_manager() const {
    if (!BBAManager::instance(_classify_manager_name)
             .init(_fod_subsets, _subset_names)) {
      return false;
    }
    return true;
  }

  std::vector<std::string> get_all_managers_names() const {
    std::vector<std::string> managers_names = {_classify_manager_name};
    return managers_names;
  }

 private:
  // for classifying
  std::vector<uint64_t> _fod_subsets;
  std::vector<std::string> _subset_names;
  std::map<size_t, uint64_t> _typ_to_hyp_map;
  std::map<uint64_t, size_t> _hyp_to_typ_map;
  std::string _classify_manager_name;

  DSTInitiator() {
    // for classifying
    _classify_manager_name = "classify";
    _fod_subsets = {PEDESTRIAN,       BICYCLE, VEHICLE, OTHERS_MOVABLE,
                    OTHERS_UNMOVABLE, OTHERS,  UNKNOWN};
    _subset_names = {"PEDESTRIAN",       "BICYCLE", "VEHICLE", "OTHERS_MOVABLE",
                     "OTHERS_UNMOVABLE", "OTHERS",  "UNKNOWN"};
    _typ_to_hyp_map = {
        {ObjectType::PEDESTRIAN, PEDESTRIAN},
        {ObjectType::BICYCLE, BICYCLE},
        {ObjectType::VEHICLE, VEHICLE},
        {ObjectType::UNKNOWN_MOVABLE, OTHERS_MOVABLE},
        {ObjectType::UNKNOWN_UNMOVABLE, OTHERS_UNMOVABLE},
        {ObjectType::UNKNOWN, OTHERS},
    };
    // note: others and unkown hyp types are mapping to unkown object type
    _hyp_to_typ_map = {{PEDESTRIAN, ObjectType::PEDESTRIAN},
                       {BICYCLE, ObjectType::BICYCLE},
                       {VEHICLE, ObjectType::VEHICLE},
                       {OTHERS_MOVABLE, ObjectType::UNKNOWN_MOVABLE},
                       {OTHERS_UNMOVABLE, ObjectType::UNKNOWN_UNMOVABLE},
                       {OTHERS, ObjectType::UNKNOWN},
                       {UNKNOWN, ObjectType::UNKNOWN}};
  }
  DSTInitiator(const DSTInitiator& ohter);
  DSTInitiator& operator=(const BBAManager& other);
  ~DSTInitiator() {}
};

}  // namespace perception
}  // namespace apollo

#endif
