/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <map>
#include <string>
#include <vector>

#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "gflags/gflags.h"
#include "modules/perception/base/object_types.h"
#include "modules/perception/camera/common/proto/object_template_meta_schema.pb.h"
#include "modules/perception/lib/thread/mutex.h"

namespace apollo {
namespace perception {
namespace camera {

enum class TemplateIndex {
  CAR_MIN_VOLUME_INDEX,
  VAN_MIN_VOLUME_INDEX,
  TRUCK_MIN_VOLUME_INDEX,
  BUS_MIN_VOLUME_INDEX,
};

typedef std::map<base::ObjectSubType, std::vector<float> > TemplateMap;

struct ObjectTemplateManagerInitOptions {
  std::string root_dir;
  std::string conf_file;
  int gpu_id = 0;
  bool use_cyber_work_root = false;
};

class ObjectTemplateManager {
 public:
  bool Init(const ObjectTemplateManagerInitOptions &options);
  // for general visual obj typed as vehicle
  float VehObjHwlBySearchTemplates(float *hwl, int *index = nullptr,
                                   bool *is_flip = nullptr);
  const int NrDimPerTmplt() {
    ACHECK(inited_);
    return nr_dim_per_tmplt_;
  }
  const std::vector<float> &VehHwl() {
    ACHECK(inited_);
    return veh_hwl_;
  }
  const std::map<TemplateIndex, int> &LookUpTableMinVolumeIndex() {
    ACHECK(inited_);
    return look_up_table_min_volume_index_;
  }
  const std::map<base::ObjectSubType, float> &TypeSpeedLimit() {
    ACHECK(inited_);
    return type_speed_limit_;
  }
  const std::vector<base::ObjectSubType> &TypeRefinedByTemplate() {
    ACHECK(inited_);
    return type_refined_by_template_;
  }
  const std::vector<base::ObjectSubType> &TypeRefinedByRef() {
    ACHECK(inited_);
    return type_refined_by_ref_;
  }
  const std::vector<base::ObjectSubType> &TypeCanBeRef() {
    ACHECK(inited_);
    return type_can_be_ref_;
  }
  const TemplateMap &MinTemplateHWL() {
    ACHECK(inited_);
    return min_template_hwl_;
  }
  const TemplateMap &MidTemplateHWL() {
    ACHECK(inited_);
    return mid_template_hwl_;
  }
  const TemplateMap &MaxTemplateHWL() {
    ACHECK(inited_);
    return max_template_hwl_;
  }
  const std::vector<TemplateMap> &TemplateHWL() {
    ACHECK(inited_);
    return template_hwl_;
  }

 private:
  void LoadVehTemplates(const ObjectTemplate &tmplt);
  void LoadVehMinMidMaxTemplates(const base::ObjectSubType &type,
                                 const ObjectTemplate &tmplt);

  // util for tmplt search
  float Get3dDimensionSimilarity(const float *hwl1, const float *hwl2);

 private:
  bool inited_ = false;
  lib::Mutex mutex_;

  int nr_dim_per_tmplt_ = 0;
  int total_nr_tmplts_veh_ = 0;
  float max_dim_change_ratio_ = 0.0f;
  // tmplt
  std::vector<float> veh_hwl_;
  // index for min vol in veh class
  std::map<TemplateIndex, int> look_up_table_min_volume_index_;

  std::map<base::ObjectSubType, float> type_speed_limit_;

  std::vector<base::ObjectSubType> type_refined_by_template_;
  std::vector<base::ObjectSubType> type_refined_by_ref_;
  std::vector<base::ObjectSubType> type_can_be_ref_;

  TemplateMap min_template_hwl_;
  TemplateMap mid_template_hwl_;
  TemplateMap max_template_hwl_;
  std::vector<TemplateMap> template_hwl_;

  DECLARE_SINGLETON(ObjectTemplateManager)
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
