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
#include "modules/perception/camera/common/object_template_manager.h"

#include <algorithm>
#include <cfloat>
#include <tuple>
#include <utility>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/common/io/io_util.h"
#include "modules/perception/lib/config_manager/config_manager.h"

namespace apollo {
namespace perception {
namespace camera {

std::vector<base::ObjectSubType> kTypeCanBeRef = {base::ObjectSubType::CAR,
                                                  base::ObjectSubType::VAN};

std::vector<base::ObjectSubType> kTypeRefinedByTemplate = {
    base::ObjectSubType::CAR,        base::ObjectSubType::VAN,
    base::ObjectSubType::BUS,        base::ObjectSubType::TRUCK,
    base::ObjectSubType::PEDESTRIAN, base::ObjectSubType::TRAFFICCONE,
    base::ObjectSubType::CYCLIST,    base::ObjectSubType::MOTORCYCLIST,
};

std::vector<base::ObjectSubType> kTypeRefinedByRef = {
    base::ObjectSubType::BUS,        base::ObjectSubType::TRUCK,
    base::ObjectSubType::PEDESTRIAN, base::ObjectSubType::TRAFFICCONE,
    base::ObjectSubType::CYCLIST,    base::ObjectSubType::MOTORCYCLIST,
    base::ObjectSubType::TRICYCLIST};

ObjectTemplateManager::ObjectTemplateManager() {}

bool ObjectTemplateManager::Init(
    const ObjectTemplateManagerInitOptions &options) {
  lib::MutexLock lock(&mutex_);
  if (inited_) {
    return true;
  }

  nr_dim_per_tmplt_ = 3;

  std::string config =
      cyber::common::GetAbsolutePath(options.root_dir, options.conf_file);
  ObjectTemplateMeta proto;
  if (!cyber::common::GetProtoFromFile(config, &proto)) {
    AERROR << "Read config failed: " << config;
    return false;
  }

  CHECK(proto.has_max_dim_change_ratio());

  CHECK(proto.has_unknown());
  CHECK(proto.has_unknown_movable());
  CHECK(proto.has_unknown_unmovable());
  CHECK(proto.has_car());
  CHECK(proto.has_van());
  CHECK(proto.has_truck());
  CHECK(proto.has_bus());
  CHECK(proto.has_cyclist());
  CHECK(proto.has_motorcyclist());
  CHECK(proto.has_tricyclist());
  CHECK(proto.has_pedestrian());
  CHECK(proto.has_trafficcone());

  CHECK_LE(proto.car().dim_size(), 3);
  CHECK_LE(proto.van().dim_size(), 3);
  CHECK_LE(proto.truck().dim_size(), 3);
  CHECK_LE(proto.bus().dim_size(), 3);
  CHECK_LE(proto.cyclist().dim_size(), 3);
  CHECK_LE(proto.motorcyclist().dim_size(), 3);
  CHECK_LE(proto.tricyclist().dim_size(), 3);
  CHECK_LE(proto.pedestrian().dim_size(), 3);
  CHECK_LE(proto.trafficcone().dim_size(), 3);

  CHECK(proto.unknown().has_speed_limit());
  CHECK(proto.unknown_movable().has_speed_limit());
  CHECK(proto.unknown_unmovable().has_speed_limit());
  CHECK(proto.car().has_speed_limit());
  CHECK(proto.van().has_speed_limit());
  CHECK(proto.truck().has_speed_limit());
  CHECK(proto.bus().has_speed_limit());
  CHECK(proto.cyclist().has_speed_limit());
  CHECK(proto.motorcyclist().has_speed_limit());
  CHECK(proto.tricyclist().has_speed_limit());
  CHECK(proto.pedestrian().has_speed_limit());
  CHECK(proto.trafficcone().has_speed_limit());

  max_dim_change_ratio_ = proto.max_dim_change_ratio();

  total_nr_tmplts_veh_ = proto.car().dim_size() + proto.van().dim_size() +
                         proto.truck().dim_size() + proto.bus().dim_size();

  look_up_table_min_volume_index_.clear();
  veh_hwl_.resize(0);
  look_up_table_min_volume_index_[TemplateIndex::CAR_MIN_VOLUME_INDEX] =
      static_cast<int>(veh_hwl_.size());
  LoadVehTemplates(proto.car());
  look_up_table_min_volume_index_[TemplateIndex::VAN_MIN_VOLUME_INDEX] =
      static_cast<int>(veh_hwl_.size());
  LoadVehTemplates(proto.van());
  look_up_table_min_volume_index_[TemplateIndex::TRUCK_MIN_VOLUME_INDEX] =
      static_cast<int>(veh_hwl_.size());
  LoadVehTemplates(proto.truck());
  look_up_table_min_volume_index_[TemplateIndex::BUS_MIN_VOLUME_INDEX] =
      static_cast<int>(veh_hwl_.size());
  LoadVehTemplates(proto.bus());

  min_template_hwl_.clear();
  mid_template_hwl_.clear();
  max_template_hwl_.clear();
  LoadVehMinMidMaxTemplates(base::ObjectSubType::CAR, proto.car());
  LoadVehMinMidMaxTemplates(base::ObjectSubType::VAN, proto.van());
  LoadVehMinMidMaxTemplates(base::ObjectSubType::TRUCK, proto.truck());
  LoadVehMinMidMaxTemplates(base::ObjectSubType::BUS, proto.bus());
  LoadVehMinMidMaxTemplates(base::ObjectSubType::CYCLIST, proto.cyclist());
  LoadVehMinMidMaxTemplates(base::ObjectSubType::MOTORCYCLIST,
                            proto.motorcyclist());
  LoadVehMinMidMaxTemplates(base::ObjectSubType::TRICYCLIST,
                            proto.tricyclist());
  LoadVehMinMidMaxTemplates(base::ObjectSubType::PEDESTRIAN,
                            proto.pedestrian());
  LoadVehMinMidMaxTemplates(base::ObjectSubType::TRAFFICCONE,
                            proto.trafficcone());

  template_hwl_.resize(0);
  template_hwl_.push_back(min_template_hwl_);
  template_hwl_.push_back(mid_template_hwl_);
  template_hwl_.push_back(max_template_hwl_);

  type_speed_limit_.clear();
  type_speed_limit_[base::ObjectSubType::UNKNOWN] =
      proto.unknown().speed_limit();
  type_speed_limit_[base::ObjectSubType::UNKNOWN_MOVABLE] =
      proto.unknown_movable().speed_limit();
  type_speed_limit_[base::ObjectSubType::UNKNOWN_UNMOVABLE] =
      proto.unknown_unmovable().speed_limit();
  type_speed_limit_[base::ObjectSubType::CAR] = proto.car().speed_limit();
  type_speed_limit_[base::ObjectSubType::VAN] = proto.van().speed_limit();
  type_speed_limit_[base::ObjectSubType::TRUCK] = proto.truck().speed_limit();
  type_speed_limit_[base::ObjectSubType::BUS] = proto.bus().speed_limit();
  type_speed_limit_[base::ObjectSubType::CYCLIST] =
      proto.cyclist().speed_limit();
  type_speed_limit_[base::ObjectSubType::MOTORCYCLIST] =
      proto.motorcyclist().speed_limit();
  type_speed_limit_[base::ObjectSubType::TRICYCLIST] =
      proto.tricyclist().speed_limit();
  type_speed_limit_[base::ObjectSubType::PEDESTRIAN] =
      proto.pedestrian().speed_limit();
  type_speed_limit_[base::ObjectSubType::TRAFFICCONE] =
      proto.trafficcone().speed_limit();

  type_can_be_ref_ = kTypeCanBeRef;
  type_refined_by_template_ = kTypeRefinedByTemplate;
  type_refined_by_ref_ = kTypeRefinedByRef;

  inited_ = true;
  AINFO << "Init object_template_manager success.";
  return true;
}

void ObjectTemplateManager::LoadVehTemplates(const ObjectTemplate &tmplt) {
  std::vector<std::tuple<float, float, float> > list_tpl;
  list_tpl.resize(0);
  for (int i = 0; i < tmplt.dim_size(); ++i) {
    Dim dim = tmplt.dim(i);
    list_tpl.push_back(std::make_tuple(dim.h(), dim.w(), dim.l()));
  }
  std::sort(list_tpl.begin(), list_tpl.end());
  for (size_t i = 0; i < list_tpl.size(); ++i) {
    veh_hwl_.push_back(std::get<0>(list_tpl[i]));
    veh_hwl_.push_back(std::get<1>(list_tpl[i]));
    veh_hwl_.push_back(std::get<2>(list_tpl[i]));
  }
}

void ObjectTemplateManager::LoadVehMinMidMaxTemplates(
    const base::ObjectSubType &type, const ObjectTemplate &tmplt) {
  std::vector<std::tuple<float, float, float> > list_tpl;
  list_tpl.resize(0);
  for (int i = 0; i < tmplt.dim_size(); ++i) {
    Dim dim = tmplt.dim(i);
    list_tpl.push_back(std::make_tuple(dim.h(), dim.w(), dim.l()));
  }

  std::sort(list_tpl.begin(), list_tpl.end());

  int ind_min = 0;
  int ind_max = static_cast<int>(list_tpl.size()) - 1;
  int ind_mid = (ind_min + ind_max) / 2;
  std::vector<float> tmplt_min = {std::get<0>(list_tpl[ind_min]),
                                  std::get<1>(list_tpl[ind_min]),
                                  std::get<2>(list_tpl[ind_min])};
  std::vector<float> tmplt_mid = {std::get<0>(list_tpl[ind_mid]),
                                  std::get<1>(list_tpl[ind_mid]),
                                  std::get<2>(list_tpl[ind_mid])};
  std::vector<float> tmplt_max = {std::get<0>(list_tpl[ind_max]),
                                  std::get<1>(list_tpl[ind_max]),
                                  std::get<2>(list_tpl[ind_max])};
  min_template_hwl_[type] = tmplt_min;
  mid_template_hwl_[type] = tmplt_mid;
  max_template_hwl_[type] = tmplt_max;
}

// util for tmplt search
float ObjectTemplateManager::Get3dDimensionSimilarity(const float *hwl1,
                                                      const float *hwl2) {
  CHECK(hwl1 != nullptr);
  CHECK(hwl2 != nullptr);

  float max_h = std::max(hwl1[0], hwl2[0]);
  float min_h = hwl1[0] + hwl2[0] - max_h;
  float max_w = std::max(hwl1[1], hwl2[1]);
  float min_w = hwl1[1] + hwl2[1] - max_w;
  float max_l = std::max(hwl1[2], hwl2[2]);
  float min_l = hwl1[2] + hwl2[2] - max_l;

  float iou_h = min_h / (FLT_EPSILON + max_h);
  float iou_w = min_w / (FLT_EPSILON + max_w);
  float iou_l = min_l / (FLT_EPSILON + max_l);

  return iou_h * iou_h * iou_w * iou_l;  // h^2 * w * l
}

// for general visual obj
float ObjectTemplateManager::VehObjHwlBySearchTemplates(float *hwl, int *index,
                                                        bool *is_flip) {
  CHECK(inited_);
  CHECK(hwl != nullptr);

  float hwl_flip[3] = {hwl[0], hwl[2], hwl[1]};
  float score_best = -FLT_MAX;
  int i_best = -1;
  int i3 = 0;
  bool from_flip = false;
  for (int i = 0; i < total_nr_tmplts_veh_; ++i) {
    float score = Get3dDimensionSimilarity(hwl, &veh_hwl_[i3]);
    float score_flip = Get3dDimensionSimilarity(hwl_flip, &veh_hwl_[i3]);
    bool from_flip_cur = false;
    if (score_flip > score) {
      score = score_flip;
      from_flip_cur = true;
    }
    if (score > score_best) {
      score_best = score;
      i_best = i;
      from_flip = from_flip_cur;
    }
    i3 += 3;
  }
  int i_best_by_3 = i_best * 3;
  float hwl_tmplt_matched[3] = {veh_hwl_[i_best_by_3],
                                veh_hwl_[i_best_by_3 + 1],
                                veh_hwl_[i_best_by_3 + 2]};
  if (from_flip) {
    std::swap(hwl_tmplt_matched[1], hwl_tmplt_matched[2]);
  }

  float dh = fabsf(hwl[0] - hwl_tmplt_matched[0]);
  float dh_ratio = dh / hwl_tmplt_matched[0];
  float dw = fabsf(hwl[1] - hwl_tmplt_matched[1]);
  float dw_ratio = dw / hwl_tmplt_matched[1];
  float dl = fabsf(hwl[2] - hwl_tmplt_matched[2]);
  float dl_ratio = dl / hwl_tmplt_matched[2];
  float dh_dw_dl_ratio_mean = (dh_ratio + dw_ratio + dl_ratio) / 3;
  float dh_ratio_check = std::min(dh_ratio, dh_dw_dl_ratio_mean);
  if (score_best < FLT_EPSILON || dh_ratio_check > max_dim_change_ratio_) {
    return -1.0f;
  }
  ADEBUG << dh_ratio << ", " << dw_ratio << ", " << dl_ratio;

  hwl[0] = veh_hwl_[i_best_by_3];
  hwl[1] = veh_hwl_[i_best_by_3 + 1];
  hwl[2] = veh_hwl_[i_best_by_3 + 2];
  if (index != nullptr) {
    *index = i_best;
  }
  if (is_flip != nullptr) {
    *is_flip = from_flip;
  }
  return score_best;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
