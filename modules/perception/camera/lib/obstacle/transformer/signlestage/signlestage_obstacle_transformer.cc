/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/camera/lib/obstacle/transformer/signlestage/signlestage_obstacle_transformer.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/camera/common/global_config.h"

namespace apollo {
namespace perception {
namespace camera {

bool SignleStageObstacleTransformer::Init(
    const ObstacleTransformerInitOptions &options) {
  std::string transformer_config =
      cyber::common::GetAbsolutePath(options.root_dir, options.conf_file);

  if (!cyber::common::GetProtoFromFile(transformer_config,
                                      &signlestage_param_)) {
    AERROR << "Read config failed: " << transformer_config;
    return false;
  }
  AINFO << "Load transformer parameters from " << transformer_config
        << " \nmin dimension: " << signlestage_param_.min_dimension_val()
        << " \ndo template search: " << signlestage_param_.check_dimension();

  // Init object template
  object_template_manager_ = ObjectTemplateManager::Instance();

  return true;
}

int SignleStageObstacleTransformer::MatchTemplates(
    base::ObjectSubType sub_type, float *dimension_hwl) {
  const TemplateMap &kMinTemplateHWL =
      object_template_manager_->MinTemplateHWL();
  const TemplateMap &kMidTemplateHWL =
      object_template_manager_->MidTemplateHWL();
  const TemplateMap &kMaxTemplateHWL =
      object_template_manager_->MaxTemplateHWL();

  int type_min_vol_index = 0;
  float min_dimension_val =
      std::min(std::min(dimension_hwl[0], dimension_hwl[1]), dimension_hwl[2]);

  const std::map<TemplateIndex, int> &kLookUpTableMinVolumeIndex =
      object_template_manager_->LookUpTableMinVolumeIndex();

  switch (sub_type) {
    case base::ObjectSubType::TRAFFICCONE: {
      const float *min_tmplt_cur_type = kMinTemplateHWL.at(sub_type).data();
      const float *max_tmplt_cur_type = kMaxTemplateHWL.at(sub_type).data();
      dimension_hwl[0] = std::min(dimension_hwl[0], max_tmplt_cur_type[0]);
      dimension_hwl[0] = std::max(dimension_hwl[0], min_tmplt_cur_type[0]);
      break;
    }
    case base::ObjectSubType::PEDESTRIAN:
    case base::ObjectSubType::CYCLIST:
    case base::ObjectSubType::MOTORCYCLIST: {
      const float *min_tmplt_cur_type = kMinTemplateHWL.at(sub_type).data();
      const float *mid_tmplt_cur_type = kMidTemplateHWL.at(sub_type).data();
      const float *max_tmplt_cur_type = kMaxTemplateHWL.at(sub_type).data();
      float dh_min = fabsf(dimension_hwl[0] - min_tmplt_cur_type[0]);
      float dh_mid = fabsf(dimension_hwl[0] - mid_tmplt_cur_type[0]);
      float dh_max = fabsf(dimension_hwl[0] - max_tmplt_cur_type[0]);
      std::vector<std::pair<float, float>> diff_hs;
      diff_hs.push_back(std::make_pair(dh_min, min_tmplt_cur_type[0]));
      diff_hs.push_back(std::make_pair(dh_mid, mid_tmplt_cur_type[0]));
      diff_hs.push_back(std::make_pair(dh_max, max_tmplt_cur_type[0]));
      sort(diff_hs.begin(), diff_hs.end(),
           [](const std::pair<float, float> &a,
              const std::pair<float, float> &b) -> bool {
             return a.first < b.first;
           });
      dimension_hwl[0] = diff_hs[0].second;
      break;
    }
    case base::ObjectSubType::CAR:
      type_min_vol_index =
          kLookUpTableMinVolumeIndex.at(TemplateIndex::CAR_MIN_VOLUME_INDEX);
      break;
    case base::ObjectSubType::VAN:
      type_min_vol_index =
          kLookUpTableMinVolumeIndex.at(TemplateIndex::VAN_MIN_VOLUME_INDEX);
      break;
    case base::ObjectSubType::TRUCK:
      type_min_vol_index =
          kLookUpTableMinVolumeIndex.at(TemplateIndex::TRUCK_MIN_VOLUME_INDEX);
      break;
    case base::ObjectSubType::BUS:
      type_min_vol_index =
          kLookUpTableMinVolumeIndex.at(TemplateIndex::BUS_MIN_VOLUME_INDEX);
      break;
    default:
      if (min_dimension_val < signlestage_param_.min_dimension_val()) {
        common::IScale3(dimension_hwl, signlestage_param_.min_dimension_val() *
                                           common::IRec(min_dimension_val));
      }
      break;
  }
  return type_min_vol_index;
}

void SignleStageObstacleTransformer::FillResults(
    float object_center[3], float dimension_hwl[3], float rotation_y,
    Eigen::Affine3d camera2world_pose, float theta_ray, base::ObjectPtr obj) {
  if (obj == nullptr) {
    return;
  }
  object_center[1] -= dimension_hwl[0] / 2;
  obj->camera_supplement.local_center(0) = object_center[0];
  obj->camera_supplement.local_center(1) = object_center[1];
  obj->camera_supplement.local_center(2) = object_center[2];
  ADEBUG << "Obj id: " << obj->track_id;
  ADEBUG << "Obj type: " << static_cast<int>(obj->sub_type);
  ADEBUG << "Obj ori dimension: " << obj->size[2] << ", " << obj->size[1]
         << ", " << obj->size[0];
  obj->center(0) = static_cast<double>(object_center[0]);
  obj->center(1) = static_cast<double>(object_center[1]);
  obj->center(2) = static_cast<double>(object_center[2]);
  obj->center = camera2world_pose * obj->center;

  obj->size(2) = dimension_hwl[0];
  obj->size(1) = dimension_hwl[1];
  obj->size(0) = dimension_hwl[2];

  obj->center_uncertainty(0) = static_cast<float>(0);
  obj->center_uncertainty(1) = static_cast<float>(0);
  obj->center_uncertainty(2) = static_cast<float>(0);

  float theta = rotation_y;
  Eigen::Vector3d dir = (camera2world_pose.matrix().block(0, 0, 3, 3) *
                         Eigen::Vector3d(cos(theta), 0, -sin(theta)));
  obj->direction[0] = static_cast<float>(dir[0]);
  obj->direction[1] = static_cast<float>(dir[1]);
  obj->direction[2] = static_cast<float>(dir[2]);
  obj->theta = static_cast<float>(atan2(dir[1], dir[0]));
  obj->theta_variance = static_cast<float>(1.0);

  obj->camera_supplement.alpha = rotation_y - theta_ray;

  ADEBUG << "Dimension hwl: " << dimension_hwl[0] << ", " << dimension_hwl[1]
         << ", " << dimension_hwl[2];
  ADEBUG << "Obj ry:" << rotation_y;
  ADEBUG << "Obj theta: " << obj->theta;
  ADEBUG << "Obj center from transformer: " << obj->center.transpose();
}

bool SignleStageObstacleTransformer::Transform(
    const ObstacleTransformerOptions &options, CameraFrame *frame) {
  if (frame->detected_objects.empty()) {
    ADEBUG << "No object input to transformer.";
    return true;
  }

  const auto &camera_k_matrix = frame->camera_k_matrix;
  float k_mat[9] = {0};
  for (size_t i = 0; i < 3; i++) {
    size_t i3 = i * 3;
    for (size_t j = 0; j < 3; j++) {
      k_mat[i3 + j] = camera_k_matrix(i, j);
    }
  }
  ADEBUG << "Camera k matrix input to transformer: \n"
         << k_mat[0] << ", " << k_mat[1] << ", " << k_mat[2] << "\n"
         << k_mat[3] << ", " << k_mat[4] << ", " << k_mat[5] << "\n"
         << k_mat[6] << ", " << k_mat[7] << ", " << k_mat[8] << "\n";
  const auto &camera2world_pose = frame->camera2world_pose;

  int nr_transformed_obj = 0;
  const float PI = common::Constant<float>::PI();
  for (auto &obj : frame->detected_objects) {
    if (obj == nullptr) {
      ADEBUG << "Empty object input to transformer.";
      continue;
    }

    // set object mapper options
    float theta_ray = atan2(obj->camera_supplement.local_center[0],
                           obj->camera_supplement.local_center[2]);

    // process
    float object_center[3] = {obj->camera_supplement.local_center[0],
                              obj->camera_supplement.local_center[1],
                              obj->camera_supplement.local_center[2]};
    float dimension_hwl[3] = {obj->size(2), obj->size(1), obj->size(0)};
    float rotation_y =
              theta_ray + static_cast<float>(obj->camera_supplement.alpha);
    if (rotation_y < -PI) {
      rotation_y += 2 * PI;
    } else if (rotation_y >= PI) {
      rotation_y -= 2 * PI;
    }

    // fill back results
    FillResults(object_center, dimension_hwl, rotation_y, camera2world_pose,
                theta_ray, obj);

    ++nr_transformed_obj;
  }
  return nr_transformed_obj > 0;
}

std::string SignleStageObstacleTransformer::Name() const {
  return "SignleStageObstacleTransformer";
}

// Register plugin.
REGISTER_OBSTACLE_TRANSFORMER(SignleStageObstacleTransformer);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
