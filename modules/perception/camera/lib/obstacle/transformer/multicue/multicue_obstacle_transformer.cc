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
#include "modules/perception/camera/lib/obstacle/transformer/multicue/multicue_obstacle_transformer.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/camera/common/global_config.h"

namespace apollo {
namespace perception {
namespace camera {

bool MultiCueObstacleTransformer::Init(
    const ObstacleTransformerInitOptions &options) {
  std::string transformer_config =
      cyber::common::GetAbsolutePath(options.root_dir, options.conf_file);

  if (!cyber::common::GetProtoFromFile(transformer_config, &multicue_param_)) {
    AERROR << "Read config failed: " << transformer_config;
    return false;
  }
  AINFO << "Load transformer parameters from " << transformer_config
        << " \nmin dimension: " << multicue_param_.min_dimension_val()
        << " \ndo template search: " << multicue_param_.check_dimension();

  // Init object template
  object_template_manager_ = ObjectTemplateManager::Instance();

  return true;
}

void MultiCueObstacleTransformer::SetObjMapperOptions(
    base::ObjectPtr obj, Eigen::Matrix3f camera_k_matrix, int width_image,
    int height_image, ObjMapperOptions *obj_mapper_options, float *theta_ray) {
  // prepare bbox2d
  float bbox2d[4] = {
      obj->camera_supplement.box.xmin, obj->camera_supplement.box.ymin,
      obj->camera_supplement.box.xmax, obj->camera_supplement.box.ymax};
  // input insanity check
  bbox2d[0] = std::max(0.0f, bbox2d[0]);
  bbox2d[1] = std::max(0.0f, bbox2d[1]);
  bbox2d[2] = std::min(static_cast<float>(width_image) - 1.0f, bbox2d[2]);
  bbox2d[3] = std::min(static_cast<float>(height_image) - 1.0f, bbox2d[3]);
  bbox2d[2] = std::max(bbox2d[0] + 1.0f, bbox2d[2]);
  bbox2d[3] = std::max(bbox2d[1] + 1.0f, bbox2d[3]);

  // prepare dimension_hwl
  float dimension_hwl[3] = {obj->size(2), obj->size(1), obj->size(0)};

  // prepare rotation_y
  float box_cent_x = (bbox2d[0] + bbox2d[2]) / 2;
  Eigen::Vector3f image_point_low_center(box_cent_x, bbox2d[3], 1);
  Eigen::Vector3f point_in_camera =
      static_cast<Eigen::Matrix<float, 3, 1, 0, 3, 1>>(
          camera_k_matrix.inverse() * image_point_low_center);
  *theta_ray =
      static_cast<float>(atan2(point_in_camera.x(), point_in_camera.z()));
  float rotation_y =
      *theta_ray + static_cast<float>(obj->camera_supplement.alpha);
  base::ObjectSubType sub_type = obj->sub_type;

  // enforce rotation_y to be in the range [-pi, pi)
  const float PI = common::Constant<float>::PI();
  if (rotation_y < -PI) {
    rotation_y += 2 * PI;
  } else if (rotation_y >= PI) {
    rotation_y -= 2 * PI;
  }

  memcpy(obj_mapper_options->bbox, bbox2d, sizeof(float) * 4);
  memcpy(obj_mapper_options->hwl, dimension_hwl, sizeof(float) * 3);
  obj_mapper_options->ry = rotation_y;
  obj_mapper_options->is_veh = (obj->type == base::ObjectType::VEHICLE);
  obj_mapper_options->check_dimension = multicue_param_.check_dimension();
  obj_mapper_options->type_min_vol_index =
      MatchTemplates(sub_type, dimension_hwl);

  ADEBUG << "#2D-to-3D for one obj:";
  ADEBUG << "Obj pred ry:" << rotation_y;
  ADEBUG << "Obj pred type: " << static_cast<int>(sub_type);
  ADEBUG << "Bbox: " << bbox2d[0] << ", " << bbox2d[1] << ", " << bbox2d[2]
         << ", " << bbox2d[3];
}

int MultiCueObstacleTransformer::MatchTemplates(base::ObjectSubType sub_type,
                                                float *dimension_hwl) {
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
      if (min_dimension_val < multicue_param_.min_dimension_val()) {
        common::IScale3(dimension_hwl, multicue_param_.min_dimension_val() *
                                           common::IRec(min_dimension_val));
      }
      break;
  }
  return type_min_vol_index;
}

void MultiCueObstacleTransformer::FillResults(
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

  Eigen::Matrix3d pos_var = mapper_->get_position_uncertainty();
  obj->center_uncertainty(0) = static_cast<float>(pos_var(0));
  obj->center_uncertainty(1) = static_cast<float>(pos_var(1));
  obj->center_uncertainty(2) = static_cast<float>(pos_var(2));

  float theta = rotation_y;
  Eigen::Vector3d dir = (camera2world_pose.matrix().block(0, 0, 3, 3) *
                         Eigen::Vector3d(cos(theta), 0, -sin(theta)));
  obj->direction[0] = static_cast<float>(dir[0]);
  obj->direction[1] = static_cast<float>(dir[1]);
  obj->direction[2] = static_cast<float>(dir[2]);
  obj->theta = static_cast<float>(atan2(dir[1], dir[0]));
  obj->theta_variance = static_cast<float>((mapper_->get_orientation_var())(0));

  obj->camera_supplement.alpha = rotation_y - theta_ray;

  ADEBUG << "Dimension hwl: " << dimension_hwl[0] << ", " << dimension_hwl[1]
         << ", " << dimension_hwl[2];
  ADEBUG << "Obj ry:" << rotation_y;
  ADEBUG << "Obj theta: " << obj->theta;
  ADEBUG << "Obj center from transformer: " << obj->center.transpose();
}

bool MultiCueObstacleTransformer::Transform(
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

  const int width_image = frame->data_provider->src_width();
  const int height_image = frame->data_provider->src_height();
  const auto &camera2world_pose = frame->camera2world_pose;
  mapper_->Init(k_mat, width_image, height_image);

  ObjMapperOptions obj_mapper_options;
  float object_center[3] = {0};
  float dimension_hwl[3] = {0};
  float rotation_y = 0.0f;

  int nr_transformed_obj = 0;
  for (auto &obj : frame->detected_objects) {
    if (obj == nullptr) {
      ADEBUG << "Empty object input to transformer.";
      continue;
    }

    // set object mapper options
    float theta_ray = 0.0f;
    SetObjMapperOptions(obj, camera_k_matrix, width_image, height_image,
                        &obj_mapper_options, &theta_ray);

    // process
    mapper_->Solve3dBbox(obj_mapper_options, object_center, dimension_hwl,
                         &rotation_y);

    // fill back results
    FillResults(object_center, dimension_hwl, rotation_y, camera2world_pose,
                theta_ray, obj);

    ++nr_transformed_obj;
  }
  return nr_transformed_obj > 0;
}

std::string MultiCueObstacleTransformer::Name() const {
  return "MultiCueObstacleTransformer";
}

// Register plugin.
REGISTER_OBSTACLE_TRANSFORMER(MultiCueObstacleTransformer);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
