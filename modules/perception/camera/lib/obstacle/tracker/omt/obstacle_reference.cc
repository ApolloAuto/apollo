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
#include "modules/perception/camera/lib/obstacle/tracker/omt/obstacle_reference.h"

#include <algorithm>

#include "cyber/common/log.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/camera/lib/interface/base_calibration_service.h"

namespace apollo {
namespace perception {
namespace camera {

void ObstacleReference::Init(const omt::ReferenceParam &ref_param, float width,
                             float height) {
  ref_param_ = ref_param;
  img_width_ = width;
  img_height_ = height;
  ref_width_ = static_cast<int>(
      width / static_cast<float>(ref_param.down_sampling()) + 1);
  ref_height_ = static_cast<int>(
      height / static_cast<float>(ref_param.down_sampling()) + 1);
  static const float k =
      static_cast<float>(ref_height_) / static_cast<float>(ref_width_);
  static const float b = static_cast<float>(ref_height_);
  init_ref_map_.resize(ref_height_);
  for (int y = 0; y < ref_height_; ++y) {
    init_ref_map_[y].resize(ref_width_, -1);
    if (y < ref_height_ / 2 || y > ref_height_ - ref_param_.margin()) {
      continue;
    }
    for (int x = ref_param.margin(); x < ref_width_ - ref_param.margin(); ++x) {
      if (y > -k * static_cast<float>(x) + b && y > k * static_cast<float>(x)) {
        init_ref_map_[y][x] = 0;
      }
    }
  }
  object_template_manager_ = ObjectTemplateManager::Instance();
}

void ObstacleReference::UpdateReference(const CameraFrame *frame,
                                        const EigenVector<Target> &targets) {
  std::string sensor = frame->data_provider->sensor_name();
  SyncGroundEstimator(sensor, frame->camera_k_matrix,
                      static_cast<int>(img_width_),
                      static_cast<int>(img_height_));
  auto &ground_estimator = ground_estimator_mapper_[sensor];

  auto &refs = reference_[sensor];
  auto &ref_map = ref_map_[sensor];
  if (ref_map.empty()) {
    ref_map = init_ref_map_;
  }
  for (auto &&reference : refs) {
    reference.area *= ref_param_.area_decay();
  }
  for (auto &&target : targets) {
    if (!target.isTracked()) {
      continue;
    }
    if (target.isLost()) {
      continue;
    }
    auto obj = target[-1]->object;
    if (!Contain(object_template_manager_->TypeCanBeRef(), obj->sub_type)) {
      continue;
    }
    auto &box = obj->camera_supplement.box;

    if (box.ymax - box.ymin < ref_param_.min_allow_height()) {
      continue;
    }
    if (box.ymax < frame->camera_k_matrix(1, 2) + 1) {
      continue;
    }
    if (box.ymax >= img_height_ + 1) {
      AERROR << "box.ymax (" << box.ymax << ") is larger than img_height_ + 1 ("
             << img_height_ + 1 << ")";
      return;
    }
    if (box.xmax >= img_width_ + 1) {
      AERROR << "box.xmax (" << box.xmax << ") is larger than img_width_ + 1 ("
             << img_width_ + 1 << ")";
      return;
    }
    float x = box.Center().x;
    float y = box.ymax;

    AINFO << "Target: " << target.id
          << " can be Ref. Type: " << base::kSubType2NameMap.at(obj->sub_type);
    int y_discrete =
        static_cast<int>(y / static_cast<float>(ref_param_.down_sampling()));
    int x_discrete =
        static_cast<int>(x / static_cast<float>(ref_param_.down_sampling()));
    if (ref_map[y_discrete][x_discrete] == 0) {
      Reference r;
      r.area = box.Area();
      r.ymax = y;
      r.k = obj->size[2] / (y - box.ymin);
      refs.push_back(r);
      ref_map[y_discrete][x_discrete] = static_cast<int>(refs.size());
    } else if (ref_map[y_discrete][x_discrete] > 0) {
      int index = ref_map[y_discrete][x_discrete] - 1;
      if (box.Area() > refs[index].area) {
        refs[index].area = box.Area();
        refs[index].ymax = y;
        refs[index].k = obj->size[2] / (y - box.ymin);
      }
    }
  }

  // detect the ground from reference
  std::vector<float> vd_samples(static_cast<int>(refs.size() * 2), 0);
  int count_vd = 0;
  for (auto &&reference : refs) {
    float z_ref = reference.k * frame->camera_k_matrix(1, 1);
    vd_samples[count_vd++] = reference.ymax;
    vd_samples[count_vd++] = common::IRec(z_ref);
  }

  int count_samples = static_cast<int>(vd_samples.size() / 2);
  if (count_samples > ground_estimator.get_min_nr_samples()) {
    ground_estimator.DetetGround(
        frame->calibration_service->QueryPitchAngle(),
        frame->calibration_service->QueryCameraToGroundHeight(),
        vd_samples.data(), count_samples);
  }
}
void ObstacleReference::CorrectSize(CameraFrame *frame) {
  const TemplateMap &kMinTemplateHWL =
      object_template_manager_->MinTemplateHWL();
  const TemplateMap &kMaxTemplateHWL =
      object_template_manager_->MaxTemplateHWL();
  const std::vector<TemplateMap> &kTemplateHWL =
      object_template_manager_->TemplateHWL();

  for (auto &obj : frame->detected_objects) {
    float volume_object = obj->size[0] * obj->size[1] * obj->size[2];
    ADEBUG << "Det " << frame->frame_id << " (" << obj->id << ") "
           << "ori size:" << obj->size.transpose() << " "
           << "type: " << static_cast<int>(obj->sub_type) << " "
           << volume_object << " " << frame->data_provider->sensor_name();
    base::CameraObjectSupplement &supplement = obj->camera_supplement;

    if (Contain(object_template_manager_->TypeRefinedByTemplate(),
                obj->sub_type)) {
      float min_template_volume = 0.0f;
      float max_template_volume = 0.0f;
      auto min_tmplt = kMinTemplateHWL.at(obj->sub_type);
      auto max_tmplt = kMaxTemplateHWL.at(obj->sub_type);
      min_template_volume = min_tmplt[0] * min_tmplt[1] * min_tmplt[2];
      max_template_volume = max_tmplt[0] * max_tmplt[1] * max_tmplt[2];
      if (volume_object < min_template_volume ||
          obj->size[2] < (1 - ref_param_.height_diff_ratio()) * min_tmplt[0]) {
        obj->size[0] = min_tmplt[2];
        obj->size[1] = min_tmplt[1];
        obj->size[2] = min_tmplt[0];
      } else if (volume_object > max_template_volume ||
                 obj->size[2] >
                     (1 + ref_param_.height_diff_ratio()) * max_tmplt[0]) {
        obj->size[0] = max_tmplt[2];
        obj->size[1] = max_tmplt[1];
        obj->size[2] = max_tmplt[0];
      }
    }

    if (Contain(object_template_manager_->TypeRefinedByRef(), obj->sub_type)) {
      std::string sensor = frame->data_provider->sensor_name();
      std::vector<float> height;

      double z_obj = frame->camera_k_matrix(1, 1) * obj->size[2] /
                     (supplement.box.ymax - supplement.box.ymin);

      // h from ground detection
      SyncGroundEstimator(sensor, frame->camera_k_matrix,
                          static_cast<int>(img_width_),
                          static_cast<int>(img_height_));
      auto &ground_estimator = ground_estimator_mapper_[sensor];
      float l[3] = {0};
      if (ground_estimator.GetGroundModel(l)) {
        double z_ground_reversed =
            (-l[0] * supplement.box.ymax - l[2]) * common::IRec(l[1]);
        double z_ground = common::IRec(z_ground_reversed);
        z_ground = std::max(z_ground, z_obj);
        float k2 = static_cast<float>(z_ground / frame->camera_k_matrix(1, 1));
        float h = k2 * (supplement.box.ymax - supplement.box.ymin);
        h = std::max(std::min(h, kMaxTemplateHWL.at(obj->sub_type).at(0)),
                     kMinTemplateHWL.at(obj->sub_type).at(0));
        height.push_back(h);
      }

      // h from calibration service
      if (frame->calibration_service != nullptr) {
        double z_calib = 0.0;
        bool success = frame->calibration_service->QueryDepthOnGroundPlane(
            static_cast<int>(supplement.box.Center().x),
            static_cast<int>(supplement.box.ymax), &z_calib);
        if (success) {
          z_calib = std::max(z_calib, z_obj);
          float k2 = static_cast<float>(z_calib / frame->camera_k_matrix(1, 1));
          float h = k2 * (supplement.box.ymax - supplement.box.ymin);
          h = std::max(std::min(h, kMaxTemplateHWL.at(obj->sub_type).at(0)),
                       kMinTemplateHWL.at(obj->sub_type).at(0));
          height.push_back(h);
        } else {
          height.push_back(kMaxTemplateHWL.at(obj->sub_type).at(0));
        }
        Eigen::Vector4d plane_param;
        frame->calibration_service->QueryGroundPlaneInCameraFrame(&plane_param);
      }

      // h from history reference vote
      if (obj->sub_type != base::ObjectSubType::TRAFFICCONE || height.empty()) {
        float cy = frame->camera_k_matrix(1, 2);
        cy = std::min(supplement.box.ymax - 1, cy);
        for (auto &&reference : reference_[sensor]) {
          // pick out the refs close enough in y directions
          float dy = std::abs(reference.ymax - supplement.box.ymax);
          float scale_y = (supplement.box.ymax - cy) / (img_height_ - cy);
          float thres_dy =
              std::max(static_cast<float>(ref_param_.down_sampling()) * scale_y,
                       static_cast<float>(ref_param_.margin()) * 2.0f);
          if (dy < thres_dy) {
            float k2 = reference.k;
            k2 *= (reference.ymax - cy) / (supplement.box.ymax - cy);
            float h = k2 * (supplement.box.ymax - supplement.box.ymin);
            // bounded by templates
            h = std::max(std::min(h, kMaxTemplateHWL.at(obj->sub_type).at(0)),
                         kMinTemplateHWL.at(obj->sub_type).at(0));
            height.push_back(h);
          }
        }
        height.push_back(obj->size[2]);
      }

      if (height.empty()) {
        AERROR << "height vector is empty";
        continue;
      }
      std::sort(height.begin(), height.end());
      int nr_hs = static_cast<int>(height.size());
      float h_updated = height[nr_hs / 2];
      AINFO << "Estimate " << h_updated << " with " << height.size();
      obj->size[2] =
          std::min(h_updated, kMaxTemplateHWL.at(obj->sub_type).at(0));
      obj->size[2] =
          std::max(obj->size[2], kMinTemplateHWL.at(obj->sub_type).at(0));
      std::vector<float> height_error(kTemplateHWL.size(), 0);

      float obj_h = obj->size[2];
      for (size_t i = 0; i < height_error.size(); ++i) {
        height_error[i] =
            std::abs(kTemplateHWL[i].at(obj->sub_type).at(0) - obj_h);
      }
      auto min_error =
          std::min_element(height_error.begin(), height_error.end());
      auto min_index = std::distance(height_error.begin(), min_error);
      auto &tmplt = kTemplateHWL.at(min_index).at(obj->sub_type);
      float scale_factor = obj->size[2] / tmplt[0];
      obj->size[1] = tmplt[1] * scale_factor;
      obj->size[0] = tmplt[2] * scale_factor;
    }
    ADEBUG << "correct size:" << obj->size.transpose();
  }
}
}  // namespace camera
}  // namespace perception
}  // namespace apollo
