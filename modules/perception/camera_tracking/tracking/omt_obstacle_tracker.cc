/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/camera_tracking/tracking/omt_obstacle_tracker.h"

#include <algorithm>
#include <functional>

#include "cyber/common/file.h"
#include "modules/perception/camera_tracking/feature_extract/tracking_feat_extractor.h"
#include "modules/perception/common/algorithm/geometry/common.h"
#include "modules/perception/common/camera/common/math_functions.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace camera {

bool OMTObstacleTracker::Init(const ObstacleTrackerInitOptions &options) {
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);
  if (!cyber::common::GetProtoFromFile(config_file, &omt_param_)) {
    AERROR << "Read config failed: " << config_file;
    return false;
  }
  track_id_ = 0;
  frame_num_ = 0;
  // frame list
  frame_buffer_.set_capacity(omt_param_.img_capability());
  // similar map
  gpu_id_ = options.gpu_id;
  similar_map_.Init(omt_param_.img_capability(), gpu_id_);
  similar_.reset(new GPUSimilar);
  // reference
  width_ = options.image_width;
  height_ = options.image_height;
  reference_.Init(omt_param_.reference(), width_, height_);
  // type change cost file
  std::string type_change_cost_file =
      GetConfigFile(options.config_path, omt_param_.type_change_cost_file());
  std::ifstream fin(type_change_cost_file);
  ACHECK(fin.is_open());
  kTypeAssociatedCost_.clear();
  int n_type = static_cast<int>(base::ObjectSubType::MAX_OBJECT_TYPE);
  for (int i = 0; i < n_type; ++i) {
    kTypeAssociatedCost_.emplace_back(std::vector<float>(n_type, 0));
    for (int j = 0; j < n_type; ++j) {
      fin >> kTypeAssociatedCost_[i][j];
    }
  }
  targets_.clear();
  used_.clear();

  // reset object template
  object_template_manager_ = ObjectTemplateManager::Instance();

  // reset feature_extractor_;
  PluginParam plugin_param = omt_param_.plugin_param();
  feature_extractor_.reset(
      BaseFeatureExtractorRegisterer::GetInstanceByName(plugin_param.name()));
  if (nullptr == feature_extractor_) {
    AERROR << "Failed to get tracking instance.";
    return false;
  }

  // init feature_extractor_
  FeatureExtractorInitOptions feature_init_options;

  feature_init_options.input_height = omt_param_.feature_input_height();
  feature_init_options.input_width = omt_param_.feature_input_width();
  feature_init_options.gpu_id = gpu_id_;
  feature_init_options.config_path = plugin_param.config_path();
  feature_init_options.config_file = plugin_param.config_file();

  if (!feature_extractor_->Init(feature_init_options)) {
    AERROR << "feature_extracotr init false";
    return false;
  }

  // init object template
  ACHECK(ObjectTemplateManager::Instance()->Init());

  return true;
}

bool OMTObstacleTracker::Process(std::shared_ptr<CameraTrackingFrame> frame) {
  if (nullptr == frame) {
    return false;
  }
  if (!FeatureExtract(frame.get())) {
    AERROR << "FeatureExtract failed!";
    return false;
  }

  if (!Predict(frame.get())) {
    AERROR << "Predict failed!";
    return false;
  }

  if (!Associate2D(frame)) {
    AERROR << "Associate2D failed!";
    return false;
  }

  if (!Associate3D(frame)) {
    AERROR << "Associate3D failed!";
    return false;
  }
  return true;
}

bool OMTObstacleTracker::FeatureExtract(CameraTrackingFrame *frame) {
  FeatureExtractorOptions feature_options;
  feature_options.normalized = true;
  bool res = feature_extractor_->Extract(feature_options, frame);
  return res;
}

// todo(huqilin): need to optimize
// bool OMTObstacleTracker::CalSimilarity(CameraTrackingFrame *frame) {
//   frame_list_.Add(frame);
//   for (int t = 0; t < frame_list_.Size(); t++) {
//     int frame1 = frame_list_[t]->frame_id;
//     int frame2 = frame_list_[-1]->frame_id;
//     similar_->Calc(frame_list_[frame1], frame_list_[frame2],
//                    similar_map_.get(frame1, frame2).get());
//   }
//   for (auto &target : targets_) {
//     target.RemoveOld(frame_list_.OldestFrameId());
//     ++target.lost_age;
//   }
//   return true;
// }

bool OMTObstacleTracker::Predict(CameraTrackingFrame *frame) {
  for (auto &target : targets_) {
    // use kalman filter to predict the image_center,
    // world_center, world_center_const
    target.Predict(frame);
    auto obj = target.latest_object;
    frame->proposed_objects.push_back(obj->object);
  }
  return true;
}

// @description combine targets using iou after association
bool OMTObstacleTracker::CombineDuplicateTargets() {
  std::vector<Hypothesis> score_list;
  Hypothesis hypo;
  for (size_t i = 0; i < targets_.size(); ++i) {
    if (targets_[i].Size() == 0) {
      continue;
    }
    for (size_t j = i + 1; j < targets_.size(); ++j) {
      if (targets_[j].Size() == 0) {
        continue;
      }
      int count = 0;
      float score = 0.0f;
      int index1 = 0;
      int index2 = 0;
      while (index1 < targets_[i].Size() && index2 < targets_[j].Size()) {
        auto p1 = targets_[i][index1];
        auto p2 = targets_[j][index2];
        if (std::abs(p1->timestamp - p2->timestamp) <
            omt_param_.same_ts_eps()) {
          if (p1->indicator.sensor_name != p2->indicator.sensor_name) {
            auto box1 = p1->projected_box;
            auto box2 = p2->projected_box;
            score += algorithm::CalculateIOUBBox(box1, box2);
            base::RectF rect1(box1);
            base::RectF rect2(box2);
            score -= std::abs((rect1.width - rect2.width) *
                              (rect1.height - rect2.height) /
                              (rect1.width * rect1.height));
            count += 1;
          }
          ++index1;
          ++index2;
        } else {
          if (p1->timestamp > p2->timestamp) {
            ++index2;
          } else {
            ++index1;
          }
        }
      }
      ADEBUG << "Overlap: (" << targets_[i].id << "," << targets_[j].id
             << ") score " << score << " count " << count;
      hypo.target = static_cast<int>(i);
      hypo.object = static_cast<int>(j);
      hypo.score = (count > 0) ? score / static_cast<float>(count) : 0;
      if (hypo.score < omt_param_.target_combine_iou_threshold()) {
        continue;
      }
      score_list.push_back(hypo);
    }
  }
  sort(score_list.begin(), score_list.end(), std::greater<Hypothesis>());
  std::vector<bool> used_target(targets_.size(), false);
  for (auto &pair : score_list) {
    if (used_target[pair.target] || used_target[pair.object]) {
      continue;
    }
    int index1 = pair.target;
    int index2 = pair.object;
    if (targets_[pair.target].id > targets_[pair.object].id) {
      index1 = pair.object;
      index2 = pair.target;
    }
    Target &target_save = targets_[index1];
    Target &target_del = targets_[index2];
    for (int i = 0; i < target_del.Size(); i++) {
      // no need to change track_id of all objects in target_del
      target_save.Add(target_del[i]);
    }
    std::sort(
        target_save.tracked_objects.begin(), target_save.tracked_objects.end(),
        [](const TrackObjectPtr object1, const TrackObjectPtr object2) -> bool {
          return object1->indicator.frame_id < object2->indicator.frame_id;
        });
    target_save.latest_object = target_save.get_object(-1);
    base::ObjectPtr object = target_del.latest_object->object;
    target_del.Clear();
    AINFO << "Target " << target_del.id << " is merged into Target "
          << target_save.id << " with iou " << pair.score;
    used_target[pair.object] = true;
    used_target[pair.target] = true;
  }
  return true;
}

void OMTObstacleTracker::GenerateHypothesis(const TrackObjectPtrs &objects) {
  std::vector<Hypothesis> score_list;
  Hypothesis hypo;
  for (size_t i = 0; i < targets_.size(); ++i) {
    for (size_t j = 0; j < objects.size(); ++j) {
      hypo.target = static_cast<int>(i);
      hypo.object = static_cast<int>(j);
      // calculate the feature similarity between Target and object
      float sa = ScoreAppearance(targets_[i], objects[j]);
      // calculate the motion similarity between Target and object
      float sm = ScoreMotion(targets_[i], objects[j]);
      // calculate the shape similarity between Target and object
      float ss = ScoreShape(targets_[i], objects[j]);
      // calculate the overlap similarity (iou) between Target and object
      float so = ScoreOverlap(targets_[i], objects[j]);

      hypo.score = (omt_param_.weight_same_camera().appearance() * sa +
                    omt_param_.weight_same_camera().motion() * sm);

      int change_from_type = static_cast<int>(targets_[i].type);
      int change_to_type = static_cast<int>(objects[j]->object->sub_type);
      hypo.score += -kTypeAssociatedCost_[change_from_type][change_to_type];

      AINFO << "Detection " << objects[j]->indicator.frame_id << " object "
            << objects[j]->object->id << " "
            << "(" << j << ") sa:" << sa << " sm: " << sm << " ss: " << ss
            << " so: " << so << " score: " << hypo.score;

      // 95.44% area is range [mu - sigma*2, mu + sigma*2]
      // don't match if motion is beyond the range
      if (sm < 0.045 || hypo.score < omt_param_.target_thresh()) {
        continue;
      }
      score_list.push_back(hypo);
    }
  }

  sort(score_list.begin(), score_list.end(), std::greater<Hypothesis>());
  std::vector<bool> used_target(targets_.size(), false);
  for (auto &pair : score_list) {
    if (used_target[pair.target] || used_[pair.object]) {
      continue;
    }
    Target &target = targets_[pair.target];
    auto det_obj = objects[pair.object];
    target.Add(det_obj);
    used_[pair.object] = true;
    used_target[pair.target] = true;
    AINFO << "Target " << target.id << " match " << det_obj->indicator.frame_id
          << " (" << pair.object << ")"
          << "at " << pair.score << " size: " << target.Size();
  }
}

float OMTObstacleTracker::ScoreMotion(const Target &target,
                                      TrackObjectPtr track_obj) {
  Eigen::Vector4d x = target.image_center.get_state();
  float target_centerx = static_cast<float>(x[0]);
  float target_centery = static_cast<float>(x[1]);
  base::Point2DF center = track_obj->projected_box.Center();
  base::RectF rect(track_obj->projected_box);
  float s = gaussian(center.x, target_centerx, rect.width) *
            gaussian(center.y, target_centery, rect.height);
  return s;
}

float OMTObstacleTracker::ScoreShape(const Target &target,
                                     TrackObjectPtr track_obj) {
  Eigen::Vector2d shape = target.image_wh.get_state();
  base::RectF rect(track_obj->projected_box);
  float s = static_cast<float>((shape[1] - rect.height) *
                               (shape[0] - rect.width) / (shape[1] * shape[0]));
  return -std::abs(s);
}

float OMTObstacleTracker::ScoreAppearance(const Target &target,
                                          TrackObjectPtr track_obj) {
  float energy = 0.0f;
  int count = 0;
  // auto sensor_name = track_obj->indicator.sensor_name;
  for (int i = target.Size() - 1; i >= 0; --i) {
    // if (target[i]->indicator.sensor_name != sensor_name) {
    //   continue;
    // }
    PatchIndicator p1 = target[i]->indicator;
    PatchIndicator p2 = track_obj->indicator;

    energy += similar_map_.sim(p1, p2);
    count += 1;
  }

  return energy / (0.1f + static_cast<float>(count) * 0.9f);
}

// [new]
float OMTObstacleTracker::ScoreOverlap(const Target &target,
                                       TrackObjectPtr track_obj) {
  Eigen::Vector4d center = target.image_center.get_state();
  Eigen::VectorXd wh = target.image_wh.get_state();
  base::BBox2DF box_target;
  box_target.xmin = static_cast<float>(center[0] - wh[0] * 0.5);
  box_target.xmax = static_cast<float>(center[0] + wh[0] * 0.5);
  box_target.ymin = static_cast<float>(center[1] - wh[1] * 0.5);
  box_target.ymax = static_cast<float>(center[1] + wh[1] * 0.5);

  auto box_obj = track_obj->projected_box;

  float iou = algorithm::CalculateIOUBBox(box_target, box_obj);
  return iou;
}

void ProjectBox(const base::BBox2DF &box_origin,
                const Eigen::Matrix3d &transform,
                base::BBox2DF *box_projected) {
  Eigen::Vector3d point;
  //  top left
  point << box_origin.xmin, box_origin.ymin, 1;
  point = transform * point;
  box_projected->xmin = static_cast<float>(point[0] / point[2]);
  box_projected->ymin = static_cast<float>(point[1] / point[2]);
  //  bottom right
  point << box_origin.xmax, box_origin.ymax, 1;
  point = transform * point;
  box_projected->xmax = static_cast<float>(point[0] / point[2]);
  box_projected->ymax = static_cast<float>(point[1] / point[2]);
}

int OMTObstacleTracker::CreateNewTarget(const TrackObjectPtrs &objects) {
  const TemplateMap &kMinTemplateHWL =
      object_template_manager_->MinTemplateHWL();
  std::vector<base::RectF> target_rects;
  for (auto &&target : targets_) {
    if (!target.isTracked() || target.isLost()) {
      continue;
    }
    base::RectF target_rect(target[-1]->object->camera_supplement.box);
    target_rects.push_back(target_rect);
  }
  int created_count = 0;
  for (size_t i = 0; i < objects.size(); ++i) {
    if (!used_[i]) {
      bool is_covered = false;
      const auto &sub_type = objects[i]->object->sub_type;
      base::RectF rect(objects[i]->object->camera_supplement.box);
      auto &min_tmplt = kMinTemplateHWL.at(sub_type);
      // check if the object is out of valid region
      if (OutOfValidRegion(rect, width_, height_, omt_param_.border())) {
        AINFO << "Out of valid region";
        AINFO << "Rect x: " << rect.x << " Rect y: " << rect.y
              << " Rect height: " << rect.height
              << " Rect width: " << rect.width << " GT height_: " << height_
              << " GT width_: " << width_;
        continue;
      }
      for (auto &&target_rect : target_rects) {
        // check if rect is covered by target_rect
        if (IsCovered(rect, target_rect, 0.4f) ||
            IsCoveredHorizon(rect, target_rect, 0.5f)) {
          is_covered = true;
          break;
        }
      }
      if (is_covered) {
        continue;
      }
      if (min_tmplt.empty()  // unknown type
          || rect.height > min_tmplt[0] * omt_param_.min_init_height_ratio()) {
        Target target(omt_param_.target_param());
        target.Add(objects[i]);
        targets_.push_back(target);
        AINFO << "Target " << target.id << " is created by "
              << objects[i]->indicator.frame_id << " ("
              << objects[i]->indicator.patch_id << ")";
        created_count += 1;
      }
    }
  }
  return created_count;
}

bool OMTObstacleTracker::Associate2D(
    std::shared_ptr<CameraTrackingFrame> frame) {
  // pre-compute appearance similarities between frames
  inference::CudaUtil::set_device_id(gpu_id_);
  frame_buffer_.push_back(frame);
  // calculate the similarity between the current frame and the previous frames
  std::shared_ptr<CameraTrackingFrame> cur_frame_ptr = frame_buffer_.back();
  for (auto frame_ptr : frame_buffer_) {
    int pre_frame_id = frame_ptr->frame_id;
    int cur_frame_id = cur_frame_ptr->frame_id;
    similar_->Calc(frame_ptr.get(), cur_frame_ptr.get(),
                   similar_map_.get(pre_frame_id, cur_frame_id).get());
  }

  // remove oldest frame in Target
  for (auto &target : targets_) {
    target.RemoveOld(frame_buffer_.front()->frame_id);
    ++target.lost_age;
  }
  // clear non Target in targets_
  ClearTargets();
  // correct detection 3d size
  reference_.CorrectSize(frame.get());

  TrackObjectPtrs track_objects;
  for (size_t i = 0; i < frame->detected_objects.size(); ++i) {
    TrackObjectPtr track_ptr(new TrackObject);
    track_ptr->object = frame->detected_objects[i];
    track_ptr->object->id = static_cast<int>(i);
    track_ptr->timestamp = frame->timestamp;
    track_ptr->indicator = PatchIndicator(frame->frame_id, static_cast<int>(i),
                                          frame->data_provider->sensor_name());
    track_ptr->object->camera_supplement.sensor_name =
        frame->data_provider->sensor_name();
    ProjectBox(frame->detected_objects[i]->camera_supplement.box,
               frame->project_matrix, &(track_ptr->projected_box));
    track_objects.push_back(track_ptr);
  }
  // wxt todo: solve reference and calibration_service
  used_.clear();
  used_.resize(frame->detected_objects.size(), false);
  // generate hypothesis between existing Targets
  // and track_objects in the new frame
  GenerateHypothesis(track_objects);
  // create new Targets for track_objects that are not
  // associated with any existing Targets
  int new_count = CreateNewTarget(track_objects);
  AINFO << "Create " << new_count << " new target";

  for (auto &target : targets_) {
    // remove old Target
    if (target.lost_age > omt_param_.reserve_age()) {
      AINFO << "Target " << target.id << " is lost";
      target.Clear();
    } else {
      // update the sub_type and size of the Target
      target.UpdateType(frame.get());
      // update the 2D center, width and height of the Target
      target.Update2D(frame.get());
    }
  }

  // combine targets using iou after association
  CombineDuplicateTargets();

  // return filter reulst to original box
  Eigen::Matrix3d inverse_project = frame->project_matrix.inverse();
  for (auto &target : targets_) {
    if (!target.isLost()) {
      ProjectBox(target[-1]->projected_box, inverse_project,
                 &(target[-1]->object->camera_supplement.box));
      RefineBox(target[-1]->object->camera_supplement.box, width_, height_,
                &(target[-1]->object->camera_supplement.box));
    }
  }
  return true;
}

void OMTObstacleTracker::ClearTargets() {
  int left = 0;
  int end = static_cast<int>(targets_.size() - 1);
  while (left <= end) {
    if ((targets_[left].Size() == 0)) {
      while ((left < end) && (targets_[end].Size() == 0)) {
        --end;
      }
      if (left >= end) {
        break;
      }
      targets_[left] = targets_[end];
      --end;
    }
    ++left;
  }
  targets_.erase(targets_.begin() + left, targets_.end());
}

bool OMTObstacleTracker::Associate3D(
    std::shared_ptr<CameraTrackingFrame> frame) {
  reference_.UpdateReference(frame.get(), targets_);
  frame->tracked_objects.clear();
  TrackObjectPtrs track_objects;
  // mismatch may lead to an abnormal movement
  // if an abnormal movement is found, remove old target and create new one
  for (auto &target : targets_) {
    if (target.isLost() || target.Size() == 1) {
      continue;
    }
    auto obj = target[-1]->object;
    if (obj->type != base::ObjectType::UNKNOWN_UNMOVABLE) {
      Eigen::VectorXd x = target.world_center.get_state();
      double move = sqr(x[0] - obj->center[0]) + sqr(x[1] - obj->center[1]);
      float obj_2_car_x = obj->camera_supplement.local_center[0];
      float obj_2_car_y = obj->camera_supplement.local_center[2];
      float dis = obj_2_car_x * obj_2_car_x + obj_2_car_y * obj_2_car_y;
      if (move > sqr(omt_param_.abnormal_movement()) * dis) {
        AINFO << "Target " << target.id << " is removed for abnormal movement";
        track_objects.push_back(target.latest_object);
        target.Clear();
      }
    }
  }
  ClearTargets();
  used_.clear();
  used_.resize(track_objects.size(), false);
  int new_count = CreateNewTarget(track_objects);
  AINFO << "Create " << new_count << " new target";
  for (int j = 0; j < new_count; ++j) {
    targets_[targets_.size() - j - 1].Update2D(frame.get());
    targets_[targets_.size() - j - 1].UpdateType(frame.get());
  }
  for (Target &target : targets_) {
    target.Update3D(frame.get());
    if (!target.isLost()) {
      frame->tracked_objects.push_back(target[-1]->object);
      ADEBUG << "Target " << target.id
             << " velocity: " << target.world_center.get_state().transpose()
             << " % " << target.world_center.variance_.diagonal().transpose()
             << " % " << target[-1]->object->velocity.transpose();
    }
  }
  return true;
}

REGISTER_OBSTACLE_TRACKER(OMTObstacleTracker);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
