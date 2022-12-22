/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/camera/lib/obstacle/tracker/omt2/omt_bev_tracker.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/camera/common/math_functions.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/common/geometry/common.h"

namespace apollo {
namespace perception {
namespace camera {

using cyber::common::GetAbsolutePath;

bool OMTBEVTracker::Init(const StageConfig& stage_config) {
  if (!Initialize(stage_config)) {
    return false;
  }

  omt_param_ = stage_config.omt_param();
  AINFO << "Load omt parameters: " << omt_param_.DebugString();

  frame_list_.Init(omt_param_.img_capability());
  gpu_id_ = omt_param_.gpu_id();
  // todo(zero): need to cacl similar by detection model output feature
  // similar_map_.Init(omt_param_.img_capability(), gpu_id_);
  // similar_.reset(new GPUSimilar);

  // todo(zero): do we need to adjust object size by reference
  // base::BaseCameraModelPtr model =
  //     common::SensorManager::Instance()->GetUndistortCameraModel(
  //         omt_param_.camera_name());
  // width_ = model->get_width();
  // height_ = model->get_height();
  // reference_.Init(omt_param_.reference(), width_, height_);

  // Read type change cost table
  std::string type_change_cost =
      GetAbsolutePath(omt_param_.root_dir(), omt_param_.type_change_cost());
  std::ifstream fin(type_change_cost);
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

  // Init object template
  object_template_manager_ = ObjectTemplateManager::Instance();
  return true;
}

bool OMTBEVTracker::Predict(const ObstacleTrackerOptions& options,
                            CameraFrame* frame) {
  for (auto& target : targets_) {
    target.Predict(frame);
    // todo(zero): proposed_objects not used by detection model
    auto obj = target.latest_object;
    frame->proposed_objects.push_back(obj->object);
  }
  return true;
}

float OMTBEVTracker::ScoreAppearance(const Target& target,
                                     TrackObjectPtr object) {
  // todo(zero): need tracking feature to cacl
  return 0.0f;
}

float OMTBEVTracker::ScoreMotion(const Target& target, TrackObjectPtr object) {
  Eigen::Vector4d state = target.world_center.get_state();
  float target_center_x = static_cast<float>(state[0]);
  float target_center_y = static_cast<float>(state[1]);

  // In normal projected_box is image coor, but in bev is world coor.
  base::Point2DF object_center = object->projected_box.Center();
  base::RectF rect(object->projected_box);
  float score = gaussian(object_center.x, target_center_x, rect.width) *
                gaussian(object_center.y, target_center_y, rect.height);
  return score;
}

float OMTBEVTracker::ScoreShape(const Target& target, TrackObjectPtr object) {
  Eigen::VectorXd shape = target.world_lwh.get_state();
  base::RectF rect(object->projected_box);

  float score =
      static_cast<float>((shape[2] - rect.width) * (shape[1] - rect.height) /
                         (shape[2] * shape[1]));

  return -std::abs(score);
}

float OMTBEVTracker::ScoreOverlap(const Target& target, TrackObjectPtr object) {
  Eigen::Vector4d state = target.world_center.get_state();
  Eigen::VectorXd shape = target.world_lwh.get_state();
  base::BBox2DF box_target;
  double x = state[0];
  double y = state[1];
  double w = shape[2];
  double l = shape[1];
  box_target.xmin = static_cast<float>(x - w / 2);
  box_target.xmax = static_cast<float>(x + w / 2);
  box_target.ymin = static_cast<float>(y - l / 2);
  box_target.ymax = static_cast<float>(y + l / 2);

  base::BBox2DF& box_object = object->projected_box;

  float iou = common::CalculateIOUBBox(box_target, box_object);
  return iou;
}

void OMTBEVTracker::GenerateHypothesis(const TrackObjectPtrs& objects,
                                       std::vector<HypothesisBev>* score_list) {
  HypothesisBev hypo;
  for (size_t i = 0; i < targets_.size(); ++i) {
    for (size_t j = 0; j < objects.size(); ++j) {
      hypo.target_idx = static_cast<int>(i);
      hypo.object_idx = static_cast<int>(j);

      // todo(zero): need to complete ScoreAppearance
      float sa = ScoreAppearance(targets_[i], objects[j]);
      float sm = ScoreMotion(targets_[i], objects[j]);
      float ss = ScoreShape(targets_[i], objects[j]);
      float so = ScoreOverlap(targets_[i], objects[j]);

      // todo(zero): always sa == 0 for now.
      if (sa == 0) {
        hypo.score = omt_param_.weight_diff_camera().motion() * sm +
                     omt_param_.weight_diff_camera().shape() * ss +
                     omt_param_.weight_diff_camera().overlap() * so;
      } else {
        hypo.score = (omt_param_.weight_same_camera().appearance() * sa +
                      omt_param_.weight_same_camera().motion() * sm +
                      omt_param_.weight_same_camera().shape() * ss +
                      omt_param_.weight_same_camera().overlap() * so);
      }

      int change_from_type = static_cast<int>(targets_[i].type);
      int change_to_type = static_cast<int>(objects[j]->object->sub_type);
      hypo.score -= kTypeAssociatedCost_[change_from_type][change_to_type];

      ADEBUG << "Detection " << objects[j]->indicator.frame_id << "(" << j
             << ") sa:" << sa << " sm: " << sm << " ss: " << ss << " so: " << so
             << " score: " << hypo.score;

      if (sm < 0.01 || hypo.score < omt_param_.target_thresh()) {
        continue;
      }

      score_list->push_back(hypo);
    }
  }
}

int OMTBEVTracker::CreateNewTarget(const TrackObjectPtrs& objects) {
  const TemplateMap& kMinTemplateHWL =
      object_template_manager_->MinTemplateHWL();
  std::vector<base::RectF> target_rects;
  for (const auto& target : targets_) {
    if (!target.isTracked() || target.isLost()) {
      continue;
    }
    base::RectF target_rect(target[-1]->projected_box);
    target_rects.push_back(target_rect);
  }
  int created_count = 0;
  for (size_t i = 0; i < objects.size(); ++i) {
    // todo(zero): OutOfValidRegion
    bool is_covered = false;
    base::RectF rect(objects[i]->projected_box);
    for (auto& target_rect : target_rects) {
      if (IsCovered(rect, target_rect, 0.4f) ||
          IsCoveredHorizon(rect, target_rect, 0.5f)) {
        is_covered = true;
        break;
      }
    }

    if (is_covered) {
      continue;
    }

    const auto& sub_type = objects[i]->object->sub_type;
    const std::vector<float>* min_tmplt = nullptr;

    if (kMinTemplateHWL.count(sub_type)) {
      min_tmplt = &kMinTemplateHWL.at(sub_type);
    }
    if (min_tmplt == nullptr ||
        rect.height > min_tmplt->at(2) * omt_param_.min_init_height_ratio()) {
      Target target(omt_param_.target_param());
      target.Add(objects[i]);
      targets_.push_back(target);
      AINFO << "Target " << target.id << " is created by "
            << objects[i]->indicator.frame_id << " ("
            << objects[i]->indicator.patch_id << ")";
      created_count += 1;
    }
  }
  return created_count;
}

void OMTBEVTracker::ClearTargets() {
  int l = 0;
  int r = static_cast<int>(targets_.size()) - 1;

  while (l <= r) {
    if (targets_[l].Size() == 0) {
      if (targets_[r].Size() != 0) {
        targets_[l] = targets_[r];
      }
      --r;
    } else {
      ++l;
    }
  }
  targets_.erase(targets_.begin() + l, targets_.end());
}

bool OMTBEVTracker::CombineDuplicateTargets() {
  std::vector<HypothesisBev> score_list;
  HypothesisBev hypo;
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
          // todo(zero): Remove duplicate objects in multi-camera
          auto box1 = p1->projected_box;
          auto box2 = p2->projected_box;
          score += common::CalculateIOUBBox(box1, box2);
          base::RectF rect1(box1);
          base::RectF rect2(box2);
          score -= std::abs((rect1.width - rect2.width) *
                            (rect1.height - rect2.height) /
                            (rect1.width * rect1.height));
          ++count;
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
      hypo.target_idx = static_cast<int>(i);
      hypo.object_idx = static_cast<int>(j);
      hypo.score = (count > 0) ? score / static_cast<float>(count) : 0;
      if (hypo.score < omt_param_.target_combine_iou_threshold()) {
        continue;
      }
      score_list.push_back(hypo);
    }
  }

  std::sort(score_list.begin(), score_list.end(),
            std::greater<HypothesisBev>());
  bool used_target[targets_.size()] = {false};
  for (const auto& pair : score_list) {
    if (used_target[pair.target_idx] || used_target[pair.object_idx]) {
      continue;
    }
    int index1 = pair.target_idx;
    int index2 = pair.object_idx;
    if (targets_[pair.target_idx].id > targets_[pair.object_idx].id) {
      index1 = pair.object_idx;
      index2 = pair.target_idx;
    }
    Target& target_save = targets_[index1];
    Target& target_del = targets_[index2];
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
    used_target[pair.object_idx] = true;
    used_target[pair.target_idx] = true;
  }
  return true;
}

void OMTBEVTracker::FindAbnormalTrack(TrackObjectPtrs* track_objects_ptr) {
  // mismatch may lead to an abnormal movement
  // if an abnormal movement is found, remove old target and create new one
  for (auto& target : targets_) {
    if (target.isLost() || target.Size() == 1) {
      continue;
    }

    auto obj = target[-1]->object;
    if (obj->type != base::ObjectType::UNKNOWN_UNMOVABLE) {
      Eigen::VectorXd x = target.world_center.get_state();
      double move = sqr(x[0] - obj->center[0]) + sqr(x[1] - obj->center[1]);
      float obj_2_car_x = obj->camera_supplement.local_center[0];
      float obj_2_car_y = obj->camera_supplement.local_center[2];
      float dis = sqr(obj_2_car_x) + sqr(obj_2_car_y);
      if (move > sqr(omt_param_.abnormal_movement()) * dis) {
        track_objects_ptr->push_back(target.latest_object);
        target.Clear();
      }
    }
  }
  ClearTargets();
}

bool OMTBEVTracker::Process(DataFrame* data_frame) {
  if (data_frame == nullptr) {
    return false;
  }

  CameraFrame* camera_frame = data_frame->camera_frame;
  ObstacleTrackerOptions tracker_options;

  auto track_state = data_frame->camera_frame->track_state;

  switch (track_state) {
    case TrackState::Predict: {
      Predict(tracker_options, camera_frame);
      data_frame->camera_frame->track_state = TrackState::Associate2D;
      break;
    }
    case TrackState::Associate2D: {
      Associate2D(tracker_options, camera_frame);
      data_frame->camera_frame->track_state = TrackState::Associate3D;
      break;
    }
    case TrackState::Associate3D: {
      Associate3D(tracker_options, camera_frame);
      data_frame->camera_frame->track_state = TrackState::Track;
      break;
    }
    case TrackState::Track: {
      Track(tracker_options, camera_frame);
      data_frame->camera_frame->track_state = TrackState::Predict;
      break;
    }
    default:
      // do nothing
      break;
  }

  return true;
}

bool OMTBEVTracker::Init(const ObstacleTrackerInitOptions& options) {
  return true;
}

bool OMTBEVTracker::Track(const ObstacleTrackerOptions& options,
                          CameraFrame* frame) {
  return true;
}

bool OMTBEVTracker::Associate2D(const ObstacleTrackerOptions& options,
                                CameraFrame* frame) {
  return true;
}

bool OMTBEVTracker::Associate3D(const ObstacleTrackerOptions& options,
                                CameraFrame* frame) {
  if (frame == nullptr) return false;

  inference::CudaUtil::set_device_id(gpu_id_);
  frame_list_.Add(frame);
  // todo(zero): need to calc similar for objects in each frame
  // for (int t = 0; t < frame_list_.Size(); t++) {
  //   // todo(zero): frame_id equal to index?
  //   int frame1 = frame_list_[t]->frame_id;
  //   int frame2 = frame_list_[-1]->frame_id;
  //   similar_->Calc(frame_list_[frame1], frame_list_[frame2],
  //                  similar_map_.get(frame1, frame2).get());
  // }

  for (auto& target : targets_) {
    target.RemoveOld(frame_list_.OldestFrameId());
    ++target.lost_age;
  }

  TrackObjectPtrs track_objects;
  for (size_t i = 0; i < frame->detected_objects.size(); ++i) {
    TrackObjectPtr track_ptr(new TrackObject);
    track_ptr->object = frame->detected_objects[i];
    track_ptr->object->id = static_cast<int>(i);
    track_ptr->timestamp = frame->timestamp;
    track_ptr->indicator = PatchIndicator(frame->frame_id, static_cast<int>(i),
                                          frame->data_provider->sensor_name());

    // todo(zero): world coor！ OBB-box is better, but for now we use AABB-box
    base::BBox2DF box_target;

    double x = frame->detected_objects[i]->center[0];
    double y = frame->detected_objects[i]->center[1];
    double l = frame->detected_objects[i]->size[0];
    double w = frame->detected_objects[i]->size[1];
    box_target.xmin = static_cast<float>(x - w / 2);
    box_target.xmax = static_cast<float>(x + w / 2);
    box_target.ymin = static_cast<float>(y - l / 2);
    box_target.ymax = static_cast<float>(y + l / 2);
    track_ptr->projected_box = box_target;
    track_objects.push_back(track_ptr);
  }

  // todo(zero): reference_
  // reference_.CorrectSize(frame);

  std::vector<HypothesisBev> score_list;
  GenerateHypothesis(track_objects, &score_list);

  std::sort(score_list.begin(), score_list.end(),
            std::greater<HypothesisBev>());
  bool used_target[targets_.size()] = {false};
  bool used_object[track_objects.size()] = {false};
  for (const auto& hypo : score_list) {
    int target_idx = hypo.target_idx;
    int object_idx = hypo.object_idx;
    if (used_target[target_idx] || used_object[object_idx]) continue;

    Target& target = targets_[target_idx];
    TrackObjectPtr track_object = track_objects[object_idx];
    target.Add(track_object);
    used_target[target_idx] = true;
    used_object[object_idx] = true;
    ADEBUG << "Target " << target.id << " match "
           << track_object->indicator.frame_id << " (" << object_idx << ")"
           << "at " << hypo.score << " size: " << target.Size();
  }

  // Find untracked objects and create new targets
  TrackObjectPtrs untrack_objects;
  for (size_t i = 0; i < track_objects.size(); ++i) {
    if (!used_object[i]) untrack_objects.push_back(track_objects[i]);
  }
  int new_count = CreateNewTarget(untrack_objects);
  AINFO << "Create " << new_count << " new target";

  for (auto& target : targets_) {
    if (target.lost_age > omt_param_.reserve_age()) {
      AINFO << "Target " << target.id << " is lost";
      target.Clear();
    }
  }
  // todo(zero): Remove duplicate objects in different cameras
  // CombineDuplicateTargets();
  ClearTargets();

  // todo(zero): reference_.UpdateReference
  TrackObjectPtrs invalid_track_objects;
  FindAbnormalTrack(&invalid_track_objects);

  new_count = CreateNewTarget(invalid_track_objects);
  AINFO << "Create " << new_count << " new target";

  frame->tracked_objects.clear();
  for (Target& target : targets_) {
    target.Update(frame);
    if (!target.isLost()) {
      frame->tracked_objects.push_back(target[-1]->object);
      AERROR << "Target " << target.id
             << " velocity: " << target.world_center.get_state().transpose()
             << " % " << target.world_center.variance_.diagonal().transpose()
             << " % " << target[-1]->object->velocity.transpose();
    }
  }
  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
