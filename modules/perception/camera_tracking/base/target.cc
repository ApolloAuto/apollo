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
#include "modules/perception/camera_tracking/base/target.h"

#include <algorithm>
#include <map>

#include "cyber/common/log.h"
#include "modules/perception/common/algorithm/geometry/basic.h"
#include "modules/perception/common/camera/common/math_functions.h"
#include "modules/perception/common/camera/common/util.h"

namespace apollo {
namespace perception {
namespace camera {

int Target::global_track_id = 0;
int Target::Size() const { return static_cast<int>(tracked_objects.size()); }

void Target::Clear() { tracked_objects.clear(); }

TrackObjectPtr Target::operator[](int index) const { return get_object(index); }
TrackObjectPtr Target::get_object(int index) const {
  CHECK_GT(static_cast<int>(tracked_objects.size()), 0);
  CHECK_LT(index, static_cast<int>(tracked_objects.size()));
  CHECK_GE(index, -static_cast<int>(tracked_objects.size()));
  return tracked_objects[(index + tracked_objects.size()) %
                         tracked_objects.size()];
}
void Target::Add(TrackObjectPtr object) {
  if (tracked_objects.empty()) {
    start_ts = object->timestamp;
    id = Target::global_track_id++;
    type = object->object->sub_type;
  }
  object->object->track_id = id;
  object->object->tracking_time = object->timestamp - start_ts;
  object->object->latest_tracked_time = object->timestamp;
  latest_object = object;
  lost_age = 0;
  tracked_objects.push_back(object);
}
void Target::RemoveOld(int frame_id) {
  size_t index = 0;
  while (index < tracked_objects.size() &&
         tracked_objects[index]->indicator.frame_id < frame_id) {
    ++index;
  }
  tracked_objects.erase(tracked_objects.begin(),
                        tracked_objects.begin() + index);
}
void Target::Init(const TargetParam &param) {
  target_param_ = param;
  id = -1;
  lost_age = 0;

  world_lwh.SetWindow(param.world_lhw_history());
  world_lwh_for_unmovable.SetWindow(param.world_lhw_history());
  image_wh.SetAlpha(param.image_wh_update_rate());
  // TODO(gaohan)  should update variance when predict and update
  image_center.variance_ *= target_param_.image_center().init_variance();
  image_center.measure_noise_ *=
      target_param_.image_center().measure_variance();
  image_center.process_noise_ *=
      target_param_.image_center().process_variance();
  world_center.variance_ *= target_param_.world_center().init_variance();
  world_center.process_noise_ *= 1;
  world_center.measure_noise_ *= 1;

  type_probs.assign(static_cast<int>(base::ObjectSubType::MAX_OBJECT_TYPE),
                    0.0f);
  world_center_for_unmovable.SetWindow(param.world_lhw_history());
  world_velocity.SetWindow(param.mean_filter_window());

  // record history state after kalman filter
  history_world_states_.set_capacity(param.world_state_history());

  // record orientation of displacement
  displacement_theta.SetWindow(param.mean_filter_window());
  direction.SetAlpha(param.direction_filter_ratio());
  // Init constant position Kalman Filter
  world_center_const.covariance_.setIdentity();
  world_center_const.measure_noise_.setIdentity();
  world_center_const.process_noise_.setIdentity();
  world_center_const.covariance_ *=
      target_param_.world_center().init_variance();
  // Init object template
  object_template_manager_ = ObjectTemplateManager::Instance();
}
Target::Target(const TargetParam &param) { Init(param); }

void Target::Predict(CameraTrackingFrame *frame) {
  auto delta_t =
      static_cast<float>(frame->timestamp - latest_object->timestamp);
  if (delta_t < 0) {
    return;
  }
  image_center.Predict(delta_t);
  float acc_variance = target_param_.world_center().process_variance();
  float delta_t_2 = delta_t * delta_t;
  float pos_variance = 0.25f * acc_variance * delta_t_2 * delta_t_2;
  float vel_variance = acc_variance * delta_t_2;
  world_center.process_noise_(0, 0) = pos_variance;
  world_center.process_noise_(1, 1) = pos_variance;
  world_center.process_noise_(2, 2) = vel_variance;
  world_center.process_noise_(3, 3) = vel_variance;
  world_center.Predict(delta_t);

  // const position kalman predict
  world_center_const.process_noise_.setIdentity();
  world_center_const.process_noise_(0, 0) = vel_variance * delta_t_2;
  world_center_const.process_noise_(1, 1) =
      world_center_const.process_noise_(0, 0);
  world_center_const.Predict(delta_t);
}

void Target::Update2D(CameraTrackingFrame *frame) {
  // measurements
  auto obj = latest_object->object;
  float width = static_cast<float>(frame->data_provider->src_width());
  float height = static_cast<float>(frame->data_provider->src_height());
  base::RectF rect(latest_object->projected_box);
  base::Point2DF center = rect.Center();

  if (!isLost()) {
    Eigen::Vector2d measurement;
    measurement << rect.width, rect.height;
    image_wh.AddMeasure(measurement);
    measurement << center.x, center.y;
    image_center.Correct(measurement);

    // update 2d bbox size
    Eigen::Vector4d state;
    state = image_center.get_state();
    center.x = static_cast<float>(state(0));
    center.y = static_cast<float>(state(1));
    ADEBUG << "2d move:" << id << " " << state(2) << " " << state(3);

    auto shape = image_wh.get_state();
    rect.width = static_cast<float>(shape(0));
    rect.height = static_cast<float>(shape(1));
    rect.SetCenter(center);
    RefineBox(rect, width, height, &rect);
    latest_object->projected_box = rect;
  }
}

void Target::Update3D(CameraTrackingFrame *frame) {
  auto object = latest_object->object;
  if (!isLost()) {
    // camera coordinate
    Eigen::Vector2d z;
    z << std::sin(object->theta), std::cos(object->theta);
    direction.AddMeasure(z);
    z = direction.get_state();
    float theta = static_cast<float>(std::atan2(z[0], z[1]));
    AINFO << "dir " << id << " " << object->theta << " " << theta;
    object->theta = theta;
    object->direction[0] = static_cast<float>(cos(object->theta));
    object->direction[1] = static_cast<float>(sin(object->theta));
    object->direction[2] = 0;

    z << object->center(0), object->center(1);
    if (object->type == base::ObjectType::UNKNOWN_UNMOVABLE) {
      world_center_for_unmovable.AddMeasure(z);
    } else {
      // Eigen::Vector4d x = world_center.get_state();
      float obj_2_car_x = object->camera_supplement.local_center[0];
      float obj_2_car_z = object->camera_supplement.local_center[2];
      float obj_distance_to_main_car =
          obj_2_car_x * obj_2_car_x + obj_2_car_z * obj_2_car_z;
      float dis_err = target_param_.world_center().measure_variance() *
                      obj_distance_to_main_car;
      world_center.measure_noise_.setIdentity();
      world_center.measure_noise_ *= dis_err;
      world_center.Correct(z);
      ADEBUG << "Velocity: " << id << " " << z.transpose() << " "
             << world_center.get_state().transpose();

      // const position kalman correct
      world_center_const.measure_noise_.setIdentity();
      world_center_const.measure_noise_ *= dis_err;
      KalmanFilterConstState<2>::VectorNd pos_measure;
      pos_measure << object->center(0), object->center(1);
      world_center_const.Correct(z);
    }
  }

  if (object->type == base::ObjectType::UNKNOWN_UNMOVABLE) {
    const Eigen::VectorXd &x = world_center_for_unmovable.get_state();
    const Eigen::MatrixXd &var = world_center_for_unmovable.get_variance();
    object->center(0) = x(0);
    object->center(1) = x(1);
    object->center_uncertainty(0) = static_cast<float>(var(0));
    object->center_uncertainty(1) = static_cast<float>(var(1));
    object->velocity(0) = 0;
    object->velocity(1) = 0;
    object->velocity(2) = 0;
  } else {
    Eigen::Vector4d x = world_center.get_state();
    // TODO(gaohan02): refactor
    if (tracked_objects.size() > 10) {
      auto pose1 = get_object(-2)->object->center;
      auto pose2 = get_object(-10)->object->center;
      double ts1 = get_object(-2)->timestamp;
      double ts2 = get_object(-10)->timestamp;
      Eigen::Vector3d vel = (pose1 - pose2) / (ts1 - ts2);
      double speed1 = std::sqrt(vel(0) * vel(0) + vel(1) * vel(1));
      double speed2 = std::sqrt(x(2) * x(2) + x(3) * x(3));
      double ratio = (Equal(speed1, 0)) ? 0 : speed2 / speed1;
      ADEBUG << "Target: " << id << " " << vel.transpose() << " , "
             << x.block<2, 1>(2, 0).transpose();
      if (ratio > target_param_.too_large_velocity_ratio()) {
        world_center.MagicVelocity(vel);
        ADEBUG << "Velocity too large";
      } else if (ratio > target_param_.large_velocity_ratio()) {
        vel(0) = (x(2) + vel(0)) / 2;
        vel(1) = (x(3) + vel(1)) / 2;
        world_center.MagicVelocity(vel);
        ADEBUG << "Velocity large";
      } else {
        ADEBUG << "Velocity normal";
      }
    }

    x = world_center.get_state();
    double speed = std::sqrt(x(2) * x(2) + x(3) * x(3));
    const std::map<base::ObjectSubType, float> &kTypeSpeedLimit =
        object_template_manager_->TypeSpeedLimit();
    if (speed > kTypeSpeedLimit.at(object->sub_type)) {
      double ratio = kTypeSpeedLimit.at(object->sub_type) / speed;
      Eigen::Vector2d vel;
      vel << x(2) * ratio, x(3) * ratio;
      world_center.MagicVelocity(vel);
      x = world_center.get_state();
    }
    object->center(0) = x(0);
    object->center(1) = x(1);
    object->center_uncertainty.setConstant(0);
    object->center_uncertainty(0, 0) =
        static_cast<float>(world_center.variance_(0, 0));
    object->center_uncertainty(1, 1) =
        static_cast<float>(world_center.variance_(1, 1));
    object->velocity(0) = static_cast<float>(x(2));
    object->velocity(1) = static_cast<float>(x(3));
    object->velocity(2) = 0;
    world_velocity.AddMeasure(object->velocity.cast<double>());
    object->velocity_uncertainty = world_velocity.get_variance().cast<float>();
    if (speed > target_param_.velocity_threshold()) {
      object->direction(0) = static_cast<float>(x(2) / speed);
      object->direction(1) = static_cast<float>(x(3) / speed);
      object->direction(2) = 0;
      object->theta = static_cast<float>(std::atan2(x(3), x(2)));
    }
  }

  if (object->type != base::ObjectType::UNKNOWN_UNMOVABLE &&
      target_param_.clapping_velocity()) {
    // check displacement orientation
    bool stable_moving = false;
    double avg_vel_norm = world_velocity.get_state().norm();
    if (avg_vel_norm > target_param_.stable_moving_speed()) {
      stable_moving = true;
    } else if (tracked_objects.size() > 1) {
      auto prev_pos = get_object(-2)->object->center;
      auto displacement = object->center - prev_pos;
      Eigen::VectorXd measured_theta(1);
      measured_theta << std::atan2(displacement[1], displacement[0]);
      displacement_theta.AddMeasure(measured_theta);
      // if an object is moving, its displacement orientation should be
      // consistent
      stable_moving = displacement_theta.get_variance()(0, 0) <
                      target_param_.displacement_theta_var();
    }

    // check const position kalman residuals
    const auto &residual = world_center_const.residual_;
    if (!stable_moving &&
        residual(0) * residual(0) <
            4 * world_center_const.measure_noise_(0, 0) &&
        residual(1) * residual(1) <
            4 * world_center_const.measure_noise_(1, 1)) {
      object->velocity(0) = 0;
      object->velocity(1) = 0;
    }

    // check history states
    history_world_states_.push_back(object);
    ClappingTrackVelocity(object);
  }

  // debug velocity
  ADEBUG << "obj_speed--id: " << id << " " << object->velocity.head(2).norm();
}

void Target::Update(CameraTrackingFrame *frame) {
  auto object = latest_object->object;
  if (!isLost()) {
    // todo(zero): need to fix alpha
    float alpha = 0.01f;

    Eigen::Vector4d size_measurement;
    size_measurement << alpha, object->size(0), object->size(1),
        object->size(2);
    world_lwh.AddMeasure(size_measurement);
    world_lwh_for_unmovable.AddMeasure(size_measurement);
    // update 3d object size
    if (object->type == base::ObjectType::UNKNOWN_UNMOVABLE) {
      object->size =
          world_lwh_for_unmovable.get_state().block<3, 1>(1, 0).cast<float>();
    } else {
      object->size = world_lwh.get_state().block<3, 1>(1, 0).cast<float>();
    }
    ADEBUG << " size is " << world_lwh.get_state().transpose();
  }

  Update3D(frame);
}

void Target::UpdateType(CameraTrackingFrame *frame) {
  auto object = latest_object->object;
  if (!isLost()) {
    base::RectF rect(object->camera_supplement.box);
    // 1.  6mm: reliable z is 40. intrinsic is approximate 2000
    // 2. 12mm: reliable z is 80. intrinsic is approximate 4000
    // hf = H * f/z = H * 50
    const TemplateMap &kMidTemplateHWL =
        object_template_manager_->MidTemplateHWL();
    float alpha = gaussian(
        rect.height /
            (50 * (kMidTemplateHWL.at(object->sub_type).at(0) + 0.01f)),
        1.0f, target_param_.type_filter_var());
    alpha = std::max(0.01f, alpha);

    type_probs[static_cast<int>(object->sub_type)] += alpha;
    auto max_prob = std::max_element(type_probs.begin(), type_probs.end());
    auto index = static_cast<int>(std::distance(type_probs.begin(), max_prob));
    type = static_cast<base::ObjectSubType>(index);
    ADEBUG << "Target " << id << " change type from "
           << static_cast<int>(object->sub_type) << " to "
           << static_cast<int>(type);
    object->sub_type = type;

    Eigen::Vector4d size_measurement;
    size_measurement << alpha, object->size(0), object->size(1),
        object->size(2);
    world_lwh.AddMeasure(size_measurement);
    world_lwh_for_unmovable.AddMeasure(size_measurement);
    // update 3d object size
    if (object->type == base::ObjectType::UNKNOWN_UNMOVABLE) {
      object->size =
          world_lwh_for_unmovable.get_state().block<3, 1>(1, 0).cast<float>();
    } else {
      object->size = world_lwh.get_state().block<3, 1>(1, 0).cast<float>();
    }
    ADEBUG << " size is " << world_lwh.get_state().transpose();
  }
}

void Target::ClappingTrackVelocity(const base::ObjectPtr &obj) {
  // check angle between velocity and heading(orientation)
  if (obj->type == base::ObjectType::VEHICLE) {
    Eigen::Vector3f obj_dir = obj->direction;
    Eigen::Vector3f obj_vel = obj->velocity;
    double vel_heading_angle_change_1 =
        algorithm::CalculateTheta2DXY(obj_dir, obj_vel);
    obj_dir *= -1;  // obj's heading may flip
    double vel_heading_angle_change_2 =
        algorithm::CalculateTheta2DXY(obj_dir, obj_vel);
    double vel_heading_angle_change =
        std::min(std::fabs(vel_heading_angle_change_1),
                 std::fabs(vel_heading_angle_change_2));
    ADEBUG << "obj_id: " << obj->track_id
           << " vel_heading_angle, 1: " << vel_heading_angle_change_1
           << " 2: " << vel_heading_angle_change_2
           << " final: " << vel_heading_angle_change;
    if (vel_heading_angle_change >
        target_param_.abnormal_velocity_heading_angle_threshold()) {
      ADEBUG << "omt set zero velocity because vel_heading_angle_change >"
                " abnormal_velocity_heading_angle_threshold : "
             << target_param_.abnormal_velocity_heading_angle_threshold();
      obj->velocity = Eigen::Vector3f::Zero();
      return;
    }
  }

  // check static
  if (CheckStatic()) {
    ADEBUG << "omt set zero velocity because of small speed.";
    obj->velocity = Eigen::Vector3f::Zero();
    return;
  }
}

/*
 * 1. check small speed
 * 2. check moved distance
 * 3. check velocity theta's variance
 */
bool Target::CheckStatic() {
  if (static_cast<int>(history_world_states_.size()) <
      target_param_.min_cached_world_state_history_size()) {
    return false;
  }

  // 1. Check small speed
  bool small_speed = false;
  const int min_vel_size =
      std::min(static_cast<int>(history_world_states_.size()),
               target_param_.min_cached_velocity_size());
  // calculate average velocity
  std::shared_ptr<base::Object> tmp_obj_vel_avg = nullptr;
  tmp_obj_vel_avg.reset(new base::Object);
  tmp_obj_vel_avg->velocity = Eigen::Vector3f(0, 0, 0);
  tmp_obj_vel_avg =
      std::accumulate(history_world_states_.begin() +
                          history_world_states_.size() - min_vel_size,
                      history_world_states_.end(), tmp_obj_vel_avg,
                      [](const std::shared_ptr<base::Object> obj1,
                         const std::shared_ptr<base::Object> obj2) {
                        std::shared_ptr<base::Object> ret_obj = nullptr;
                        ret_obj.reset(new base::Object);
                        ret_obj->velocity = obj1->velocity + obj2->velocity;
                        return ret_obj;
                      });
  tmp_obj_vel_avg->velocity /= static_cast<float>(min_vel_size);
  double speed_avg = tmp_obj_vel_avg->velocity.head(2).norm();
  const auto &obj_type = history_world_states_.back()->type;
  // check speed by type
  if (obj_type == base::ObjectType::PEDESTRIAN) {
    small_speed = speed_avg < target_param_.static_speed_threshold_ped();
  } else {
    small_speed = speed_avg < target_param_.static_speed_threshold();
  }
  if (small_speed) {
    return true;
  }

  // 2. Check small moved distance
  bool not_move = false;
  if (static_cast<int>(history_world_states_.size()) >=
      std::max(target_param_.min_cached_position_size(),
               target_param_.calc_avg_position_window_size())) {
    double move_distance = 0.0;
    Eigen::Vector3d start_position(0.0, 0.0, 0.0);
    Eigen::Vector3d end_position(0.0, 0.0, 0.0);
    const int start_idx =
        static_cast<int>(history_world_states_.size()) -
        static_cast<int>(target_param_.min_cached_position_size());
    const int end_idx = static_cast<int>(history_world_states_.size()) - 1;
    // calculate window-averaged start and end positions
    // and calculate moved distance
    if (end_idx - start_idx >= target_param_.calc_avg_position_window_size() &&
        start_idx + target_param_.calc_avg_position_window_size() - 1 <
            static_cast<int>(history_world_states_.size()) &&
        end_idx - target_param_.calc_avg_position_window_size() + 1 >= 0) {
      for (int i = 0; i < target_param_.calc_avg_position_window_size(); ++i) {
        start_position += history_world_states_[start_idx + i]->center;
        end_position += history_world_states_[end_idx - i]->center;
      }
      double time_diff = history_world_states_[end_idx]->latest_tracked_time -
                         history_world_states_[start_idx]->latest_tracked_time;
      // do not consider moved distance when time_diff is small
      if (std::fabs(time_diff) > 1e-3) {
        start_position /= target_param_.calc_avg_position_window_size();
        end_position /= target_param_.calc_avg_position_window_size();
        move_distance = (end_position - start_position).head(2).norm();
        // check moving speed by type
        double avg_speed = move_distance / time_diff;
        if (obj_type == base::ObjectType::PEDESTRIAN) {
          not_move = avg_speed < target_param_.min_moving_avg_speed_ped();
        } else {
          not_move = avg_speed < target_param_.min_moving_avg_speed();
        }
      }
    }
  }
  if (not_move) {
    return true;
  }

  // 3. Check velocity theta's variance
  std::vector<double> theta_vec(min_vel_size);
  std::transform(history_world_states_.begin() + history_world_states_.size() -
                     min_vel_size,
                 history_world_states_.end(), theta_vec.begin(),
                 [](const std::shared_ptr<base::Object> obj) -> double {
                   return std::atan2(obj->velocity[1], obj->velocity[0]);
                 });
  double mean = 0.0;
  double var = 0.0;
  CalculateMeanAndVariance(theta_vec, &mean, &var);
  double stddev = std::sqrt(var);
  if (stddev > target_param_.velocity_theta_var()) {  // 28.64 degree
    return true;
  }
  return false;
}

bool Target::isTracked() const {
  return Size() >= target_param_.tracked_life();
}
bool Target::isLost() const { return lost_age > 0; }

}  // namespace camera
}  // namespace perception
}  // namespace apollo
