/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/evaluator/vehicle/vectornet_evaluator.h"

#include <limits>
#include <omp.h>

#include "Eigen/Dense"

#include "cyber/common/file.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_map.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/common/prediction_util.h"
#include "modules/prediction/evaluator/warm_up/warm_up.h"

namespace apollo {
namespace prediction {

using apollo::common::TrajectoryPoint;
using apollo::common::math::Vec2d;
using apollo::prediction::VectorNet;

VectornetEvaluator::VectornetEvaluator() : device_(torch::kCPU) {
  evaluator_type_ = ObstacleConf::VECTORNET_EVALUATOR;
  LoadModel();
}

void VectornetEvaluator::Clear() {}

bool VectornetEvaluator::VectornetProcessObstaclePosition(
                             Obstacle* obstacle_ptr,
                             ObstaclesContainer* obstacles_container,
                             torch::Tensor* ptr_target_obs_pos,
                             torch::Tensor* ptr_target_obs_pos_step,
                             torch::Tensor* ptr_vector_mask,
                             torch::Tensor* ptr_obstacle_data,
                             torch::Tensor* ptr_all_obs_p_id) {
  // Extract features of pos_history
  std::vector<std::pair<double, double>> target_pos_history(20, {0.0, 0.0});
  std::vector<std::pair<double, double>> all_obs_length;
  std::vector<std::vector<std::pair<double, double>>> all_obs_pos_history;

  if (!ExtractObstaclesHistory(obstacle_ptr, obstacles_container,
                               &target_pos_history, &all_obs_length,
                               &all_obs_pos_history, ptr_vector_mask)) {
    ADEBUG << "Obstacle [" << obstacle_ptr->id()
           << "] failed to extract obstacle history";
    return false;
  }

  for (int j = 0; j < 20; ++j) {
    ptr_target_obs_pos->index_put_({19 - j, 0}, target_pos_history[j].first);
    ptr_target_obs_pos->index_put_({19 - j, 1}, target_pos_history[j].second);
    if (j == 19 || (j > 0 && target_pos_history[j + 1].first == 0.0)) {
      break;
    }
    ptr_target_obs_pos_step->index_put_(
        {19 - j, 0},
        target_pos_history[j].first - target_pos_history[j + 1].first);
    ptr_target_obs_pos_step->index_put_(
        {19 - j, 1},
        target_pos_history[j].second - target_pos_history[j + 1].second);
  }

  int obs_num =
      obstacles_container->curr_frame_considered_obstacle_ids().size();
  torch::Tensor all_obstacle_pos = torch::zeros({obs_num, 20, 2});
  torch::Tensor obs_length_data = torch::zeros({obs_num, 2});
  auto opts = torch::TensorOptions().dtype(torch::kDouble);

  for (int i = 0; i < obs_num; ++i) {
    std::vector<double> obs_p_id{std::numeric_limits<float>::max(),
                                 std::numeric_limits<float>::max()};
    for (int j = 0; j < 20; ++j) {
      // Process obs pid
      if (obs_p_id[0] > all_obs_pos_history[i][j].first) {
        obs_p_id[0] = all_obs_pos_history[i][j].first;
      }
      if (obs_p_id[1] > all_obs_pos_history[i][j].second) {
        obs_p_id[1] = all_obs_pos_history[i][j].second;
      }
      // Process obs pos history
      all_obstacle_pos.index_put_(
          {i, 19 - j, 0},
          all_obs_pos_history[i][j].first);
      all_obstacle_pos.index_put_(
          {i, 19 - j, 1},
          all_obs_pos_history[i][j].second);
    }

    ptr_all_obs_p_id->index_put_({i},
                                 torch::from_blob(obs_p_id.data(), {2}, opts));
    obs_length_data.index_put_({i, 0}, all_obs_length[i].first);
    obs_length_data.index_put_({i, 1}, all_obs_length[i].second);
  }

  // Extend obs data to specific dimension
  torch::Tensor obs_pos_data = torch::cat(
      {all_obstacle_pos.index(
           {torch::indexing::Slice(),
            torch::indexing::Slice(torch::indexing::None, -1),
            torch::indexing::Slice()}),
       all_obstacle_pos.index({torch::indexing::Slice(),
                               torch::indexing::Slice(1, torch::indexing::None),
                               torch::indexing::Slice()})},
      2);

  // Add obs attribute
  torch::Tensor obs_attr_agent =
      torch::tensor({11.0, 4.0}).unsqueeze(0).unsqueeze(0).repeat({1, 19, 1});
  torch::Tensor obs_attr_other =
      torch::tensor({10.0, 4.0}).unsqueeze(0).unsqueeze(0).repeat(
        {(obs_num - 1), 19, 1});
  torch::Tensor obs_attr = torch::cat({obs_attr_agent, obs_attr_other}, 0);
  // ADD obs id
  // add 500 to avoid same id as in map_info
  torch::Tensor obs_id =
      torch::arange(500,
                    obs_num + 500).unsqueeze(1).repeat({1, 19}).unsqueeze(2);
  // Process obs data
  obs_length_data = obs_length_data.unsqueeze(1).repeat({1, 19, 1});
  torch::Tensor obs_data_with_len =
      torch::cat({obs_pos_data, obs_length_data}, 2);

  torch::Tensor obs_data_with_attr =
      torch::cat({obs_data_with_len, obs_attr}, 2);

  torch::Tensor obs_data_with_id = torch::cat({obs_data_with_attr, obs_id}, 2);
  *ptr_obstacle_data =
      torch::cat({torch::zeros({obs_num, (50 - 19), 9}), obs_data_with_id}, 1);

  return true;
}

bool VectornetEvaluator::VectornetProcessMapData(
                             FeatureVector* map_feature,
                             PidVector* map_p_id,
                             const int obs_num,
                             torch::Tensor* ptr_map_data,
                             torch::Tensor* ptr_all_map_p_id,
                             torch::Tensor* ptr_vector_mask) {
  int map_polyline_num = map_feature->size();

  for (int i = 0; i < map_polyline_num && obs_num + i < 450; ++i) {
    size_t one_polyline_vector_size = map_feature->at(i).size();
    if (one_polyline_vector_size < 50) {
      ptr_vector_mask->index_put_({obs_num + i,
                                   torch::indexing::Slice(
                                       one_polyline_vector_size,
                                       torch::indexing::None)},
                                  1);
    }
  }

  auto opts = torch::TensorOptions().dtype(torch::kDouble);

  for (int i = 0; i < map_polyline_num && i + obs_num < 450; ++i) {
    ptr_all_map_p_id->index_put_({i}, torch::from_blob(map_p_id->at(i).data(),
                                                       {2},
                                                       opts));

    int one_polyline_vector_size = map_feature->at(i).size();
    for (int j = 0; j < one_polyline_vector_size && j < 50; ++j) {
      ptr_map_data->index_put_({i, j},
                               torch::from_blob(map_feature->at(i)[j].data(),
                                                {9},
                                                opts));
    }
  }
  *ptr_map_data = ptr_map_data->toType(at::kFloat);
  *ptr_all_map_p_id = ptr_all_map_p_id->toType(at::kFloat);

  return true;
}

bool VectornetEvaluator::Evaluate(Obstacle* obstacle_ptr,
                                  ObstaclesContainer* obstacles_container) {
  omp_set_num_threads(1);

  obstacle_ptr->SetEvaluatorType(evaluator_type_);

  Clear();
  CHECK_NOTNULL(obstacle_ptr);
  int id = obstacle_ptr->id();
  if (!obstacle_ptr->latest_feature().IsInitialized()) {
    AERROR << "Obstacle [" << id << "] has no latest feature.";
    return false;
  }

  int obs_num =
      obstacles_container->curr_frame_considered_obstacle_ids().size();

  auto start_time_obs = std::chrono::system_clock::now();

  torch::Tensor target_obstacle_pos = torch::zeros({20, 2});
  torch::Tensor target_obstacle_pos_step = torch::zeros({20, 2});
  torch::Tensor vector_mask = torch::zeros({450, 50});
  torch::Tensor obstacle_data = torch::zeros({obs_num, 20, 2});
  torch::Tensor all_obs_p_id = torch::zeros({obs_num, 2});
  // torch::Tensor obs_length_tmp = torch::zeros({obs_num, 2});

  if (!VectornetProcessObstaclePosition(obstacle_ptr,
                                       obstacles_container,
                                       &target_obstacle_pos,
                                       &target_obstacle_pos_step,
                                       &vector_mask,
                                       &obstacle_data,
                                       &all_obs_p_id)) {
    AERROR << "Obstacle [" << id << "] processing obstacle position fails.";
    return false;
  }

  auto end_time_obs = std::chrono::system_clock::now();
  std::chrono::duration<double> diff_obs = end_time_obs - start_time_obs;
  AINFO << "obstacle vectors used time: " << diff_obs.count() * 1000 << " ms.";

  Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
  CHECK_NOTNULL(latest_feature_ptr);

  // Query the map data
  FeatureVector map_feature;
  PidVector map_p_id;
  const double pos_x = latest_feature_ptr->position().x();
  const double pos_y = latest_feature_ptr->position().y();
  common::PointENU center_point
     = common::util::PointFactory::ToPointENU(pos_x, pos_y);;
  const double heading = latest_feature_ptr->velocity_heading();

  auto start_time_query = std::chrono::system_clock::now();

  if (!vector_net_.query(center_point, heading, &map_feature, &map_p_id)) {
    return false;
  }

  auto end_time_query = std::chrono::system_clock::now();
  std::chrono::duration<double> diff_query = end_time_query - start_time_query;
  AINFO << "vectors query used time: " << diff_query.count() * 1000 << " ms.";

  // process map data & map p id & v_mask for map polyline
  int map_polyline_num = map_feature.size();
  int data_length =
      ((obs_num + map_polyline_num) < 450) ? (obs_num + map_polyline_num) : 450;

  // Process input tensor
  auto start_time_data_prep = std::chrono::system_clock::now();
  torch::Tensor map_data = torch::zeros({map_polyline_num, 50, 9});
  torch::Tensor all_map_p_id = torch::zeros({map_polyline_num, 2});

  if (!VectornetProcessMapData(&map_feature,
                               &map_p_id,
                               obs_num,
                               &map_data,
                               &all_map_p_id,
                               &vector_mask)) {
    AERROR << "Obstacle [" << id << "] processing map data fails.";
    return false;
  }

  // process p mask
  torch::Tensor polyline_mask = torch::zeros({450});
  if (data_length < 450) {
    polyline_mask.index_put_(
        {torch::indexing::Slice(data_length, torch::indexing::None)}, 1);
  }


  // Extend data & pid to specific demension
  torch::Tensor data_tmp = torch::cat({obstacle_data, map_data}, 0);
  torch::Tensor p_id_tmp = torch::cat({all_obs_p_id, all_map_p_id}, 0);
  torch::Tensor vector_data;
  torch::Tensor polyline_id;
  if (data_length < 450) {
    torch::Tensor data_zeros = torch::zeros({(450 - data_length), 50, 9});
    torch::Tensor p_id_zeros = torch::zeros({(450 - data_length), 2});
    vector_data = torch::cat({data_tmp, data_zeros}, 0);
    polyline_id = torch::cat({p_id_tmp, p_id_zeros}, 0);
  } else {
    vector_data = data_tmp.index({torch::indexing::Slice(0, 450)});
    polyline_id = p_id_tmp.index({torch::indexing::Slice(0, 450)});
  }

  // Empty rand mask as placeholder
  auto rand_mask = torch::zeros({450}).toType(at::kBool);
  // Change mask type to bool
  auto bool_vector_mask = vector_mask.toType(at::kBool);
  auto bool_polyline_mask = polyline_mask.toType(at::kBool);
  // Build input features for torch
  std::vector<torch::jit::IValue> torch_inputs;

  torch_inputs.push_back(c10::ivalue::Tuple::create(
      {std::move(target_obstacle_pos.unsqueeze(0).to(device_)),
       std::move(target_obstacle_pos_step.unsqueeze(0).to(device_)),
       std::move(vector_data.unsqueeze(0).to(device_)),
       std::move(bool_vector_mask.unsqueeze(0).to(device_)),
       std::move(bool_polyline_mask.unsqueeze(0).to(device_)),
       std::move(rand_mask.unsqueeze(0).to(device_)),
       std::move(polyline_id.unsqueeze(0).to(device_))}));

  auto end_time_data_prep = std::chrono::system_clock::now();
  std::chrono::duration<double> diff_data_prep =
      end_time_data_prep - start_time_data_prep;
  AINFO << "vectornet input tensor preparation used time: "
         << diff_data_prep.count() * 1000 << " ms.";

  auto start_time_inference = std::chrono::system_clock::now();

  at::Tensor torch_output_tensor = torch_default_output_tensor_;
  torch_output_tensor =
      torch_vehicle_model_.forward(torch_inputs).toTensor().to(torch::kCPU);

  auto end_time_inference = std::chrono::system_clock::now();
  std::chrono::duration<double> diff_inference =
      end_time_inference - start_time_inference;
  AINFO << "vectornet inference used time: " << diff_inference.count() * 1000
         << " ms.";

  // Get the trajectory
  auto start_time_output_process = std::chrono::system_clock::now();

  auto torch_output = torch_output_tensor.accessor<float, 3>();
  Trajectory* trajectory = latest_feature_ptr->add_predicted_trajectory();
  trajectory->set_probability(1.0);

  for (int i = 0; i < 30; ++i) {
    double prev_x = pos_x;
    double prev_y = pos_y;
    if (i > 0) {
      const auto& last_point = trajectory->trajectory_point(i - 1).path_point();
      prev_x = last_point.x();
      prev_y = last_point.y();
    }
    TrajectoryPoint* point = trajectory->add_trajectory_point();
    double dx = static_cast<double>(torch_output[0][i][0]);
    double dy = static_cast<double>(torch_output[0][i][1]);

    double heading = latest_feature_ptr->velocity_heading();
    Vec2d offset(dx, dy);
    Vec2d rotated_offset = offset.rotate(heading - (M_PI / 2));
    double point_x = pos_x + rotated_offset.x();
    double point_y = pos_y + rotated_offset.y();
    point->mutable_path_point()->set_x(point_x);
    point->mutable_path_point()->set_y(point_y);

    if (i < 10) {  // use origin heading for the first second
      point->mutable_path_point()->set_theta(
          latest_feature_ptr->velocity_heading());
    } else {
      point->mutable_path_point()->set_theta(
          std::atan2(trajectory->trajectory_point(i).path_point().y() -
                         trajectory->trajectory_point(i - 1).path_point().y(),
                     trajectory->trajectory_point(i).path_point().x() -
                         trajectory->trajectory_point(i - 1).path_point().x()));
    }
    point->set_relative_time(static_cast<double>(i) *
                             FLAGS_prediction_trajectory_time_resolution);
    if (i == 0) {
      point->set_v(latest_feature_ptr->speed());
    } else {
      double diff_x = point_x - prev_x;
      double diff_y = point_y - prev_y;
      point->set_v(std::hypot(diff_x, diff_y) /
                   FLAGS_prediction_trajectory_time_resolution);
    }
  }

  auto end_time_output_process = std::chrono::system_clock::now();
  std::chrono::duration<double> diff_output_process =
      end_time_output_process - start_time_output_process;
  AINFO << "vectornet output process used time: "
         << diff_output_process.count() * 1000 << " ms.";
  return true;
}

bool VectornetEvaluator::ExtractObstaclesHistory(
    Obstacle* obstacle_ptr, ObstaclesContainer* obstacles_container,
    std::vector<std::pair<double, double>>* target_pos_history,
    std::vector<std::pair<double, double>>* all_obs_length,
    std::vector<std::vector<std::pair<double, double>>>* all_obs_pos_history,
    torch::Tensor* vector_mask) {
  const Feature& obs_curr_feature = obstacle_ptr->latest_feature();
  double obs_curr_heading = obs_curr_feature.velocity_heading();
  std::pair<double, double> obs_curr_pos = std::make_pair(
      obs_curr_feature.position().x(), obs_curr_feature.position().y());
  // Extract target obstacle history
  for (std::size_t i = 0; i < obstacle_ptr->history_size() && i < 20; ++i) {
    const Feature& target_feature = obstacle_ptr->feature(i);
    if (!target_feature.IsInitialized()) {
      break;
    }
    target_pos_history->at(i) =
        WorldCoordToObjCoordNorth(
            std::make_pair(target_feature.position().x(),
                           target_feature.position().y()),
                           obs_curr_pos, obs_curr_heading);
  }
  all_obs_length->emplace_back(
      std::make_pair(obs_curr_feature.length(), obs_curr_feature.width()));
  all_obs_pos_history->emplace_back(*target_pos_history);

  // Extract other obstacles & convert pos to traget obstacle relative coord
  std::vector<std::pair<double, double>> pos_history(20, {0.0, 0.0});
  for (int id : obstacles_container->curr_frame_considered_obstacle_ids()) {
    Obstacle* obstacle = obstacles_container->GetObstacle(id);
    if (!obstacle) {
      continue;
    }
    int target_id = obstacle_ptr->id();
    if (id == target_id) {
      continue;
    }
    const Feature& other_obs_curr_feature = obstacle->latest_feature();
    all_obs_length->emplace_back(std::make_pair(
        other_obs_curr_feature.length(), other_obs_curr_feature.width()));

    size_t obs_his_size = obstacle->history_size();
    obs_his_size = obs_his_size <= 20 ? obs_his_size : 20;
    int cur_idx = all_obs_pos_history->size();
    // if cur_dix >= 450, index_put_ discards it automatically.
    if (obs_his_size > 1) {
      vector_mask->index_put_({cur_idx,
                               torch::indexing::Slice(torch::indexing::None,
                                                      -(obs_his_size - 1))},
                              1);
    } else {
      vector_mask->index_put_({cur_idx,
                               torch::indexing::Slice(torch::indexing::None,
                                                      -1)},
                              1);
    }

    for (size_t i = 0; i < obs_his_size; ++i) {
      const Feature& feature = obstacle->feature(i);
      if (!feature.IsInitialized()) {
        break;
      }
      pos_history[i] = WorldCoordToObjCoordNorth(
          std::make_pair(feature.position().x(), feature.position().y()),
          obs_curr_pos, obs_curr_heading);
    }
    all_obs_pos_history->emplace_back(pos_history);
  }
  return true;
}

void VectornetEvaluator::LoadModel() {
  if (FLAGS_use_cuda && torch::cuda::is_available()) {
    AINFO << "CUDA is available";
    device_ = torch::Device(torch::kCUDA);
    torch_vehicle_model_ =
        torch::jit::load(FLAGS_torch_vehicle_vectornet_file, device_);
  } else {
    torch_vehicle_model_ =
        torch::jit::load(FLAGS_torch_vehicle_vectornet_cpu_file, device_);
  }
  torch::set_num_threads(1);

  // Fake intput for the first frame
  torch::Tensor target_obstacle_pos = torch::randn({1, 20, 2});
  torch::Tensor target_obstacle_pos_step = torch::randn({1, 20, 2});
  torch::Tensor vector_data = torch::randn({1, 450, 50, 9});
  torch::Tensor vector_mask = torch::randn({1, 450, 50}) > 0.9;
  torch::Tensor polyline_mask = torch::randn({1, 450}) > 0.9;
  torch::Tensor rand_mask = torch::zeros({1, 450});
  torch::Tensor polyline_id = torch::randn({1, 450, 2});
  std::vector<torch::jit::IValue> torch_inputs;
  torch_inputs.push_back(c10::ivalue::Tuple::create(
      {std::move(target_obstacle_pos.to(device_)),
       std::move(target_obstacle_pos_step.to(device_)),
       std::move(vector_data.to(device_)), std::move(vector_mask.to(device_)),
       std::move(polyline_mask.to(device_)), std::move(rand_mask.to(device_)),
       std::move(polyline_id.to(device_))}));
  // warm up to avoid very slow first inference later
  WarmUp(torch_inputs, &torch_vehicle_model_, &torch_default_output_tensor_);
}

}  // namespace prediction
}  // namespace apollo
