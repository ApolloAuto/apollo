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

#include "modules/prediction/evaluator/vehicle/multi_agent_evaluator.h"

#include <limits>
#include <omp.h>

#include "Eigen/Dense"

#include "cyber/common/file.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/common/prediction_map.h"
#include "modules/prediction/common/prediction_util.h"
#include "modules/prediction/evaluator/warm_up/warm_up.h"

namespace apollo {
namespace prediction {

using apollo::common::math::Vec2d;
using apollo::common::math::InterpolateUsingLinearApproximation;

using apollo::prediction::VectorNet;
using apollo::perception::PerceptionObstacle;

MultiAgentEvaluator::MultiAgentEvaluator() : device_(Model::CPU) {
  evaluator_type_ = ObstacleConf::MULTI_AGENT_EVALUATOR;
  torch_default_output_tensor_ = torch::zeros({1, max_agent_num, 30, 2});
  model_manager_ = ModelManager();
  model_manager_.Init();
  LoadModel();
}

void MultiAgentEvaluator::Clear() {}

bool MultiAgentEvaluator::VectornetProcessObstaclePosition(
                             ObstaclesContainer* obstacles_container,
                             const ADCTrajectoryContainer* adc_trajectory_container,
                             std::vector<int>& prediction_obs_ids,
                             torch::Tensor* ptr_multi_obstacle_pos,
                             torch::Tensor* ptr_multi_obstacle_pos_step,
                             torch::Tensor* ptr_vector_mask,
                             torch::Tensor* ptr_all_obs_data,
                             torch::Tensor* ptr_all_obs_p_id,
                             torch::Tensor* ptr_multi_obstacle_position) {

  std::vector<std::vector<std::pair<double, double>>> multi_obstacle_pos;
  std::vector<std::vector<double>> multi_obstacle_position;
  std::vector<std::vector<std::pair<double, double>>> all_obs_pos_history;
  std::vector<std::pair<double, double>> all_obs_length;
  std::vector<std::pair<double, double>> adc_traj_curr_pos(30, {0.0, 0.0});
  std::vector<TrajectoryPoint> adc_traj_points;

  // Process ADC trajectory & Extract features of ADC trajectory
  if (with_planning_traj) {
    const auto& adc_traj = adc_trajectory_container->adc_trajectory();
    for (size_t i = 0; i < adc_traj.trajectory_point().size(); ++i) {
      adc_traj_points.emplace_back(adc_traj.trajectory_point(i));
    }
  }

  if (!ExtractObstaclesHistory(obstacles_container,
                               prediction_obs_ids,
                               &adc_traj_points,
                               &multi_obstacle_pos,
                               &multi_obstacle_position,
                               &all_obs_length,
                               &all_obs_pos_history,
                               &adc_traj_curr_pos,
                               ptr_vector_mask)) {
    ADEBUG << "Failed to extract obstacle history";
    return false;
  }

  // data convert for faster torch data convert
  std::vector<double> multi_obs_pos_reverse(obs_num * 20 * 2, 0.0);
  std::vector<double> multi_obs_pos_step(obs_num * 20 * 2, 0.0);
  std::vector<double> all_obs_pos_temp(obs_num * 20 * 2, 0.0);
  std::vector<double> all_obs_length_temp(obs_num * 2, 0.0);
  std::vector<double> multi_obs_position_temp(obs_num * 3, 0.0);
  std::vector<double> all_obs_p_id_temp(obs_num * 2, std::numeric_limits<float>::max());

  for (int i = 0; i < obs_num; ++i) {
    for (int j = 0; j < 20; ++j) {
      multi_obs_pos_reverse[i * 20 * 2 + (19 - j) * 2 + 0] =
        multi_obstacle_pos[i][j].first;
      multi_obs_pos_reverse[i * 20 * 2 + (19 - j) * 2 + 1] =
        multi_obstacle_pos[i][j].second;
      if (j == 19 || (j > 0 && multi_obstacle_pos[i][j + 1].first == 0.0)) {
        break;
      }
      multi_obs_pos_step[i * 20 * 2 + (19 - j) * 2 + 0] =
        multi_obstacle_pos[i][j].first - multi_obstacle_pos[i][j + 1].first;
      multi_obs_pos_step[i * 20 * 2 + (19 - j) * 2 + 1] =
        multi_obstacle_pos[i][j].second - multi_obstacle_pos[i][j + 1].second;
    }

    for (int j = 0; j < 20; ++j) {
      // Process obs pid
      // p_id is the minimum values of the start coordinates
      if (all_obs_p_id_temp[i * 2 + 0] > all_obs_pos_history[i][j].first) {
        all_obs_p_id_temp[i * 2 + 0] = all_obs_pos_history[i][j].first;
      }
      if (all_obs_p_id_temp[i * 2 + 1] > all_obs_pos_history[i][j].second) {
        all_obs_p_id_temp[i * 2 + 1] = all_obs_pos_history[i][j].second;
      }
      // Process obs pos history
      all_obs_pos_temp[i * 20 * 2 + (19 - j) * 2 + 0] =
        all_obs_pos_history[i][j].first;
      all_obs_pos_temp[i * 20 * 2 + (19 - j) * 2 + 1] =
        all_obs_pos_history[i][j].second;
    }

    // obs length
    all_obs_length_temp[i * 2 + 0] = all_obs_length[i].first;
    all_obs_length_temp[i * 2 + 1] = all_obs_length[i].second;
  
    // obs position
    multi_obs_position_temp[i * 3 + 0] = multi_obstacle_position[i][0];
    multi_obs_position_temp[i * 3 + 1] = multi_obstacle_position[i][1];
    multi_obs_position_temp[i * 3 + 2] = multi_obstacle_position[i][2];
  }
  
  torch::Tensor all_obs_pos_data = torch::zeros({obs_num, 20, 2});
  torch::Tensor all_obs_p_id_data = torch::zeros({obs_num, 2});
  torch::Tensor all_obs_length_data = torch::zeros({obs_num, 2});
  auto opts = torch::TensorOptions().dtype(torch::kDouble);

  *ptr_multi_obstacle_pos =
    torch::from_blob(multi_obs_pos_reverse.data(), {obs_num, 20, 2}, opts).toType(at::kFloat);
  *ptr_multi_obstacle_pos_step =
    torch::from_blob(multi_obs_pos_step.data(), {obs_num, 20, 2}, opts).toType(at::kFloat);
  *ptr_multi_obstacle_position =
    torch::from_blob(multi_obs_position_temp.data(), {obs_num, 3}, opts).toType(at::kFloat);
  all_obs_pos_data =
    torch::from_blob(all_obs_pos_temp.data(), {obs_num, 20, 2}, opts).toType(at::kFloat);
  all_obs_p_id_data =
    torch::from_blob(all_obs_p_id_temp.data(), {obs_num, 2}, opts).toType(at::kFloat);
  all_obs_length_data =
    torch::from_blob(all_obs_length_temp.data(), {obs_num, 2}, opts).toType(at::kFloat);

  // Extend obs data to specific dimension
  all_obs_pos_data = torch::cat(
      {all_obs_pos_data.index(
           {torch::indexing::Slice(),
            torch::indexing::Slice(torch::indexing::None, -1),
            torch::indexing::Slice()}),
       all_obs_pos_data.index({torch::indexing::Slice(),
                               torch::indexing::Slice(1, torch::indexing::None),
                               torch::indexing::Slice()})},
      2);

  // Add obs attribute
  torch::Tensor obs_attr_agent =
      torch::tensor({11.0, 4.0}).unsqueeze(0).unsqueeze(0).repeat(
        {1, 19, 1});
  torch::Tensor obs_attr_other =
      torch::tensor({10.0, 4.0}).unsqueeze(0).unsqueeze(0).repeat(
        {(obs_num - 1), 19, 1});
  torch::Tensor obs_attr = torch::cat({obs_attr_agent, obs_attr_other}, 0);
  // ADD obs id
  // add 401 to avoid same id as in map_info
  torch::Tensor obs_id =
      torch::arange(401,
                    obs_num + 401).unsqueeze(1).repeat({1, 19}).unsqueeze(2);
  // Process obs data
  all_obs_length_data = all_obs_length_data.unsqueeze(1).repeat({1, 19, 1});
  torch::Tensor obs_data_with_len =
      torch::cat({all_obs_pos_data, all_obs_length_data}, 2);

  torch::Tensor obs_data_with_attr =
      torch::cat({obs_data_with_len, obs_attr}, 2);

  torch::Tensor obs_data_with_id = torch::cat({obs_data_with_attr, obs_id}, 2);
  obs_data_with_id =
      torch::cat({torch::zeros({obs_num, (50 - 19), 9}), obs_data_with_id}, 1);

  // adc trajectory
  if (with_planning_traj) {
    std::vector<double> adc_traj_p_id{std::numeric_limits<float>::max(),
                                      std::numeric_limits<float>::max()};
    for (int j = 0; j < 30; ++j) {
      if (adc_traj_p_id[0] > adc_traj_curr_pos[j].first) {
        adc_traj_p_id[0] = adc_traj_curr_pos[j].first;
      }
      if (adc_traj_p_id[1] > adc_traj_curr_pos[j].second) {
        adc_traj_p_id[1] = adc_traj_curr_pos[j].second;
      }
    }
    torch::Tensor planning_p_id = torch::zeros({1, 2});
    planning_p_id.index_put_({0},
                            torch::from_blob(adc_traj_p_id.data(), {2}, opts));
    torch::Tensor planning_data = torch::zeros({30, 2});
    for (int j = 0; j < 30; ++j) {
      // Process obs pos history
      planning_data.index_put_(
          {j, 0},
          adc_traj_curr_pos[j].first);
      planning_data.index_put_(
          {j, 1},
          adc_traj_curr_pos[j].second);
    }
    planning_data = torch::cat(
        {planning_data.index(
          {torch::indexing::Slice(torch::indexing::None, -1),
              torch::indexing::Slice()}),
        planning_data.index(
          {torch::indexing::Slice(1, torch::indexing::None),
              torch::indexing::Slice()})},
        1);
    planning_data = planning_data.unsqueeze(0);

    // Add obs attribute
    torch::Tensor attr_planning =
        torch::tensor({0.0, 0.0, 12.0, 4.0}).unsqueeze(0).unsqueeze(0).repeat(
          {1, 29, 1});
    planning_data =
        torch::cat({planning_data, attr_planning}, 2);
    torch::Tensor planning_id =
        torch::tensor({400}).unsqueeze(1).repeat({1, 29}).unsqueeze(2);
    planning_data = torch::cat({planning_data, planning_id}, 2);
    planning_data =
        torch::cat({torch::zeros({1, (50 - 29), 9}), planning_data}, 1);
    ptr_vector_mask->index_put_({0,
                                torch::indexing::Slice(torch::indexing::None,-29)},
                                1);

    // planning data + obj data
    *ptr_all_obs_data = torch::cat({planning_data, obs_data_with_id}, 0);
    *ptr_all_obs_p_id = torch::cat({planning_p_id, all_obs_p_id_data}, 0);
    return true;
  }
  
  // planning data + obj data
  torch::Tensor planning_data = torch::zeros({1, 50, 9});
  torch::Tensor planning_p_id = torch::zeros({1, 2});
  ptr_vector_mask->index_put_({0,
                              torch::indexing::Slice(torch::indexing::None,-29)},
                              1);
  *ptr_all_obs_data = torch::cat({planning_data, obs_data_with_id}, 0);
  *ptr_all_obs_p_id = torch::cat({planning_p_id, all_obs_p_id_data}, 0);
  return true;
}

bool MultiAgentEvaluator::VectornetProcessMapData(
                             FeatureVector* map_feature,
                             PidVector* map_p_id,
                             const int other_obs_num,
                             torch::Tensor* ptr_map_data,
                             torch::Tensor* ptr_all_map_p_id,
                             torch::Tensor* ptr_vector_mask) {
  int map_polyline_num = map_feature->size();

  for (int i = 0; i < map_polyline_num && other_obs_num + i < 450; ++i) {
    size_t one_polyline_vector_size = map_feature->at(i).size();
    if (one_polyline_vector_size < 50) {
      ptr_vector_mask->index_put_({other_obs_num + i,
                                   torch::indexing::Slice(
                                       one_polyline_vector_size,
                                       torch::indexing::None)},
                                  1);
    }
  }

  auto opts = torch::TensorOptions().dtype(torch::kDouble);

  std::vector<double> all_map_p_id_temp(map_polyline_num * 2, 0.0);
  std::vector<double> map_data_temp(map_polyline_num * 50 * 9, 0.0);
  for (int i = 0; i < map_polyline_num && i + other_obs_num < 450; ++i) {
    all_map_p_id_temp[i * 2 + 0] = map_p_id->at(i)[0];
    all_map_p_id_temp[i * 2 + 1] = map_p_id->at(i)[1];

    int one_polyline_vector_size = map_feature->at(i).size();
    for (int j = 0; j < one_polyline_vector_size && j < 50; ++j) {
      for (int k = 0; k < 9; ++k) {
        map_data_temp[i * 50 * 9 + j * 9 + k] = 
          map_feature->at(i)[j][k];
      }
    }
  }

  *ptr_map_data = torch::from_blob(map_data_temp.data(),
    {map_polyline_num, 50, 9}, opts).toType(at::kFloat);
  *ptr_all_map_p_id = torch::from_blob(all_map_p_id_temp.data(),
    {map_polyline_num, 2}, opts).toType(at::kFloat);

  return true;
}

bool MultiAgentEvaluator::Evaluate(
  Obstacle* obstacle,
  ObstaclesContainer* obstacles_container) {
  const ADCTrajectoryContainer* adc_trajectory_container;
  Evaluate(adc_trajectory_container, obstacle, obstacles_container);
  return true;
}

bool MultiAgentEvaluator::Evaluate(
  const ADCTrajectoryContainer* adc_trajectory_container,
  Obstacle* obstacle,
  ObstaclesContainer* obstacles_container) {
  omp_set_num_threads(1);
  Clear();

  /* process the ids of obstacles to be evaluated */
  auto start_time_pre = std::chrono::system_clock::now();

  if (adc_trajectory_container == nullptr) {
    AINFO << "Null adc traj container.";
    with_planning_traj = false;
  }

  if (adc_trajectory_container->adc_trajectory().trajectory_point().size() < 1) {
    AINFO << "Adc traj points are not enough.";
    with_planning_traj = false;
  }

  std::vector<int> obs_ids =
    obstacles_container->curr_frame_considered_obstacle_ids();
  // add adc id
  obs_ids.insert(obs_ids.begin(), -1);
  std::vector<int> prediction_obs_ids;
  for (int id : obs_ids) {
    Obstacle* obstacle_ptr = obstacles_container->GetObstacle(id);
    if (!obstacle_ptr) {
      if (id == -1) {
        AERROR << "Can not obtain adc info in multi agent evaluator.";
        return false;
      }
      continue;
    }
    // do not care unknown obstacles to reduce the number of obstacles (< 50)
    if (id != -1 && (obstacle_ptr->type() == perception::PerceptionObstacle::UNKNOWN ||
                     obstacle_ptr->type() == perception::PerceptionObstacle::UNKNOWN_MOVABLE ||
                     obstacle_ptr->type() == perception::PerceptionObstacle::UNKNOWN_UNMOVABLE)) {
      continue;
    }
    // ignore the obj with histories less than 1s
    if (id != -1 && obstacle_ptr->history_size() < 10) {
      continue;
    }
    prediction_obs_ids.push_back(id);
  }
  obs_num = prediction_obs_ids.size();
  if (obs_num <= 0) {
    AERROR << "No obstacle to be evaluated in multi agent evaluator.";
    return false;
  } else if (obs_num > 50) {
    AERROR << "Number of obstacles to be evaluated is too large: " << obs_num;
  } else {
    AINFO << "Number of objects to be evaluated: " << obs_num;
  }
  // planning traj, not to be evaulated
  vector_obs_num = obs_num + 1;

  auto end_time_pre = std::chrono::system_clock::now();
  std::chrono::duration<double> diff_pre = end_time_pre - start_time_pre;
  AINFO << "Evaluator prepare used time: " << diff_pre.count() * 1000 << " ms.";
  /*************************************/

  /* process the obstacle history pos into vector */
  auto start_time_obs = std::chrono::system_clock::now();

  torch::Tensor multi_obstacle_pos = torch::zeros({obs_num, 20, 2});
  torch::Tensor multi_obstacle_pos_step = torch::zeros({obs_num, 20, 2});
  torch::Tensor obs_position = torch::zeros({obs_num, 3});
  torch::Tensor vector_mask = torch::zeros({450, 50});
  torch::Tensor obstacle_data = torch::zeros({vector_obs_num, 50, 9});
  torch::Tensor all_obs_p_id = torch::zeros({vector_obs_num, 2});

  if (!VectornetProcessObstaclePosition(obstacles_container,
                                       adc_trajectory_container,
                                       prediction_obs_ids,
                                       &multi_obstacle_pos,
                                       &multi_obstacle_pos_step,
                                       &vector_mask,
                                       &obstacle_data,
                                       &all_obs_p_id,
                                       &obs_position)) {
    AERROR << "Processing obstacle position fails.";
    return false;
  }

  auto end_time_obs = std::chrono::system_clock::now();
  std::chrono::duration<double> diff_obs = end_time_obs - start_time_obs;
  AINFO << "Obstacle vectors used time: " << diff_obs.count() * 1000 << " ms.";
  /*************************************/

  /* Query the map data */
  auto start_time_query = std::chrono::system_clock::now();

  Obstacle* selected_obstacle = obstacles_container->GetObstacle(-1);
  Feature* latest_feature_ptr = selected_obstacle->mutable_latest_feature();
  CHECK_NOTNULL(latest_feature_ptr);
  
  FeatureVector map_feature;
  PidVector map_p_id;
  const double pos_x = latest_feature_ptr->position().x();
  const double pos_y = latest_feature_ptr->position().y();
  common::PointENU center_point
    = common::util::PointFactory::ToPointENU(pos_x, pos_y);;
  const double heading = latest_feature_ptr->velocity_heading();

  if (!vector_net_.query(center_point, heading, &map_feature, &map_p_id)) {
    return false;
  }

  auto end_time_query = std::chrono::system_clock::now();
  std::chrono::duration<double> diff_query = end_time_query - start_time_query;
  AINFO << "map vector query used time: " << diff_query.count() * 1000 << " ms.";
  /*************************************/

  /* process map data & map p id & v_mask for map polyline */
  auto start_time_map_vectorize = std::chrono::system_clock::now();

  int map_polyline_num = map_feature.size();
  int data_length =
      ((vector_obs_num + map_polyline_num) < 450) ? (vector_obs_num + map_polyline_num) : 450;

  // Process input tensor
  torch::Tensor map_data = torch::zeros({map_polyline_num, 50, 9});
  torch::Tensor all_map_p_id = torch::zeros({map_polyline_num, 2});

  if (!VectornetProcessMapData(&map_feature,
                               &map_p_id,
                               vector_obs_num,
                               &map_data,
                               &all_map_p_id,
                               &vector_mask)) {
    AERROR << "Processing map data fails.";
    return false;
  }

  auto end_time_map_vectorize = std::chrono::system_clock::now();
  std::chrono::duration<double> diff_map_vectorize =
      end_time_map_vectorize - start_time_map_vectorize;
  AINFO << "Map vectors used time: "
         << diff_map_vectorize.count() * 1000 << " ms.";
  /*************************************/

  /* prepare input data for inference */
  auto start_time_data_prep = std::chrono::system_clock::now();
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

  if (obs_num < max_agent_num) {
    torch::Tensor filling_zeros = torch::zeros({max_agent_num - obs_num, 20, 2});
    multi_obstacle_pos = torch::cat({multi_obstacle_pos, filling_zeros}, 0);
    multi_obstacle_pos_step = torch::cat({multi_obstacle_pos_step, filling_zeros}, 0);
    torch::Tensor position_zeros = torch::zeros({max_agent_num - obs_num, 3});
    obs_position = torch::cat({obs_position, position_zeros}, 0);
  } else {
    multi_obstacle_pos = multi_obstacle_pos.index({torch::indexing::Slice(0, max_agent_num)});
    multi_obstacle_pos_step = multi_obstacle_pos_step.index({torch::indexing::Slice(0, max_agent_num)});
    obs_position = obs_position.index({torch::indexing::Slice(0, max_agent_num)});
  }

  // Empty rand mask as placeholder
  auto rand_mask = torch::zeros({450}).to(torch::kBool);
  // Change mask type to bool
  auto bool_vector_mask = vector_mask.to(torch::kBool);
  auto bool_polyline_mask = polyline_mask.to(torch::kBool);

  // Build input features for torch
  std::vector<void*> input_buffers{
    multi_obstacle_pos.unsqueeze(0).data_ptr<float>(),
    multi_obstacle_pos_step.unsqueeze(0).data_ptr<float>(),
    vector_data.unsqueeze(0).data_ptr<float>(),
    bool_vector_mask.unsqueeze(0).data_ptr<bool>(),
    bool_polyline_mask.unsqueeze(0).data_ptr<bool>(),
    rand_mask.unsqueeze(0).data_ptr<bool>(),
    polyline_id.unsqueeze(0).data_ptr<float>(),
    obs_position.unsqueeze(0).data_ptr<float>()
  };

  std::vector<void*> output_buffers{
    torch_default_output_tensor_.data_ptr<float>()};

  auto end_time_data_prep = std::chrono::system_clock::now();
  std::chrono::duration<double> diff_data_prep =
      end_time_data_prep - start_time_data_prep;
  AINFO << "vectornet input tensor preparation used time: "
         << diff_data_prep.count() * 1000 << " ms.";
  /*************************************/

  /* Inference*/
  auto start_time_inference = std::chrono::system_clock::now();

  if (obstacle->IsPedestrian()) {
    if (!model_manager_.SelectModel(
          device_, ObstacleConf::MULTI_AGENT_EVALUATOR,
          apollo::perception::PerceptionObstacle::PEDESTRIAN)->Inference(
            input_buffers, 8, &output_buffers, 1)) {
      return false;
    }
  } else {
    if (!model_manager_.SelectModel(
          device_, ObstacleConf::MULTI_AGENT_EVALUATOR,
          apollo::perception::PerceptionObstacle::VEHICLE)->Inference(
            input_buffers, 8, &output_buffers, 1)) {
      return false;
    }
  }
  at::Tensor torch_output_tensor = torch::from_blob(
      output_buffers[0], {1, max_agent_num, 30, 2});

  auto end_time_inference = std::chrono::system_clock::now();
  std::chrono::duration<double> diff_inference =
      end_time_inference - start_time_inference;
  AINFO << "vectornet inference used time: " << diff_inference.count() * 1000
         << " ms.";
  /*************************************/

  /* post process to get the trajectory */
  auto start_time_output_process = std::chrono::system_clock::now();

  auto torch_output = torch_output_tensor.accessor<float, 4>();

  // The first obstacle is the ego vehicle
  // So the index starts from 1
  int obj_id = 1;
  for (int id : prediction_obs_ids) {
    if (id == -1) {
      continue;
    }
    Obstacle* obstacle_ptr = obstacles_container->GetObstacle(id);
    if (obstacle_ptr->IsStill() || (obstacle_ptr->type() != obstacle->type())) {
      obj_id++;
      continue;
    }

    obstacle_ptr->SetEvaluatorType(evaluator_type_);
    Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
    Trajectory* trajectory = latest_feature_ptr->add_predicted_trajectory();
    trajectory->set_probability(1.0);
    for (int i = 0; i < 30; ++i) {
      double prev_x = latest_feature_ptr->position().x();
      double prev_y = latest_feature_ptr->position().y();
      if (i > 0) {
        const auto& last_point = trajectory->trajectory_point(i - 1).path_point();
        prev_x = last_point.x();
        prev_y = last_point.y();
      }
      TrajectoryPoint* point = trajectory->add_trajectory_point();
      double dx = static_cast<double>(torch_output[0][obj_id][i][0]);
      double dy = static_cast<double>(torch_output[0][obj_id][i][1]);

      double heading = latest_feature_ptr->velocity_heading();
      Vec2d offset(dx, dy);
      Vec2d rotated_offset = offset.rotate(heading - (M_PI / 2));
      double point_x = latest_feature_ptr->position().x() + rotated_offset.x();
      double point_y = latest_feature_ptr->position().y() + rotated_offset.y();
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
    obj_id++;
  }

  auto end_time_output_process = std::chrono::system_clock::now();
  std::chrono::duration<double> diff_output_process =
      end_time_output_process - start_time_output_process;
  AINFO << "vectornet output process used time: "
         << diff_output_process.count() * 1000 << " ms.";
  /*************************************/

  return true;
}

bool MultiAgentEvaluator::ExtractObstaclesHistory(
    ObstaclesContainer* obstacles_container,
    std::vector<int>& prediction_obs_ids,
    std::vector<TrajectoryPoint>* trajectory_points,
    std::vector<std::vector<std::pair<double, double>>>* multi_obstacle_pos,
    std::vector<std::vector<double>>* multi_obstacle_position,
    std::vector<std::pair<double, double>>* all_obs_length,
    std::vector<std::vector<std::pair<double, double>>>* all_obs_pos_history,
    std::vector<std::pair<double, double>>* adc_traj_curr_pos,
    torch::Tensor* vector_mask) {

  std::vector<double> adc_world_coord;
  std::pair<double, double> adc_world_pos;

  // v_mask
  int cur_idx = 1;

  // all the obstacles
  for (int id : prediction_obs_ids) {
    Obstacle* obstacle_ptr = obstacles_container->GetObstacle(id);
    const Feature& obs_curr_feature = obstacle_ptr->latest_feature();
    
    std::vector<double> ref_world_coord;
    ref_world_coord.emplace_back(obs_curr_feature.position().x());
    ref_world_coord.emplace_back(obs_curr_feature.position().y());
    ref_world_coord.emplace_back(obs_curr_feature.velocity_heading());

    if (id == -1) {
      adc_world_coord = ref_world_coord;
      adc_world_pos = std::make_pair(
          adc_world_coord[0], adc_world_coord[1]);
    }

    // multi obs
    std::pair<double, double> obs_position_pair = WorldCoordToObjCoordNorth(
      std::make_pair(ref_world_coord[0],
                    ref_world_coord[1]),
                    adc_world_pos, adc_world_coord[2]);

    std::vector<double> obs_position;
    obs_position.emplace_back(obs_position_pair.first);
    obs_position.emplace_back(obs_position_pair.second);
    obs_position.emplace_back(ref_world_coord[2] - adc_world_coord[2]);

    double obs_curr_heading = obs_curr_feature.velocity_heading();
    std::pair<double, double> obs_curr_pos = std::make_pair(
        obs_curr_feature.position().x(), obs_curr_feature.position().y());

    std::vector<std::pair<double, double>> target_pos_history(20, {0.0, 0.0});
    std::vector<std::pair<double, double>> obs_pos_history(20, {0.0, 0.0});
    for (std::size_t i = 0; i < obstacle_ptr->history_size() && i < 20; ++i) {
      const Feature& target_feature = obstacle_ptr->feature(i);
      if (!target_feature.IsInitialized()) {
        break;
      }
      target_pos_history[i] =
          WorldCoordToObjCoordNorth(
              std::make_pair(target_feature.position().x(),
                             target_feature.position().y()),
                             obs_curr_pos, obs_curr_heading);
      obs_pos_history[i] =
          WorldCoordToObjCoordNorth(
              std::make_pair(target_feature.position().x(),
                             target_feature.position().y()),
                             adc_world_pos, adc_world_coord[2]);
    }
    multi_obstacle_pos->emplace_back(target_pos_history);
    all_obs_pos_history->emplace_back(obs_pos_history);
    all_obs_length->emplace_back(std::make_pair(
        obs_curr_feature.length(), obs_curr_feature.width()));
    multi_obstacle_position->emplace_back(obs_position);

    size_t obs_his_size = obstacle_ptr->history_size();
    obs_his_size = obs_his_size <= 20 ? obs_his_size : 20;
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
    cur_idx++;
  }

  // adc trajectory
  if (with_planning_traj) {
    adc_traj_curr_pos->resize(30, {0.0, 0.0});
    size_t adc_traj_points_num = trajectory_points->size();
    for (size_t i = 0; i < 30; ++i) {
      if (i > adc_traj_points_num -1) {
        adc_traj_curr_pos->at(i) =
            adc_traj_curr_pos->at(adc_traj_points_num - 1);
      } else {
        adc_traj_curr_pos->at(i) = WorldCoordToObjCoordNorth(
            std::make_pair(trajectory_points->at(i).path_point().x(),
            trajectory_points->at(i).path_point().y()),
            adc_world_pos, adc_world_coord[2]);
      }
    }
  }

  return true;
}

void MultiAgentEvaluator::LoadModel() {
  if (FLAGS_use_cuda && torch::cuda::is_available()) {
    device_ = Model::GPU;
  }

  auto multi_agent_pedestrian_model_ptr = model_manager_.SelectModel(
    device_, ObstacleConf::MULTI_AGENT_EVALUATOR,
    apollo::perception::PerceptionObstacle::PEDESTRIAN);

  auto multi_agent_vehicle_model_ptr = model_manager_.SelectModel(
    device_, ObstacleConf::MULTI_AGENT_EVALUATOR,
    apollo::perception::PerceptionObstacle::VEHICLE);

  ACHECK(multi_agent_pedestrian_model_ptr->Init());
  ACHECK(multi_agent_vehicle_model_ptr->Init());
  AERROR << "Load Multi agent model success.";
}

}  // namespace prediction
}  // namespace apollo
