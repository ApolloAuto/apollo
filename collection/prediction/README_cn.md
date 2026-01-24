## 预测模块概述

预测模块是Apollo自动驾驶系统的重要组成部分，负责预测感知输出的障碍物未来的运动轨迹。

轨迹预测的上游模块是感知，定位，场景，地图，规划模块，预测模块通过消息队列进行数据和信息的获取：

**感知** ：预测模块必需的信息，提供障碍物的位置，长宽高等基本的特征。

**定位** ：获取本车的位置信息，并处理成序列，作为普通的一条轨迹，参与到障碍物的轨迹预测中。

**场景** ：获取场景信息，不同的场景对应不同的评估器和预器。

**地图** ：提供道路信息的编码。

**规划** ：规划信息，参与到障碍物车辆的轨迹预测中，部分场景下有效。

预测模块的下游模块是规划模块。预测内部有容器（Container），场景（Scenario）、评估器（Evaluator）和预测器（Predictor）等子模块。在容器内，获取相关模块的消息后，需要对各个模块的消息进行加工，feature.proto中定义了预测模块需要完成的数据结构；在评估器（Evaluator）和预测器（Predictor）中是行为/轨迹预测的pipline，通过各种神经网络（Vectornet、Semantic LSTM、RNN、MLP等）对障碍物未来行为/轨迹进行预测，输出障碍物未来N秒的轨迹点。

### Channel+消息格式

输入消息

| 消息名称                                   | 通道名称                     | 功能                   |
| ------------------------------------------ | ---------------------------- | ---------------------- |
| apollo::localization::LocalizationEstimate | /apollo/localization/pose    | 定位消息通道           |
| apollo::perception::PerceptionObstacles    | /apollo/perception/obstacles | 感知输出障碍物消息通道 |
| apollo::planning::ADCTrajectory            | /apollo/planning             | planning消息通道       |
| apollo::storytelling::Stories              | /apollo/storytelling         | 场景消息通道           |

输出消息

| 消息名称                                   | 通道名称                                | 功能                |
| ------------------------------------------ | --------------------------------------- | ------------------- |
| apollo::prediction::ADCTrajectoryContainer | /apollo/prediction/adccontainer         | 主车container通道   |
| apollo::prediction::SubmoduleOutput        | /apollo/prediction/container            | 子模块container通道 |
| apollo::prediction::SubmoduleOutput        | /apollo/prediction/evaluator            | 子模块evaluator通道 |
| apollo::perception::PerceptionObstacles    | /apollo/prediction/perception_obstacles | 障碍物消息通道      |
| apollo::prediction::PredictionObstacles    | /apollo/prediction                      | 障碍物未来轨迹通道  |

### 参数

glags文件路径：modules/prediction/common/prediction_gflags.cc

| 参数类型 | 参数名                                           | 默认值       | 含义                                         |
| -------- | ------------------------------------------------ | ------------ | -------------------------------------------- |
| double   | prediction_trajectory_time_length                | 6.0          | 预测轨迹时间长度                             |
| double   | prediction_trajectory_time_resolution            | 0.1          | 预测轨迹时间分辨率                           |
| double   | min_prediction_trajectory_spatial_length         | 100.0        | 最小空间长度                                 |
| bool     | enable_trajectory_validation_check               | false        | 是否验证轨迹的有效性                         |
| bool     | free_move_predict_with_accelerate                | false        | freemove predictor是否使用加速度             |
| double   | vehicle_max_linear_acc                           | 4.0          | 车辆线性加速度的上边界                       |
| double   | vehicle_min_linear_acc                           | -4.0         | 车辆线性加速度的下边界                       |
| double   | vehicle_max_speed                                | 35.0         | 车辆最大速度                                 |
| double   | max_tracking_time                                | 0.5          | 障碍物最大tracking时间                       |
| double   | max_tracking_dist                                | 3.0          | 障碍物最大tracking距离                       |
| double   | lane_search_radius                               | 3.0          | lane搜索半径                                 |
| double   | lane_search_radius_in_junction                   | 15.0         | junction内的lane搜索半径                     |
| double   | junction_search_radius                           | 1.0          | junction搜索半径                             |
| double   | pedestrian_nearby_lane_search_radius             | 5.0          | 判断是否行人在lane附近的搜索半径             |
| int32    | road_graph_max_search_horizon                    | 20           | 用来构建路网的最大搜索深度                   |
| double   | surrounding_lane_search_radius                   | 3.0          | 周围lane的搜索半径                           |
| double   | base_image_half_range                            | 100.0        | base image的一半距离                         |
| bool     | enable_draw_adc_trajectory                       | true         | 是否在semantic map中绘制主车轨迹             |
| bool     | img_show_semantic_map                            | false        | 是否展示semantic map图像                     |
| double   | junction_distance_threshold                      | 10.0         | junction距离阈值                             |
| bool     | enable_all_junction                              | false        | 是否使用junction_mlp_model处理所有的junction |
| bool     | enable_all_pedestrian_caution_in_front           | false        | 如果true，主车前方所有行人都是caution        |
| bool     | enable_rank_caution_obstacles                    | true         | 是否对所有caution级别的障碍物进行排序        |
| bool     | enable_rank_interactive_obstacles                | true         | 是否对交互的障碍物进行排序                   |
| int32    | caution_obs_max_nums                             | 6            | caution目标最大数量                          |
| double   | caution_distance_threshold                       | 60.0         | caution目标的距离阈值                        |
| double   | caution_search_distance_ahead                    | 50.0         | 前方搜索caution目标的距离阈值                |
| double   | caution_search_distance_backward                 | 50.0         | 后方搜索caution目标的距离阈值                |
| double   | caution_search_distance_backward_for_merge       | 60.0         | merging时后方搜索caution目标的距离阈值       |
| double   | caution_search_distance_backward_for_overlap     | 30.0         | overlap时后方搜索caution目标的距离阈值       |
| double   | caution_pedestrian_approach_time                 | 3.0          | caution行人通过时间                          |
| int32    | interactive_obs_max_nums                         | 6            | 交互目标最大数量                             |
| double   | interaction_distance_threshold                   | 60.0         | 交互目标距离阈值                             |
| double   | interaction_search_distance_ahead                | 50.0         | 交互目标前方搜索距离                         |
| double   | interaction_search_distance_backward             | 50.0         | 交互目标后方搜索距离                         |
| double   | interaction_search_distance_backward_for_merge   | 60.0         | mergeing时后方搜索距离                       |
| double   | interaction_search_distance_backward_for_overlap | 30.0         | overlap时后方搜索距离                        |
| int32    | ego_vehicle_id                                   | -1           | 主车id                                       |
| double   | scan_length                                      | 80.0         | 目标搜索区域的长度                           |
| double   | scan_width                                       | 12.0         | 目标搜索区域的宽度                           |
| double   | back_dist_ignore_ped                             | -2.0         | 忽略行人的背后障碍物                         |
| uint64   | cruise_historical_frame_length                   | 5            | cruise模型的历史帧数                         |
| bool     | enable_kf_tracking                               | false        | 是否使用卡尔曼滤波                           |
| double   | max_angle_diff_to_adjust_velocity                | M_PI / 6.0   | 最大的角度差来调整速度朝向                   |
| int32    | min_still_obstacle_history_length                | 4            | 静止目标最小的历史帧数                       |
| int32    | max_still_obstacle_history_length                | 10           | 静止目标最大的历史帧数                       |
| double   | still_obstacle_speed_threshold                   | 0.99         | 静止目标的速度阈值                           |
| double   | still_pedestrian_speed_threshold                 | 0.2          | 静止行人的速度阈值                           |
| double   | still_unknown_speed_threshold                    | 0.5          | 静止unknown目标的速度阈值                    |
| double   | slow_obstacle_speed_threshold                    | 2.0          | 低速障碍物的速度阈值                         |
| double   | max_history_time                                 | 7.0          | 最大的历史时间                               |
| double   | target_lane_gap                                  | 2.0          | 两个lane点的距离                             |
| double   | dense_lane_gap                                   | 0.2          | 用于构建密集路网的两个lane点的距离           |
| int32    | max_num_current_lane                             | 2            | 当前lane最大数量                             |
| int32    | max_num_nearby_lane                              | 2            | 附近lane的最大数量                           |
| double   | max_lane_angle_diff                              | M_PI / 3.0   | lane最大角度差                               |
| int32    | max_num_current_lane_in_junction                 | 3            | junction中当前lane最大数量                   |
| int32    | max_num_nearby_lane_in_junction                  | 2            | junction中附近lane最大数量                   |
| double   | max_lane_angle_diff_in_junction                  | M_PI / 4.0   | junction中lane最大角度差                     |
| double   | pedestrian_max_speed                             | 10.0         | 行人最大速度                                 |
| double   | pedestrian_max_acc                               | 2.0          | 行人最大加速度                               |
| double   | still_speed                                      | 0.01         | 静止速度阈值                                 |
| double   | max_num_obstacles                                | 300          | 容器中最大目标数量                           |
| double   | valid_position_diff_threshold                    | 0.5          | 有效位置差异阈值                             |
| double   | valid_position_diff_rate_threshold               | 0.075        | 有效位置差异率阈值                           |
| bool     | adjust_velocity_by_obstacle_heading              | true         | 通过目标heading来调整速度朝向                |
| bool     | adjust_velocity_by_position_shift                | false        | 调整速度朝向为lane heading                   |
| bool     | adjust_vehicle_heading_by_lane                   | true         | 通过lane来调整主车朝向                       |
| uint64   | max_num_lane_point                               | 20           | 最大lane点数                                 |
| double   | distance_threshold_to_junction_exit              | 1.0          | 离开junction的距离阈值                       |
| double   | angle_threshold_to_junction_exit                 | M_PI \* 0.25 | 离开junction的角度阈值                       |
| uint32   | junction_historical_frame_length                 | 5            | junction模型的历史帧数                       |
| double   | defualt_junction_range                           | 10.0         | junction默认距离                             |
| double   | distance_to_slow_down_at_stop_sign               | 80.0         | 停止线减速距离阈值                           |

### 包列表

| 包名                                       | 说明       |
| ------------------------------------------ | ---------- |
| [prediction](modules/prediction/README.md) | 预测模块包 |
