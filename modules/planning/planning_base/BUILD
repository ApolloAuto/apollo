load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools/platform:build_defs.bzl", "if_gpu")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]
PLANNING_FOPENMP = [
    "-DMODULE_NAME=\\\"planning\\\"",
    "-fopenmp",
]

apollo_component(
    name = "libplanning_component.so",
    srcs = ["planning_component.cc"],
    hdrs = ["planning_component.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":apollo_planning_planning_base",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/external_command_msgs:command_status_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/perception_msgs:traffic_light_detection_cc_proto",
        "//modules/common_msgs/planning_msgs:navigation_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_cc_proto",
        "//modules/common_msgs/storytelling_msgs:story_cc_proto",
    ],
)

apollo_cc_library(
    name = "apollo_planning_planning_base",
    copts = [
        "-fopenmp",
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    linkopts = ["-lgomp"],
    srcs = [
        "common/config_util.cc",
        "common/ego_info.cc",
        "common/feature_output.cc",
        "common/frame.cc",
        "common/history.cc",
        "common/learning_based_data.cc",
        "common/message_process.cc",
        "common/obstacle.cc",
        "common/obstacle_blocking_analyzer.cc",
        "common/open_space_info.cc",
        "common/path/discretized_path.cc",
        "common/path/frenet_frame_path.cc",
        "common/path/path_data.cc",
        "common/path_boundary.cc",
        "common/path_decision.cc",
        "common/planning_context.cc",
        "common/planning_gflags.cc",
        "common/reference_line_info.cc",
        "common/smoothers/smoother.cc",
        "common/speed/speed_data.cc",
        "common/speed/st_boundary.cc",
        "common/speed/st_point.cc",
        "common/speed_limit.cc",
        "common/speed_profile_generator.cc",
        "common/st_graph_data.cc",
        "common/trajectory/discretized_trajectory.cc",
        "common/trajectory/publishable_trajectory.cc",
        "common/trajectory1d/constant_deceleration_trajectory1d.cc",
        "common/trajectory1d/constant_jerk_trajectory1d.cc",
        "common/trajectory1d/piecewise_acceleration_trajectory1d.cc",
        "common/trajectory1d/piecewise_jerk_trajectory1d.cc",
        "common/trajectory1d/piecewise_trajectory1d.cc",
        "common/trajectory1d/standing_still_trajectory1d.cc",
        "common/trajectory_evaluator.cc",
        "common/trajectory_stitcher.cc",
        "common/util/common.cc",
        "common/util/math_util.cc",
        "common/util/print_debug_info.cc",
        "common/util/util.cc",
        "constraint_checker/collision_checker.cc",
        "constraint_checker/constraint_checker.cc",
        "constraint_checker/constraint_checker1d.cc",
        "lattice/behavior/feasible_region.cc",
        "lattice/behavior/path_time_graph.cc",
        "lattice/behavior/prediction_querier.cc",
        "lattice/trajectory_generation/backup_trajectory_generator.cc",
        "lattice/trajectory_generation/end_condition_sampler.cc",
        "lattice/trajectory_generation/lateral_osqp_optimizer.cc",
        "lattice/trajectory_generation/lateral_qp_optimizer.cc",
        "lattice/trajectory_generation/lattice_trajectory1d.cc",
        "lattice/trajectory_generation/piecewise_braking_trajectory_generator.cc",
        "lattice/trajectory_generation/trajectory1d_generator.cc",
        "lattice/trajectory_generation/trajectory_combiner.cc",
        "lattice/trajectory_generation/trajectory_evaluator.cc",
        "learning_based/img_feature_renderer/birdview_img_feature_renderer.cc",
        "learning_based/model_inference/trajectory_imitation_libtorch_inference.cc",
        "math/curve1d/cubic_polynomial_curve1d.cc",
        "math/curve1d/piecewise_quintic_spiral_path.cc",
        "math/curve1d/quartic_polynomial_curve1d.cc",
        "math/curve1d/quintic_polynomial_curve1d.cc",
        "math/curve1d/quintic_spiral_path.cc",
        "math/curve_math.cc",
        "math/discrete_points_math.cc",
        "math/discretized_points_smoothing/cos_theta_ipopt_interface.cc",
        "math/discretized_points_smoothing/cos_theta_smoother.cc",
        "math/discretized_points_smoothing/fem_pos_deviation_ipopt_interface.cc",
        "math/discretized_points_smoothing/fem_pos_deviation_osqp_interface.cc",
        "math/discretized_points_smoothing/fem_pos_deviation_smoother.cc",
        "math/discretized_points_smoothing/fem_pos_deviation_sqp_osqp_interface.cc",
        "math/piecewise_jerk/piecewise_jerk_path_problem.cc",
        "math/piecewise_jerk/piecewise_jerk_problem.cc",
        "math/piecewise_jerk/piecewise_jerk_speed_problem.cc",
        "math/polynomial_xd.cc",
        "math/smoothing_spline/affine_constraint.cc",
        "math/smoothing_spline/osqp_spline_1d_solver.cc",
        "math/smoothing_spline/osqp_spline_2d_solver.cc",
        "math/smoothing_spline/spline_1d.cc",
        "math/smoothing_spline/spline_1d_constraint.cc",
        "math/smoothing_spline/spline_1d_kernel.cc",
        "math/smoothing_spline/spline_1d_seg.cc",
        "math/smoothing_spline/spline_1d_solver.cc",
        "math/smoothing_spline/spline_2d.cc",
        "math/smoothing_spline/spline_2d_constraint.cc",
        "math/smoothing_spline/spline_2d_kernel.cc",
        "math/smoothing_spline/spline_2d_seg.cc",
        "math/smoothing_spline/spline_seg_kernel.cc",
        "navi/decider/navi_obstacle_decider.cc",
        "navi/decider/navi_path_decider.cc",
        "navi/decider/navi_speed_decider.cc",
        "navi/decider/navi_speed_ts_graph.cc",
        "navi/decider/navi_task.cc",
        "navi_planning.cc",
        "on_lane_planning.cc",
        "open_space/coarse_trajectory_generator/grid_search.cc",
        "open_space/coarse_trajectory_generator/hybrid_a_star.cc",
        "open_space/coarse_trajectory_generator/node3d.cc",
        "open_space/coarse_trajectory_generator/reeds_shepp_path.cc",
        "open_space/trajectory_smoother/distance_approach_ipopt_cuda_interface.cc",
        "open_space/trajectory_smoother/distance_approach_ipopt_fixed_dual_interface.cc",
        "open_space/trajectory_smoother/distance_approach_ipopt_fixed_ts_interface.cc",
        "open_space/trajectory_smoother/distance_approach_ipopt_interface.cc",
        "open_space/trajectory_smoother/distance_approach_ipopt_relax_end_interface.cc",
        "open_space/trajectory_smoother/distance_approach_ipopt_relax_end_slack_interface.cc",
        "open_space/trajectory_smoother/distance_approach_problem.cc",
        "open_space/trajectory_smoother/dual_variable_warm_start_ipopt_interface.cc",
        "open_space/trajectory_smoother/dual_variable_warm_start_ipopt_qp_interface.cc",
        "open_space/trajectory_smoother/dual_variable_warm_start_osqp_interface.cc",
        "open_space/trajectory_smoother/dual_variable_warm_start_problem.cc",
        "open_space/trajectory_smoother/dual_variable_warm_start_slack_osqp_interface.cc",
        "open_space/trajectory_smoother/iterative_anchoring_smoother.cc",
        "open_space/utils/open_space_roi_util.cc",
        "pipeline/evaluator.cc",
        "planner/lattice/lattice_planner.cc",
        "planner/navi/navi_planner.cc",
        "planner/navi_planner_dispatcher.cc",
        "planner/on_lane_planner_dispatcher.cc",
        "planner/planner_dispatcher.cc",
        "planner/public_road/public_road_planner.cc",
        "planner/rtk/rtk_replay_planner.cc",
        "planning_base.cc",
        "reference_line/discrete_points_reference_line_smoother.cc",
        "reference_line/qp_spline_reference_line_smoother.cc",
        "reference_line/reference_line.cc",
        "reference_line/reference_line_provider.cc",
        "reference_line/reference_point.cc",
        "reference_line/spiral_problem_interface.cc",
        "reference_line/spiral_reference_line_smoother.cc",
        "scenario_base/base_stage_creep.cc",
        "scenario_base/base_stage_cruise.cc",
        "scenario_base/process_result.cc",
        "scenario_base/scenario.cc",
        "scenario_base/scenario_manager.cc",
        "scenario_base/stage.cc",
        "scenario_base/traffic_light_base/base_stage_traffic_light_creep.cc",
        "scenario_base/traffic_light_base/base_stage_traffic_light_cruise.cc",
        "task_base/common/decider.cc",
        "task_base/common/lane_change_util/lane_change_util.cc",
        "task_base/common/path_generation.cc",
        "task_base/common/path_util/path_assessment_decider_util.cc",
        "task_base/common/path_util/path_bounds_decider_util.cc",
        "task_base/common/path_util/path_optimizer_util.cc",
        "task_base/common/speed_optimizer.cc",
        "task_base/common/trajectory_optimizer.cc",
        "task_base/task.cc",
        "task_base/utils/st_gap_estimator.cc",
        "traffic_rules_base/traffic_decider.cc",
        "traffic_rules_base/traffic_rule.cc",
        "tuning/autotuning_mlp_net_model.cc",
        "tuning/autotuning_raw_feature_generator.cc",
        "tuning/speed_model/autotuning_speed_feature_builder.cc",
        "tuning/speed_model/autotuning_speed_mlp_model.cc",
    ],
    hdrs = [
        "common/config_util.h",
        "common/dependency_injector.h",
        "common/ego_info.h",
        "common/feature_output.h",
        "common/frame.h",
        "common/history.h",
        "common/indexed_list.h",
        "common/indexed_queue.h",
        "common/learning_based_data.h",
        "common/local_view.h",
        "common/message_process.h",
        "common/obstacle.h",
        "common/obstacle_blocking_analyzer.h",
        "common/open_space_info.h",
        "common/path/discretized_path.h",
        "common/path/frenet_frame_path.h",
        "common/path/path_data.h",
        "common/path_boundary.h",
        "common/path_decision.h",
        "common/planning_context.h",
        "common/planning_gflags.h",
        "common/reference_line_info.h",
        "common/smoothers/smoother.h",
        "common/speed/speed_data.h",
        "common/speed/st_boundary.h",
        "common/speed/st_point.h",
        "common/speed_limit.h",
        "common/speed_profile_generator.h",
        "common/st_graph_data.h",
        "common/trajectory/discretized_trajectory.h",
        "common/trajectory/publishable_trajectory.h",
        "common/trajectory1d/constant_deceleration_trajectory1d.h",
        "common/trajectory1d/constant_jerk_trajectory1d.h",
        "common/trajectory1d/piecewise_acceleration_trajectory1d.h",
        "common/trajectory1d/piecewise_jerk_trajectory1d.h",
        "common/trajectory1d/piecewise_trajectory1d.h",
        "common/trajectory1d/standing_still_trajectory1d.h",
        "common/trajectory_evaluator.h",
        "common/trajectory_stitcher.h",
        "common/util/common.h",
        "common/util/math_util.h",
        "common/util/print_debug_info.h",
        "common/util/util.h",
        "constraint_checker/collision_checker.h",
        "constraint_checker/constraint_checker.h",
        "constraint_checker/constraint_checker1d.h",
        "lattice/behavior/feasible_region.h",
        "lattice/behavior/path_time_graph.h",
        "lattice/behavior/prediction_querier.h",
        "lattice/trajectory_generation/backup_trajectory_generator.h",
        "lattice/trajectory_generation/end_condition_sampler.h",
        "lattice/trajectory_generation/lateral_osqp_optimizer.h",
        "lattice/trajectory_generation/lateral_qp_optimizer.h",
        "lattice/trajectory_generation/lattice_trajectory1d.h",
        "lattice/trajectory_generation/piecewise_braking_trajectory_generator.h",
        "lattice/trajectory_generation/trajectory1d_generator.h",
        "lattice/trajectory_generation/trajectory_combiner.h",
        "lattice/trajectory_generation/trajectory_evaluator.h",
        "learning_based/img_feature_renderer/birdview_img_feature_renderer.h",
        "learning_based/model_inference/model_inference.h",
        "learning_based/model_inference/trajectory_imitation_libtorch_inference.h",
        "math/curve1d/cubic_polynomial_curve1d.h",
        "math/curve1d/curve1d.h",
        "math/curve1d/piecewise_quintic_spiral_path.h",
        "math/curve1d/polynomial_curve1d.h",
        "math/curve1d/quartic_polynomial_curve1d.h",
        "math/curve1d/quintic_polynomial_curve1d.h",
        "math/curve1d/quintic_spiral_path.h",
        "math/curve1d/quintic_spiral_path_with_derivation.h",
        "math/curve_math.h",
        "math/discrete_points_math.h",
        "math/discretized_points_smoothing/cos_theta_ipopt_interface.h",
        "math/discretized_points_smoothing/cos_theta_smoother.h",
        "math/discretized_points_smoothing/fem_pos_deviation_ipopt_interface.h",
        "math/discretized_points_smoothing/fem_pos_deviation_osqp_interface.h",
        "math/discretized_points_smoothing/fem_pos_deviation_smoother.h",
        "math/discretized_points_smoothing/fem_pos_deviation_sqp_osqp_interface.h",
        "math/piecewise_jerk/piecewise_jerk_path_problem.h",
        "math/piecewise_jerk/piecewise_jerk_problem.h",
        "math/piecewise_jerk/piecewise_jerk_speed_problem.h",
        "math/polynomial_xd.h",
        "math/smoothing_spline/affine_constraint.h",
        "math/smoothing_spline/osqp_spline_1d_solver.h",
        "math/smoothing_spline/osqp_spline_2d_solver.h",
        "math/smoothing_spline/spline_1d.h",
        "math/smoothing_spline/spline_1d_constraint.h",
        "math/smoothing_spline/spline_1d_kernel.h",
        "math/smoothing_spline/spline_1d_seg.h",
        "math/smoothing_spline/spline_1d_solver.h",
        "math/smoothing_spline/spline_2d.h",
        "math/smoothing_spline/spline_2d_constraint.h",
        "math/smoothing_spline/spline_2d_kernel.h",
        "math/smoothing_spline/spline_2d_seg.h",
        "math/smoothing_spline/spline_2d_solver.h",
        "math/smoothing_spline/spline_seg_kernel.h",
        "navi/decider/navi_obstacle_decider.h",
        "navi/decider/navi_path_decider.h",
        "navi/decider/navi_speed_decider.h",
        "navi/decider/navi_speed_ts_graph.h",
        "navi/decider/navi_task.h",
        "navi_planning.h",
        "on_lane_planning.h",
        "open_space/coarse_trajectory_generator/grid_search.h",
        "open_space/coarse_trajectory_generator/hybrid_a_star.h",
        "open_space/coarse_trajectory_generator/node3d.h",
        "open_space/coarse_trajectory_generator/reeds_shepp_path.h",
        "open_space/trajectory_smoother/distance_approach_interface.h",
        "open_space/trajectory_smoother/distance_approach_ipopt_cuda_interface.h",
        "open_space/trajectory_smoother/distance_approach_ipopt_fixed_dual_interface.h",
        "open_space/trajectory_smoother/distance_approach_ipopt_fixed_ts_interface.h",
        "open_space/trajectory_smoother/distance_approach_ipopt_interface.h",
        "open_space/trajectory_smoother/distance_approach_ipopt_relax_end_interface.h",
        "open_space/trajectory_smoother/distance_approach_ipopt_relax_end_slack_interface.h",
        "open_space/trajectory_smoother/distance_approach_problem.h",
        "open_space/trajectory_smoother/dual_variable_warm_start_ipopt_interface.h",
        "open_space/trajectory_smoother/dual_variable_warm_start_ipopt_qp_interface.h",
        "open_space/trajectory_smoother/dual_variable_warm_start_osqp_interface.h",
        "open_space/trajectory_smoother/dual_variable_warm_start_problem.h",
        "open_space/trajectory_smoother/dual_variable_warm_start_slack_osqp_interface.h",
        "open_space/trajectory_smoother/iterative_anchoring_smoother.h",
        "open_space/utils/open_space_roi_util.h",
        "pipeline/evaluator.h",
        "pipeline/evaluator_logger.h",
        "planner/lattice/lattice_planner.h",
        "planner/navi/navi_planner.h",
        "planner/navi_planner_dispatcher.h",
        "planner/on_lane_planner_dispatcher.h",
        "planner/planner.h",
        "planner/planner_dispatcher.h",
        "planner/public_road/public_road_planner.h",
        "planner/rtk/rtk_replay_planner.h",
        "planning_base.h",
        "reference_line/discrete_points_reference_line_smoother.h",
        "reference_line/qp_spline_reference_line_smoother.h",
        "reference_line/reference_line.h",
        "reference_line/reference_line_provider.h",
        "reference_line/reference_line_smoother.h",
        "reference_line/reference_point.h",
        "reference_line/spiral_problem_interface.h",
        "reference_line/spiral_reference_line_smoother.h",
        "scenario_base/base_stage_creep.h",
        "scenario_base/base_stage_cruise.h",
        "scenario_base/process_result.h",
        "scenario_base/scenario.h",
        "scenario_base/scenario_manager.h",
        "scenario_base/stage.h",
        "scenario_base/traffic_light_base/base_stage_traffic_light_creep.h",
        "scenario_base/traffic_light_base/base_stage_traffic_light_cruise.h",
        "task_base/common/decider.h",
        "task_base/common/lane_change_util/lane_change_util.h",
        "task_base/common/path_generation.h",
        "task_base/common/path_util/path_assessment_decider_util.h",
        "task_base/common/path_util/path_bounds_decider_util.h",
        "task_base/common/path_util/path_optimizer_util.h",
        "task_base/common/speed_optimizer.h",
        "task_base/common/trajectory_optimizer.h",
        "task_base/task.h",
        "task_base/utils/st_gap_estimator.h",
        "traffic_rules_base/traffic_decider.h",
        "traffic_rules_base/traffic_rule.h",
        "tuning/autotuning_base_model.h",
        "tuning/autotuning_feature_builder.h",
        "tuning/autotuning_mlp_net_model.h",
        "tuning/autotuning_raw_feature_generator.h",
        "tuning/speed_model/autotuning_speed_feature_builder.h",
        "tuning/speed_model/autotuning_speed_mlp_model.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:config_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/math/qp_solver",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/common/util:common_util",
        "//modules/common/util:util_tool",
        "//modules/common/vehicle_model",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common_msgs/dreamview_msgs:chart_cc_proto",
        "//modules/common_msgs/dreamview_msgs:hmi_status_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/map_msgs:map_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/common_msgs/perception_msgs:traffic_light_detection_cc_proto",
        "//modules/common_msgs/planning_msgs:decision_cc_proto",
        "//modules/common_msgs/planning_msgs:navigation_cc_proto",
        "//modules/common_msgs/planning_msgs:pad_msg_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_command_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_internal_cc_proto",
        "//modules/common_msgs/prediction_msgs:feature_cc_proto",
        "//modules/common_msgs/prediction_msgs:lane_graph_cc_proto",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_cc_proto",
        "//modules/common_msgs/prediction_msgs:prediction_point_cc_proto",
        "//modules/common_msgs/prediction_msgs:scenario_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "//modules/common_msgs/storytelling_msgs:story_cc_proto",
        "//modules/map:apollo_map",
        "//modules/planning/planning_base/learning_based/model_inference/proto:learning_model_inference_cc_proto",
        "//modules/planning/planning_base/proto:auto_tuning_model_input_cc_proto",
        "//modules/planning/planning_base/proto:auto_tuning_raw_feature_cc_proto",
        "//modules/planning/planning_base/proto:lattice_structure_cc_proto",
        "//modules/planning/planning_base/proto:learning_data_cc_proto",
        "//modules/planning/planning_base/proto:navi_task_config_cc_proto",
        "//modules/planning/planning_base/proto:piecewise_jerk_path_config_cc_proto",
        "//modules/planning/planning_base/proto:planner_open_space_config_cc_proto",
        "//modules/planning/planning_base/proto:planning_config_cc_proto",
        "//modules/planning/planning_base/proto:planning_semantic_map_config_cc_proto",
        "//modules/planning/planning_base/proto:planning_status_cc_proto",
        "//modules/planning/planning_base/proto:reference_line_smoother_config_cc_proto",
        "//modules/planning/planning_base/proto:st_drivable_boundary_cc_proto",
        "//modules/planning/planning_base/proto/math:cos_theta_smoother_config_cc_proto",
        "//modules/planning/planning_base/proto/math:fem_pos_deviation_smoother_config_cc_proto",
        "//modules/planning/planning_base/proto/math:qp_problem_cc_proto",
        "//modules/planning/planning_base/scenario_base/proto:creep_stage_cc_proto",
        "//modules/planning/planning_base/scenario_base/proto:scenario_pipeline_cc_proto",
        "//modules/planning/planning_base/traffic_rules_base/proto:traffic_rules_cc_proto",
        "//modules/prediction:apollo_prediction_network",
        "@adolc",
        "@boost",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//:absl",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
        "@eigen",
        "@ipopt",
        "@libtorch_cpu",
        "@opencv//:core",
        "@opencv//:imgcodecs",
        "@osqp",
    ] + if_gpu([":planning_block"]),
)

cuda_library(
    name = "planning_block",
    srcs = ["open_space/trajectory_smoother/planning_block.cu"],
    hdrs = ["open_space/trajectory_smoother/planning_block.h"],
    deps = [
        "@local_config_cuda//cuda:cuda_headers",
        "@local_config_cuda//cuda:cudart",
    ],
    mandatory = False,
)

filegroup(
    name = "planning_testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

filegroup(
    name = "planning_conf",
    srcs = glob([
        "conf/**",
    ]),
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "dag/*.dag",
        "launch/*.launch",
    ]) + [":planning_conf"],
)

apollo_cc_test(
    name = "distance_approach_ipopt_interface_test",
    size = "small",
    srcs = ["open_space/trajectory_smoother/distance_approach_ipopt_interface_test.cc"],
    copts = PLANNING_FOPENMP,
    linkopts = ["-lgomp"],
    deps = [
        ":apollo_planning_planning_base",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "navi_planner_test",
    size = "small",
    srcs = ["planner/navi/navi_planner_test.cc"],
    data = ["//modules/planning/planning_base:planning_testdata"],
    linkopts = ["-lgomp"],
    deps = [
        ":apollo_planning_planning_base",
        "//cyber",
        "//modules/common/util:common_util",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "rtk_replay_planner_test",
    size = "small",
    srcs = ["planner/rtk/rtk_replay_planner_test.cc"],
    data = ["//modules/planning/planning_base:planning_testdata"],
    linkopts = ["-lgomp"],
    deps = [
        ":apollo_planning_planning_base",
        "//cyber",
        "//modules/common/util:common_util",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "spiral_reference_line_smoother_test",
    size = "small",
    srcs = ["reference_line/spiral_reference_line_smoother_test.cc"],
    data = [
        "//modules/planning/planning_base:planning_conf",
    ],
    deps = [
        ":apollo_planning_planning_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "qp_spline_reference_line_smoother_test",
    size = "small",
    srcs = ["reference_line/qp_spline_reference_line_smoother_test.cc"],
    data = [
        "//modules/planning/planning_base:planning_conf",
        "//modules/planning/planning_base:planning_testdata",
    ],
    deps = [
        ":apollo_planning_planning_base",
        "//modules/map:apollo_map",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "autotuning_speed_feature_builder_test",
    size = "small",
    srcs = ["tuning/speed_model/autotuning_speed_feature_builder_test.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "autotuning_speed_mlp_model_test",
    size = "small",
    srcs = ["tuning/speed_model/autotuning_speed_mlp_model_test.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "@com_google_googletest//:gtest_main",
    ],
    tags = ["exclude"]
)

apollo_cc_test(
    name = "autotuning_raw_feature_generator_test",
    size = "small",
    srcs = ["tuning/autotuning_raw_feature_generator_test.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "autotuning_mlp_net_model_test",
    size = "small",
    srcs = ["tuning/autotuning_mlp_net_model_test.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "@com_google_googletest//:gtest_main",
    ],
    tags = ["exclude"]
)

apollo_cc_test(
    name = "discrete_points_reference_line_smoother_test",
    size = "small",
    srcs = ["reference_line/discrete_points_reference_line_smoother_test.cc"],
    data = [
        "//modules/planning/planning_base:planning_conf",
        "//modules/planning/planning_base:planning_testdata",
    ],
    deps = [
        ":apollo_planning_planning_base",
        "//modules/map:apollo_map",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_binary(
    name = "smoother_util",
    srcs = ["reference_line/smoother_util.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "//modules/planning/planning_base/proto:planning_config_cc_proto",
    ],
)

apollo_cc_binary(
    name = "spiral_smoother_util",
    srcs = ["reference_line/spiral_smoother_util.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "//modules/planning/planning_base/proto:planning_config_cc_proto",
    ],
)

apollo_cc_test(
    name = "public_road_planner_test",
    size = "small",
    srcs = ["planner/public_road/public_road_planner_test.cc"],
    linkopts = ["-lgomp"],
    deps = [
        ":apollo_planning_planning_base",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "on_lane_planner_dispatcher_test",
    size = "small",
    srcs = ["planner/on_lane_planner_dispatcher_test.cc"],
    linkopts = ["-lgomp"],
    deps = [
        ":apollo_planning_planning_base",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "navi_planner_dispatcher_test",
    size = "small",
    srcs = ["planner/navi_planner_dispatcher_test.cc"],
    linkopts = ["-lgomp"],
    deps = [
        ":apollo_planning_planning_base",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "distance_approach_ipopt_cuda_interface_test",
    size = "small",
    srcs = ["open_space/trajectory_smoother/distance_approach_ipopt_cuda_interface_test.cc"],
    copts = PLANNING_FOPENMP,
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":apollo_planning_planning_base",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "distance_approach_problem_test",
    size = "small",
    srcs = ["open_space/trajectory_smoother/distance_approach_problem_test.cc"],
    copts = PLANNING_FOPENMP,
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":apollo_planning_planning_base",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "dual_variable_warm_start_ipopt_interface_test",
    size = "small",
    srcs = ["open_space/trajectory_smoother/dual_variable_warm_start_ipopt_interface_test.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "dual_variable_warm_start_osqp_interface_test",
    size = "small",
    srcs = ["open_space/trajectory_smoother/dual_variable_warm_start_osqp_interface_test.cc"],
    linkstatic = True,
    deps = [
        ":apollo_planning_planning_base",
        "@com_google_googletest//:gtest_main",
        "@ipopt",
    ],
)

apollo_cc_test(
    name = "dual_variable_warm_start_problem_test",
    size = "small",
    srcs = ["open_space/trajectory_smoother/dual_variable_warm_start_problem_test.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "indexed_list_test",
    size = "small",
    srcs = ["common/indexed_list_test.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "//modules/common/util:common_util",
        "@boost",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "indexed_queue_test",
    size = "small",
    srcs = ["common/indexed_queue_test.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "//modules/common/util:common_util",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "obstacle_test",
    size = "small",
    srcs = ["common/obstacle_test.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "//modules/common/util:common_util",
        "//modules/common_msgs/perception_msgs:perception_lane_cc_proto",
        "@boost",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "reference_line_info_test",
    size = "small",
    srcs = ["common/reference_line_info_test.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "speed_profile_generator_test",
    size = "small",
    srcs = ["common/speed_profile_generator_test.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "frame_test",
    size = "small",
    srcs = ["common/frame_test.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "//modules/common/util:common_util",
        "//modules/map:apollo_map",
        "//modules/planning/planning_base/proto:planning_config_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "quartic_polynomial_curve1d_test",
    size = "small",
    srcs = ["math/curve1d/quartic_polynomial_curve1d_test.cc"],
    copts = PLANNING_COPTS,
    deps = [
        ":apollo_planning_planning_base",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "cubic_polynomial_curve1d_test",
    size = "small",
    srcs = ["math/curve1d/cubic_polynomial_curve1d_test.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "osqp_spline_2d_solver_test",
    size = "small",
    srcs = ["math/smoothing_spline/osqp_spline_2d_solver_test.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "spline_1d_kernel_test",
    size = "small",
    srcs = ["math/smoothing_spline/spline_1d_kernel_test.cc"],
    linkopts = ["-lm"],
    deps = [
        ":apollo_planning_planning_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "spline_1d_constraint_test",
    size = "small",
    srcs = ["math/smoothing_spline/spline_1d_constraint_test.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "reeds_shepp_path_test",
    size = "small",
    srcs = ["open_space/coarse_trajectory_generator/reeds_shepp_path_test.cc"],
    linkopts = ["-lgomp"],
    deps = [
        ":apollo_planning_planning_base",
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/planning/planning_base/proto:planner_open_space_config_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "node3d_test",
    size = "small",
    srcs = ["open_space/coarse_trajectory_generator/node3d_test.cc"],
    linkopts = ["-lgomp"],
    deps = [
        ":apollo_planning_planning_base",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "hybrid_a_star_test",
    size = "small",
    srcs = ["open_space/coarse_trajectory_generator/hybrid_a_star_test.cc"],
    linkopts = ["-lgomp"],
    deps = [
        ":apollo_planning_planning_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "navi_decider_test",
    size = "small",
    srcs = [
        "navi/decider/navi_obstacle_decider_test.cc",
        "navi/decider/navi_path_decider_test.cc",
        "navi/decider/navi_speed_decider_test.cc",
        "navi/decider/navi_speed_ts_graph_test.cc",
    ],
    data = ["//modules/planning/planning_base:planning_testdata"],
    deps = [
        ":apollo_planning_planning_base",
        "//modules/common/util:common_util",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "spline_2d_kernel_test",
    size = "small",
    srcs = ["math/smoothing_spline/spline_2d_kernel_test.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "spline_2d_constraint_test",
    size = "small",
    srcs = ["math/smoothing_spline/spline_2d_constraint_test.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "quintic_polynomial_curve1d_test",
    size = "small",
    srcs = ["math/curve1d/quintic_polynomial_curve1d_test.cc"],
    copts = PLANNING_COPTS,
    deps = [
        ":apollo_planning_planning_base",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "piecewise_quintic_spiral_path_test",
    size = "small",
    srcs = ["math/curve1d/piecewise_quintic_spiral_path_test.cc"],
    copts = PLANNING_COPTS,
    deps = [
        ":apollo_planning_planning_base",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "curve_math_test",
    size = "small",
    srcs = ["math/curve_math_test.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "model_inference_test",
    size = "medium",
    srcs = ["learning_based/model_inference/model_inference_test.cc"],
    copts = PLANNING_COPTS,
    deps = [
        ":apollo_planning_planning_base", 
        "//modules/common/configs:config_gflags",
        "@com_google_googletest//:gtest",
    ],
    linkstatic = True,
    tags = ["exclude"]
)

apollo_cc_test(
    name = "history_test",
    size = "small",
    srcs = ["common/history_test.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "speed_limit_test",
    size = "small",
    srcs = ["common/speed_limit_test.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "ego_info_test",
    size = "small",
    srcs = ["common/ego_info_test.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "open_space_info_test",
    size = "small",
    srcs = ["common/open_space_info_test.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "@boost",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "discretized_path_test",
    size = "small",
    srcs = ["common/path/discretized_path_test.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "//modules/common/util:common_util",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "frenet_frame_path_test",
    size = "small",
    srcs = ["common/path/frenet_frame_path_test.cc"],
    linkstatic = True,
    deps = [
        ":apollo_planning_planning_base",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "st_boundary_test",
    size = "small",
    srcs = ["common/speed/st_boundary_test.cc"],
    copts = PLANNING_COPTS,
    deps = [
        ":apollo_planning_planning_base",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "st_graph_data_test",
    size = "small",
    srcs = ["common/st_graph_data_test.cc"],
    deps = [
        ":apollo_planning_planning_base",
        "//cyber",
        "//modules/common/util:common_util",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "discretized_trajectory_test",
    size = "small",
    srcs = ["common/trajectory/discretized_trajectory_test.cc"],
    data = [
        "//modules/planning/planning_base:planning_testdata",
    ],
    deps = [
        ":apollo_planning_planning_base",
        "//modules/common/util:common_util",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "publishable_trajectory_test",
    size = "small",
    srcs = ["common/trajectory/publishable_trajectory_test.cc"],
    data = [
        "//modules/planning/planning_base:planning_testdata",
    ],
    deps = [
        ":apollo_planning_planning_base",
        "//modules/common/util:common_util",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_binary(
    name = "hybrid_a_star_wrapper_lib.so",
    srcs = ["open_space/tools/hybrid_a_star_wrapper.cc"],
    linkshared = True,
    linkstatic = False,
    deps = [
        ":apollo_planning_planning_base",
    ],
)

apollo_cc_binary(
    name = "distance_approach_problem_wrapper_lib.so",
    srcs = ["open_space/tools/distance_approach_problem_wrapper.cc"],
    copts = ["-fopenmp"],
    linkshared = True,
    linkstatic = False,
    deps = [
        ":apollo_planning_planning_base",
    ],
)

apollo_cc_binary(
    name = "open_space_roi_wrapper_lib.so",
    srcs = ["open_space/tools/open_space_roi_wrapper.cc"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    linkshared = True,
    linkstatic = False,
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/map:apollo_map",
        ":apollo_planning_planning_base",
        "//modules/planning/planning_base/proto:planner_open_space_config_cc_proto",
        "@eigen",
    ],
)

apollo_cc_binary(
    name = "evaluate_trajectory",
    srcs = ["pipeline/evaluate_trajectory.cc"],
    copts = PLANNING_COPTS,
    deps = [
        ":apollo_planning_planning_base",
    ],
)

apollo_cc_binary(
    name = "record_to_learning_data",
    srcs = ["pipeline/record_to_learning_data.cc"],
    copts = PLANNING_COPTS,
    deps = [
        ":apollo_planning_planning_base",
    ],
)

apollo_cc_binary(
    name = "planning_pad_terminal",
    srcs = ["tools/pad_terminal.cc"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:util_tool",
        ":apollo_planning_planning_base",
        "//modules/common_msgs/planning_msgs:pad_msg_cc_proto",
        "//modules/planning/planning_base/proto:planning_config_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_cc_binary(
    name = "inference_demo",
    srcs = ["tools/inference_demo.cc"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
        "@libtorch_cpu",
    ],
)

apollo_package()

cpplint()
