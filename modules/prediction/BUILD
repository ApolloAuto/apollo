load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_aarch64", "if_gpu")

package(default_visibility = ["//visibility:public"])

PREDICTION_COPTS = ["-DMODULE_NAME=\\\"prediction\\\""]

apollo_component(
    name = "libprediction_component.so",
    srcs = ["prediction_component.cc"],
    hdrs = ["prediction_component.h"],
    copts = PREDICTION_COPTS,
    deps = [
        ":apollo_prediction",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:util_tool",
    ],
)

apollo_cc_library(
    name = "apollo_prediction_network",
    srcs = [
        "network/net_layer.cc",
        "network/net_model.cc",
        "network/net_util.cc",
        "network/rnn_model/rnn_model.cc",
    ],
    hdrs = [
        "network/net_layer.h",
        "network/net_model.h",
        "network/net_util.h",
        "network/rnn_model/rnn_model.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/util:util_lib",
        "//modules/common/util:util_tool",
        "//modules/prediction/proto:network_layers_cc_proto",
        "//modules/prediction/proto:network_model_cc_proto",
        "@eigen",
    ],
)

apollo_cc_library(
    name = "apollo_prediction",
    srcs = [
        "common/environment_features.cc",
        "common/feature_output.cc",
        "common/junction_analyzer.cc",
        "common/message_process.cc",
        "common/prediction_gflags.cc",
        "common/prediction_map.cc",
        "common/prediction_system_gflags.cc",
        "common/prediction_thread_pool.cc",
        "common/prediction_util.cc",
        "common/road_graph.cc",
        "common/semantic_map.cc",
        "common/validation_checker.cc",
        "container/adc_trajectory/adc_trajectory_container.cc",
        "container/container_manager.cc",
        "container/obstacles/obstacle.cc",
        "container/obstacles/obstacle_clusters.cc",
        "container/obstacles/obstacles_container.cc",
        "container/pose/pose_container.cc",
        "container/storytelling/storytelling_container.cc",
        "evaluator/cyclist/cyclist_keep_lane_evaluator.cc",
        "evaluator/evaluator_manager.cc",
        "evaluator/model_manager/model_manager.cc",
        "evaluator/pedestrian/pedestrian_interaction_evaluator.cc",
        "evaluator/vehicle/cost_evaluator.cc",
        "evaluator/vehicle/cruise_mlp_evaluator.cc",
        "evaluator/vehicle/jointly_prediction_planning_evaluator.cc",
        "evaluator/vehicle/junction_map_evaluator.cc",
        "evaluator/vehicle/junction_mlp_evaluator.cc",
        "evaluator/vehicle/lane_aggregating_evaluator.cc",
        "evaluator/vehicle/lane_scanning_evaluator.cc",
        "evaluator/vehicle/mlp_evaluator.cc",
        "evaluator/vehicle/semantic_lstm_evaluator.cc",
        "evaluator/vehicle/vectornet_evaluator.cc",
        "evaluator/vehicle/multi_agent_evaluator.cc",
        "evaluator/warm_up/warm_up.cc",
        "pipeline/vector_net.cc",
        "predictor/empty/empty_predictor.cc",
        "predictor/extrapolation/extrapolation_predictor.cc",
        "predictor/free_move/free_move_predictor.cc",
        "predictor/interaction/interaction_predictor.cc",
        "predictor/junction/junction_predictor.cc",
        "predictor/lane_sequence/lane_sequence_predictor.cc",
        "predictor/move_sequence/move_sequence_predictor.cc",
        "predictor/predictor.cc",
        "predictor/predictor_manager.cc",
        "predictor/sequence/sequence_predictor.cc",
        "predictor/single_lane/single_lane_predictor.cc",
        "scenario/analyzer/scenario_analyzer.cc",
        "scenario/feature_extractor/feature_extractor.cc",
        "scenario/interaction_filter/interaction_filter.cc",
        "scenario/prioritization/obstacles_prioritizer.cc",
        "scenario/right_of_way/right_of_way.cc",
        "scenario/scenario_features/cruise_scenario_features.cc",
        "scenario/scenario_features/junction_scenario_features.cc",
        "scenario/scenario_features/scenario_features.cc",
        "scenario/scenario_manager.cc",
        "submodules/evaluator_submodule.cc",
        "submodules/predictor_submodule.cc",
        "submodules/submodule_output.cc",
    ] + if_aarch64(["common/affine_transform.cc"]),
    hdrs = [
        "common/environment_features.h",
        "common/feature_output.h",
        "common/junction_analyzer.h",
        "common/kml_map_based_test.h",
        "common/message_process.h",
        "common/prediction_constants.h",
        "common/prediction_gflags.h",
        "common/prediction_map.h",
        "common/prediction_system_gflags.h",
        "common/prediction_thread_pool.h",
        "common/prediction_util.h",
        "common/road_graph.h",
        "common/semantic_map.h",
        "common/validation_checker.h",
        "container/adc_trajectory/adc_trajectory_container.h",
        "container/container.h",
        "container/container_manager.h",
        "container/obstacles/obstacle.h",
        "container/obstacles/obstacle_clusters.h",
        "container/obstacles/obstacles_container.h",
        "container/pose/pose_container.h",
        "container/storytelling/storytelling_container.h",
        "evaluator/cyclist/cyclist_keep_lane_evaluator.h",
        "evaluator/evaluator.h",
        "evaluator/evaluator_manager.h",
        "evaluator/model_manager/model/model_base.h",
        "evaluator/model_manager/model_manager.h",
        "evaluator/pedestrian/pedestrian_interaction_evaluator.h",
        "evaluator/vehicle/cost_evaluator.h",
        "evaluator/vehicle/cruise_mlp_evaluator.h",
        "evaluator/vehicle/jointly_prediction_planning_evaluator.h",
        "evaluator/vehicle/junction_map_evaluator.h",
        "evaluator/vehicle/junction_mlp_evaluator.h",
        "evaluator/vehicle/lane_aggregating_evaluator.h",
        "evaluator/vehicle/lane_scanning_evaluator.h",
        "evaluator/vehicle/mlp_evaluator.h",
        "evaluator/vehicle/semantic_lstm_evaluator.h",
        "evaluator/vehicle/vectornet_evaluator.h",
        "evaluator/vehicle/multi_agent_evaluator.h",
        "evaluator/warm_up/warm_up.h",
        "pipeline/vector_net.h",
        "predictor/empty/empty_predictor.h",
        "predictor/extrapolation/extrapolation_predictor.h",
        "predictor/free_move/free_move_predictor.h",
        "predictor/interaction/interaction_predictor.h",
        "predictor/junction/junction_predictor.h",
        "predictor/lane_sequence/lane_sequence_predictor.h",
        "predictor/move_sequence/move_sequence_predictor.h",
        "predictor/predictor.h",
        "predictor/predictor_manager.h",
        "predictor/sequence/sequence_predictor.h",
        "predictor/single_lane/single_lane_predictor.h",
        "scenario/analyzer/scenario_analyzer.h",
        "scenario/feature_extractor/feature_extractor.h",
        "scenario/interaction_filter/interaction_filter.h",
        "scenario/prioritization/obstacles_prioritizer.h",
        "scenario/right_of_way/right_of_way.h",
        "scenario/scenario_features/cruise_scenario_features.h",
        "scenario/scenario_features/junction_scenario_features.h",
        "scenario/scenario_features/scenario_features.h",
        "scenario/scenario_manager.h",
        "submodules/evaluator_submodule.h",
        "submodules/predictor_submodule.h",
        "submodules/submodule_output.h",
    ] + if_aarch64(["common/affine_transform.h"]),
    copts = ["-fopenmp"] + PREDICTION_COPTS,
    linkopts = ["-lgomp"],
    deps = [
        ":apollo_prediction_network",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/adapters/proto:adapter_config_proto",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/filters",
        "//modules/common/math",
        "//modules/common/util:common_util",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/prediction_msgs:feature_cc_proto",
        "//modules/common_msgs/prediction_msgs:lane_graph_cc_proto",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_cc_proto",
        "//modules/common_msgs/prediction_msgs:scenario_cc_proto",
        "//modules/common_msgs/storytelling_msgs:story_cc_proto",
        "//modules/map:apollo_map",
        "//modules/prediction/proto:fnn_vehicle_model_cc_proto",
        "//modules/prediction/proto:network_layers_cc_proto",
        "//modules/prediction/proto:network_model_cc_proto",
        "//modules/prediction/proto:offline_features_cc_proto",
        "//modules/prediction/proto:prediction_conf_cc_proto",
        "//modules/prediction/proto:vector_net_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_googletest//:gtest_main",
        "@eigen",
        "@npp",
        "@opencv//:highgui",
        "@opencv//:imgcodecs",
    ] + if_gpu(
        ["@libtorch_gpu"],
        ["@libtorch_cpu"],
    ),
)

apollo_cc_binary(
    name = "vector_net_feature",
    srcs = ["pipeline/vector_net_feature.cc"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        ":apollo_prediction",
    ],
)

apollo_cc_binary(
    name = "vector_net_offline_data",
    srcs = ["pipeline/vector_net_offline_data.cc"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        ":apollo_prediction",
    ],
)

apollo_cc_binary(
    name = "records_to_offline_data",
    srcs = ["pipeline/records_to_offline_data.cc"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    linkopts = [
        "-lgomp",
    ],
    deps = [
        ":apollo_prediction",
        "@boost",
        "@com_google_absl//:absl",
    ],
)

apollo_cc_binary(
    name = "evaluator_submodule.so",
    linkshared = True,
    linkstatic = True,
    deps = [":apollo_prediction"],
)

apollo_cc_binary(
    name = "predictor_submodule.so",
    linkshared = True,
    linkstatic = True,
    deps = [":apollo_prediction"],
)

apollo_cc_binary(
    name = "prediction_lego.so",
    linkshared = True,
    linkstatic = True,
    deps = [":apollo_prediction"],
)

apollo_cc_test(
    name = "empty_predictor_test",
    size = "small",
    srcs = ["predictor/empty/empty_predictor_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "free_move_predictor_test",
    size = "small",
    srcs = ["predictor/free_move/free_move_predictor_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "interaction_predictor_test",
    size = "small",
    srcs = ["predictor/interaction/interaction_predictor_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":apollo_prediction",
    ],
)

apollo_cc_test(
    name = "junction_predictor_test",
    size = "small",
    srcs = ["predictor/junction/junction_predictor_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    linkopts = [
        "-lgomp",
    ],
    linkstatic = True,
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "lane_sequence_predictor_test",
    size = "small",
    srcs = ["predictor/lane_sequence/lane_sequence_predictor_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":apollo_prediction",
    ],
)

apollo_cc_test(
    name = "move_sequence_predictor_test",
    size = "small",
    srcs = ["predictor/move_sequence/move_sequence_predictor_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":apollo_prediction",
    ],
)

apollo_cc_test(
    name = "sequence_predictor_test",
    size = "small",
    srcs = ["predictor/sequence/sequence_predictor_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":apollo_prediction",
    ],
)

apollo_cc_test(
    name = "single_lane_predictor_test",
    size = "small",
    srcs = ["predictor/single_lane/single_lane_predictor_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":apollo_prediction",
    ],
)

apollo_cc_test(
    name = "scenario_manager_test",
    size = "small",
    srcs = ["scenario/scenario_manager_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":apollo_prediction",
    ],
)

apollo_cc_test(
    name = "scenario_analyzer_test",
    size = "small",
    srcs = ["scenario/analyzer/scenario_analyzer_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "feature_extractor_test",
    size = "small",
    srcs = ["scenario/feature_extractor/feature_extractor_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":apollo_prediction",
    ],
)

apollo_cc_test(
    name = "cruise_scenario_features_test",
    size = "small",
    srcs = ["scenario/scenario_features/cruise_scenario_features_test.cc"],
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "net_util_test",
    size = "small",
    srcs = ["network/net_util_test.cc"],
    linkstatic = True,
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "net_layer_test",
    size = "small",
    srcs = ["network/net_layer_test.cc"],
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "rnn_model_test",
    size = "small",
    srcs = ["network/rnn_model/rnn_model_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
    ],
    deps = [
        ":apollo_prediction",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "mlp_evaluator_test",
    size = "small",
    srcs = ["evaluator/vehicle/mlp_evaluator_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "cost_evaluator_test",
    size = "small",
    srcs = ["evaluator/vehicle/cost_evaluator_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "junction_mlp_evaluator_test",
    size = "small",
    srcs = ["evaluator/vehicle/junction_mlp_evaluator_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    linkopts = [
        "-lgomp",
    ],
    linkstatic = True,
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "pedestrian_interaction_evaluator_test",
    size = "small",
    srcs = ["evaluator/pedestrian/pedestrian_interaction_evaluator_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    linkstatic = True,
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "cyclist_keep_lane_evaluator_test",
    size = "small",
    srcs = ["evaluator/cyclist/cyclist_keep_lane_evaluator_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "cruise_mlp_evaluator_test",
    size = "small",
    srcs = ["evaluator/vehicle/cruise_mlp_evaluator_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    linkopts = [
        "-lgomp",
    ],
    linkstatic = True,
    deps = [
        ":apollo_prediction",
    ],
)

apollo_cc_test(
    name = "container_manager_test",
    size = "small",
    srcs = ["container/container_manager_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":apollo_prediction",
    ],
)

apollo_cc_test(
    name = "obstacles_container_test",
    size = "small",
    srcs = ["container/obstacles/obstacles_container_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "pose_container_test",
    size = "small",
    srcs = ["container/pose/pose_container_test.cc"],
    linkstatic = True,
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "obstacle_clusters_test",
    size = "small",
    srcs = ["container/obstacles/obstacle_clusters_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "obstacle_test",
    size = "small",
    srcs = ["container/obstacles/obstacle_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "adc_trajectory_container_test",
    size = "small",
    srcs = ["container/adc_trajectory/adc_trajectory_container_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "prediction_thread_pool_test",
    size = "small",
    srcs = ["common/prediction_thread_pool_test.cc"],
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "prediction_util_test",
    size = "small",
    srcs = ["common/prediction_util_test.cc"],
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "prediction_map_test",
    size = "small",
    srcs = ["common/prediction_map_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "feature_output_test",
    size = "small",
    srcs = ["common/feature_output_test.cc"],
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "road_graph_test",
    size = "small",
    srcs = ["common/road_graph_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "validation_checker_test",
    size = "small",
    srcs = ["common/validation_checker_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "environment_features_test",
    size = "small",
    srcs = ["common/environment_features_test.cc"],
    linkstatic = True,
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "junction_analyzer_test",
    size = "small",
    srcs = ["common/junction_analyzer_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":apollo_prediction",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "prediction_component_test",
    size = "small",
    srcs = ["prediction_component_test.cc"],
    data = [
        ":prediction_conf",
        ":prediction_data",
        ":prediction_testdata",
    ],
    linkopts = [
        "-lgomp",
    ],
    linkstatic = True,
    deps = [
        ":DO_NOT_IMPORT_prediction_component",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = [
        ":prediction_conf",
        ":prediction_data",
    ] + glob([
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

filegroup(
    name = "prediction_data",
    srcs = glob([
        "data/*.pt",
        "data/*.bin",
    ]),
)

filegroup(
    name = "prediction_conf",
    srcs = glob([
        "conf/*.conf",
        "conf/*.txt",
    ]),
)

filegroup(
    name = "prediction_testdata",
    srcs = glob(["testdata/**"]),
)

apollo_package()

cpplint()
