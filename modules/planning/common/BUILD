load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "indexed_list",
    hdrs = ["indexed_list.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/util:map_util",
        "@boost",
    ],
)

cc_test(
    name = "indexed_list_test",
    size = "small",
    srcs = ["indexed_list_test.cc"],
    deps = [
        ":indexed_list",
        "//modules/common/util",
        "@boost",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "indexed_queue",
    hdrs = ["indexed_queue.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/util:map_util",
    ],
)

cc_test(
    name = "indexed_queue_test",
    size = "small",
    srcs = ["indexed_queue_test.cc"],
    deps = [
        ":indexed_queue",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)

filegroup(
    name = "common_testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

cc_library(
    name = "obstacle",
    srcs = ["obstacle.cc"],
    hdrs = ["obstacle.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":indexed_list",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/util:map_util",
        "//modules/planning/common/speed:st_boundary",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/planning/reference_line",
        "//modules/prediction/proto:feature_cc_proto",
        "//modules/prediction/proto:lane_graph_cc_proto",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/prediction/proto:prediction_point_cc_proto",
        "//modules/prediction/proto:scenario_cc_proto",
    ],
)

cc_test(
    name = "obstacle_test",
    size = "small",
    srcs = ["obstacle_test.cc"],
    data = [
        "//modules/planning/common:common_testdata",
    ],
    deps = [
        ":obstacle",
        "//modules/common/util",
        "//modules/perception/proto:perception_lane_cc_proto",
        "@boost",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "obstacle_blocking_analyzer",
    srcs = ["obstacle_blocking_analyzer.cc"],
    hdrs = ["obstacle_blocking_analyzer.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":frame",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/planning/reference_line",
    ],
)

cc_library(
    name = "path_boundary",
    srcs = ["path_boundary.cc"],
    hdrs = ["path_boundary.h"],
)

cc_library(
    name = "planning_context",
    srcs = ["planning_context.cc"],
    hdrs = ["planning_context.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber/common",
        "//modules/planning/common/path:path_data",
        "//modules/planning/proto:planning_status_cc_proto",
        "@eigen",
    ],
)

cc_library(
    name = "path_decision",
    srcs = ["path_decision.cc"],
    hdrs = ["path_decision.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":obstacle",
        "//modules/planning/reference_line",
    ],
)

cc_library(
    name = "planning_gflags",
    srcs = ["planning_gflags.cc"],
    hdrs = ["planning_gflags.h"],
    copts = PLANNING_COPTS,
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "reference_line_info",
    srcs = ["reference_line_info.cc"],
    hdrs = ["reference_line_info.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":ego_info",
        ":path_boundary",
        ":path_decision",
        ":planning_gflags",
        ":st_graph_data",
        "//cyber/common:log",
        "//modules/common/math",
        "//modules/common/proto:pnc_point_cc_proto",
        "//modules/common/util:point_factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/pnc_map",
        "//modules/map/proto:map_cc_proto",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/common/trajectory:publishable_trajectory",
        "//modules/planning/proto:lattice_structure_cc_proto",
        "//modules/planning/reference_line",
        "@eigen",
    ],
)

cc_test(
    name = "reference_line_info_test",
    size = "small",
    srcs = ["reference_line_info_test.cc"],
    deps = [
        ":reference_line_info",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "speed_profile_generator",
    srcs = ["speed_profile_generator.cc"],
    hdrs = ["speed_profile_generator.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":frame",
        ":reference_line_info",
        "//modules/common/proto:pnc_point_cc_proto",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/math/curve1d",
        "//modules/planning/math/curve1d:polynomial_curve1d",
        "//modules/planning/math/curve1d:quartic_polynomial_curve1d",
        "//modules/planning/math/curve1d:quintic_polynomial_curve1d",
        "//modules/planning/math/piecewise_jerk:piecewise_jerk_speed_problem",
        "//modules/planning/proto:planning_config_cc_proto",
    ],
)

cc_test(
    name = "speed_profile_generator_test",
    size = "small",
    srcs = ["speed_profile_generator_test.cc"],
    deps = [
        ":speed_profile_generator",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "local_view",
    hdrs = ["local_view.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/map/relative_map/proto:navigation_cc_proto",
        "//modules/perception/proto:traffic_light_detection_cc_proto",
        "//modules/planning/proto:pad_msg_cc_proto",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/routing/proto:routing_cc_proto",
        "//modules/storytelling/proto:story_cc_proto",
    ],
)

cc_library(
    name = "frame",
    srcs = ["frame.cc"],
    hdrs = ["frame.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":feature_output",
        ":local_view",
        ":obstacle",
        ":open_space_info",
        ":reference_line_info",
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/monitor_log",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/pnc_map",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/common/trajectory:publishable_trajectory",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/reference_line:reference_line_provider",
        "@eigen",
    ],
)

cc_test(
    name = "frame_test",
    size = "small",
    srcs = ["frame_test.cc"],
    data = [
        "//modules/planning/common:common_testdata",
    ],
    deps = [
        ":frame",
        "//modules/common/util",
        "//modules/map/hdmap:hdmap_util",
        "//modules/planning/proto:planning_config_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "history",
    srcs = ["history.cc"],
    hdrs = ["history.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":planning_gflags",
        "//cyber/common",
        "//modules/planning/proto:decision_cc_proto",
        "//modules/planning/proto:planning_cc_proto",
        "@eigen",
    ],
)

cc_test(
    name = "history_test",
    size = "small",
    srcs = ["history_test.cc"],
    deps = [
        ":history",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "speed_limit",
    srcs = ["speed_limit.cc"],
    hdrs = ["speed_limit.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/math",
    ],
)

cc_test(
    name = "speed_limit_test",
    size = "small",
    srcs = ["speed_limit_test.cc"],
    deps = [
        ":speed_limit",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "ego_info",
    srcs = ["ego_info.cc"],
    hdrs = ["ego_info.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":obstacle",
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/configs/proto:vehicle_config_cc_proto",
        "//modules/common/math:geometry",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "//modules/planning/reference_line",
        "//modules/prediction/proto:feature_cc_proto",
        "//modules/prediction/proto:lane_graph_cc_proto",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/prediction/proto:prediction_point_cc_proto",
        "//modules/prediction/proto:scenario_cc_proto",
        "@eigen",
    ],
)

cc_test(
    name = "ego_info_test",
    size = "small",
    srcs = ["ego_info_test.cc"],
    deps = [
        ":ego_info",
        ":frame",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "planning_common",
    copts = PLANNING_COPTS,
    deps = [
        ":ego_info",
        ":frame",
        ":planning_gflags",
        ":speed_limit",
        ":st_graph_data",
        "//cyber/common:log",
        "//modules/localization/common:localization_common",
        "//modules/localization/proto:localization_cc_proto",
        "@eigen",
    ],
)

cc_library(
    name = "trajectory_stitcher",
    srcs = ["trajectory_stitcher.cc"],
    hdrs = ["trajectory_stitcher.h"],
    deps = [
        "//cyber/common:log",
        "//modules/common/proto:pnc_point_cc_proto",
        "//modules/common/vehicle_model",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/trajectory:publishable_trajectory",
        "//modules/planning/reference_line",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "open_space_info",
    srcs = ["open_space_info.cc"],
    hdrs = ["open_space_info.h"],
    deps = [
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/pnc_map",
        "//modules/planning/common:indexed_queue",
        "//modules/planning/common:obstacle",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/common/trajectory:publishable_trajectory",
        "@eigen",
    ],
)

cc_test(
    name = "open_space_info_test",
    size = "small",
    srcs = ["open_space_info_test.cc"],
    deps = [
        "open_space_info",
        "@boost",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "st_graph_data",
    srcs = ["st_graph_data.cc"],
    hdrs = ["st_graph_data.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":planning_gflags",
        ":speed_limit",
        "//modules/planning/common/speed:st_boundary",
        "//modules/planning/proto:st_drivable_boundary_cc_proto",
    ],
)

cc_test(
    name = "st_graph_data_test",
    size = "small",
    srcs = ["st_graph_data_test.cc"],
    deps = [
        ":st_graph_data",
        "//cyber/common:log",
        "//cyber/time:clock",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "feature_output",
    srcs = ["feature_output.cc"],
    hdrs = ["feature_output.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:learning_data_cc_proto",
    ],
)

cc_library(
    name = "message_process",
    srcs = ["message_process.cc"],
    hdrs = ["message_process.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":dependency_injector",
        ":feature_output",
        "//cyber",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:config_gflags",
        "//modules/dreamview/proto:hmi_status_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/perception/proto:perception_obstacle_cc_proto",
        "//modules/perception/proto:traffic_light_detection_cc_proto",
        "//modules/planning/common/util:math_util_lib",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/proto:learning_data_cc_proto",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/prediction/util:data_extraction",
        "//modules/routing/proto:routing_cc_proto",
        "//modules/storytelling/proto:story_cc_proto",
        "@boost",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "trajectory_evaluator",
    srcs = ["trajectory_evaluator.cc"],
    hdrs = ["trajectory_evaluator.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/pipeline:evaluator_logger",
        "//modules/planning/proto:learning_data_cc_proto",
    ],
)

cc_library(
    name = "dependency_injector",
    hdrs = ["dependency_injector.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":ego_info",
        ":frame",
        ":history",
        ":learning_based_data",
        ":planning_context",
    ],
)

cc_library(
    name = "learning_based_data",
    srcs = ["learning_based_data.cc"],
    hdrs = ["learning_based_data.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber/common",
        "//modules/planning/proto:learning_data_cc_proto",
    ],
)

cpplint()
