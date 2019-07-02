load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "indexed_list",
    hdrs = [
        "indexed_list.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    linkopts = [
        "-lboost_thread",
    ],
    deps = [
        "//modules/common/util:map_util",
    ],
)

cc_test(
    name = "indexed_list_test",
    size = "small",
    srcs = [
        "indexed_list_test.cc",
    ],
    linkopts = [
        "-lboost_system",
    ],
    deps = [
        ":indexed_list",
        "//modules/common/util",
        "@gtest//:main",
    ],
)

cc_library(
    name = "indexed_queue",
    hdrs = [
        "indexed_queue.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//modules/common/util:map_util",
    ],
)

cc_test(
    name = "indexed_queue_test",
    size = "small",
    srcs = [
        "indexed_queue_test.cc",
    ],
    deps = [
        ":indexed_queue",
        "//modules/common/util",
        "@gtest//:main",
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
    srcs = [
        "obstacle.cc",
    ],
    hdrs = [
        "obstacle.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":indexed_list",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/util:map_util",
        "//modules/planning/common/speed:st_boundary",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/reference_line",
    ],
)

cc_test(
    name = "obstacle_test",
    size = "small",
    srcs = [
        "obstacle_test.cc",
    ],
    data = [
        "//modules/planning/common:common_testdata",
    ],
    deps = [
        ":obstacle",
        "//modules/common/util",
        "//modules/perception/proto:perception_proto",
        "@gtest//:main",
    ],
    linkopts = [
        "-lboost_system",
    ],
)

cc_library(
    name = "obstacle_blocking_analyzer",
    srcs = [
        "obstacle_blocking_analyzer.cc",
    ],
    hdrs = [
        "obstacle_blocking_analyzer.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":frame",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/planning/reference_line",
    ],
)

cc_library(
    name = "path_boundary",
    srcs = [
        "path_boundary.cc",
    ],
    hdrs = [
        "path_boundary.h",
    ],
    deps = [
    ],
)

cc_library(
    name = "planning_context",
    srcs = [
        "planning_context.cc",
    ],
    hdrs = [
        "planning_context.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":planning_gflags",
        "//cyber/common:log",
        "//modules/map/pnc_map:path",
        "//modules/planning/common/path:path_data",
        "//modules/planning/proto:path_decider_info_proto",
        "//modules/planning/proto:planning_config_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/proto:planning_status_proto",
        "@eigen",
    ],
)

cc_library(
    name = "path_decision",
    srcs = [
        "path_decision.cc",
    ],
    hdrs = [
        "path_decision.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":obstacle",
        "//modules/planning/reference_line",
    ],
)

cc_library(
    name = "planning_gflags",
    srcs = [
        "planning_gflags.cc",
    ],
    hdrs = [
        "planning_gflags.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//external:gflags",
    ],
)

cc_library(
    name = "reference_line_info",
    srcs = [
        "reference_line_info.cc",
    ],
    hdrs = [
        "reference_line_info.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":ego_info",
        ":path_boundary",
        ":path_decision",
        ":planning_gflags",
        ":st_graph_data",
        "//cyber/common:log",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/pnc_map",
        "//modules/map/proto:map_proto",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/common/trajectory:publishable_trajectory",
        "//modules/planning/proto:lattice_structure_proto",
        "//modules/planning/reference_line",
        "@eigen",
    ],
)

cc_test(
    name = "reference_line_info_test",
    size = "small",
    srcs = [
        "reference_line_info_test.cc",
    ],
    deps = [
        ":reference_line_info",
        "@gtest//:main",
    ],
)

cc_library(
    name = "speed_profile_generator",
    srcs = [
        "speed_profile_generator.cc",
    ],
    hdrs = [
        "speed_profile_generator.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":frame",
        ":reference_line_info",
        "//modules/common/proto:pnc_point_proto",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/math/curve1d",
        "//modules/planning/math/curve1d:polynomial_curve1d",
        "//modules/planning/math/curve1d:quartic_polynomial_curve1d",
        "//modules/planning/math/curve1d:quintic_polynomial_curve1d",
        "//modules/planning/math/piecewise_jerk:piecewise_jerk_speed_problem",
        "//modules/planning/proto:planning_config_proto",
    ],
)

cc_test(
    name = "speed_profile_generator_test",
    size = "small",
    srcs = [
        "speed_profile_generator_test.cc",
    ],
    deps = [
        ":speed_profile_generator",
        "@gtest//:main",
    ],
)

cc_library(
    name = "local_view",
    hdrs = [
        "local_view.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//modules/localization/proto:localization_proto",
        "//modules/map/relative_map/proto:navigation_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/prediction/proto:prediction_proto",
    ],
)

cc_library(
    name = "frame",
    srcs = [
        "frame.cc",
        "frame_manager.cc",
    ],
    hdrs = [
        "frame.h",
        "frame_manager.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":indexed_queue",
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
        "//modules/planning/proto:planning_config_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/reference_line:reference_line_provider",
        "@eigen",
    ],
)

cc_test(
    name = "frame_test",
    size = "small",
    srcs = [
        "frame_test.cc",
    ],
    data = [
        "//modules/planning/common:common_testdata",
    ],
    deps = [
        ":frame",
        "//modules/common/util",
        "//modules/map/hdmap:hdmap_util",
        "//modules/planning/proto:planning_config_proto",
        "@gtest//:main",
    ],
)

cc_library(
    name = "speed_limit",
    srcs = [
        "speed_limit.cc",
    ],
    hdrs = [
        "speed_limit.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//modules/common/math",
        "//modules/planning/proto:planning_proto",
    ],
)

cc_test(
    name = "speed_limit_test",
    size = "small",
    srcs = [
        "speed_limit_test.cc",
    ],
    deps = [
        ":speed_limit",
        "@gtest//:main",
    ],
)

cc_library(
    name = "ego_info",
    srcs = [
        "ego_info.cc",
    ],
    hdrs = [
        "ego_info.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":obstacle",
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/configs/proto:vehicle_config_proto",
        "//modules/common/math:geometry",
        "//modules/common/vehicle_state/proto:vehicle_state_proto",
        "//modules/planning/reference_line",
        "@eigen",
    ],
)

cc_test(
    name = "ego_info_test",
    size = "small",
    srcs = [
        "ego_info_test.cc",
    ],
    deps = [
        ":ego_info",
        ":frame",
        "@gtest//:main",
    ],
)

cc_library(
    name = "planning_common",
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":ego_info",
        ":frame",
        ":planning_gflags",
        ":speed_limit",
        ":st_graph_data",
        "//cyber/common:log",
        "//modules/localization/common:localization_common",
        "//modules/localization/proto:localization_proto",
        "@eigen",
    ],
)

cc_library(
    name = "trajectory_stitcher",
    srcs = [
        "trajectory_stitcher.cc",
    ],
    hdrs = [
        "trajectory_stitcher.h",
    ],
    deps = [
        "//cyber/common:log",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/vehicle_model",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/trajectory:publishable_trajectory",
        "//modules/planning/reference_line",
    ],
)

cc_library(
    name = "open_space_info",
    srcs = [
        "open_space_info.cc",
    ],
    hdrs = [
        "open_space_info.h",
    ],
    deps = [
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/vehicle_state/proto:vehicle_state_proto",
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
    srcs = [
        "open_space_info_test.cc",
    ],
    linkopts = [
        "-lboost_system",
    ],
    deps = [
        "open_space_info",
        "@gtest//:main",
    ],
)

cc_library(
    name = "st_graph_data",
    srcs = [
        "st_graph_data.cc",
    ],
    hdrs = [
        "st_graph_data.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":planning_gflags",
        ":speed_limit",
        "//modules/planning/common/speed:st_boundary",
    ],
)

cc_test(
    name = "st_graph_data_test",
    size = "small",
    srcs = [
        "st_graph_data_test.cc",
    ],
    deps = [
        ":st_graph_data",
        "//cyber/common:log",
        "//modules/common/time",
        "//modules/common/util",
        "@gtest//:main",
    ],
)

cpplint()
