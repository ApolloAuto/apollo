load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "indexed_list",
    hdrs = [
        "indexed_list.h",
    ],
)

cc_test(
    name = "indexed_list_test",
    srcs = [
        "indexed_list_test.cc",
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
)

cc_library(
    name = "obstacle",
    srcs = [
        "obstacle.cc",
    ],
    hdrs = [
        "obstacle.h",
    ],
    deps = [
        "//modules/common/math:box2d",
        "//modules/common/math:polygon2d",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/common:planning_util",
        "//modules/planning/proto:planning_proto",
        "//modules/prediction/proto:prediction_proto",
    ],
)

cc_library(
    name = "path_obstacle",
    srcs = [
        "path_obstacle.cc",
    ],
    hdrs = [
        "path_obstacle.h",
    ],
    deps = [
        ":obstacle",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/reference_line",
    ],
)

cc_library(
    name = "planning_util",
    srcs = [
        "planning_util.cc",
    ],
    hdrs = [
        "planning_util.h",
    ],
    deps = [
        "//modules/common/math",
        "//modules/common/proto:pnc_point_proto",
        "//modules/planning/math:hermite_spline",
        "//modules/planning/proto:planning_proto",
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
    deps = [
        ":indexed_list",
        ":obstacle",
        ":path_obstacle",
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
    deps = [
        "//external:gflags",
    ],
)

cc_library(
    name = "planning_data",
    srcs = [
        "planning_data.cc",
    ],
    hdrs = [
        "planning_data.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/proto:pnc_point_proto",
        "//modules/map/proto:map_proto",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/common/trajectory:publishable_trajectory",
        "//modules/planning/reference_line",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "frame",
    srcs = [
        "frame.cc",
    ],
    hdrs = [
        "frame.h",
    ],
    deps = [
        ":indexed_list",
        ":indexed_queue",
        ":obstacle",
        ":path_decision",
        ":path_obstacle",
        ":planning_data",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/vehicle_state",
        "//modules/map/pnc_map",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/common/trajectory:publishable_trajectory",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/proto:reference_line_smoother_config_proto",
        "//modules/planning/reference_line:reference_line_smoother",
    ],
)

cc_library(
    name = "planning_state",
    srcs = [
        "planning_state.cc",
    ],
    hdrs = [
        "planning_state.h",
    ],
    deps = [
        "//modules/common:macro",
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
    deps = [
        "//modules/common/math",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/proto:planning_proto",
    ],
)

cc_library(
    name = "planning_common",
    deps = [
        ":frame",
        ":planning_gflags",
        ":speed_limit",
        "//modules/common:log",
        "//modules/localization/common:localization_common",
        "//modules/localization/proto:localization_proto",
        "@eigen//:eigen",
    ],
)

cpplint()
