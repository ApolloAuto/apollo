load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "decision_data",
    srcs = [
        "decision_data.cc",
    ],
    hdrs = [
        "decision_data.h",
    ],
    deps = [
        "//modules/planning/common:map_object",
        "//modules/planning/common:obstacle",
    ],
)

cc_library(
    name = "map_object",
    srcs = [
        "map_object.cc",
    ],
    hdrs = [
        "map_object.h",
    ],
    deps = [
        "//modules/planning/common:planning_object",
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
        "//modules/planning/common:planning_object",
        "//modules/planning/common/trajectory:prediction_trajectory",
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
    name = "planning_object",
    srcs = [
        "planning_object.cc",
    ],
    hdrs = [
        "planning_object.h",
    ],
    deps = [
        "//modules/common/math:polygon2d",
        "//modules/planning/proto:planning_proto",
    ],
)

cc_library(
    name = "logic_map",
    srcs = [
        "logic_info.cc",
        "logic_node.cc",
        "logic_point.cc",
    ],
    hdrs = [
        "logic_info.h",
        "logic_node.h",
        "logic_point.h",
    ],
    deps = [
        "//modules/common/math",
        "//modules/common/proto:error_code_proto",
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
        ":decision_data",
        "//modules/common/proto:path_point_proto",
        "//modules/map/proto:map_proto",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/common/trajectory:publishable_trajectory",
        "//modules/planning/reference_line",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "em_planning_data",
    srcs = [
        "em_planning_data.cc",
    ],
    hdrs = [
        "em_planning_data.h",
    ],
    deps = [
        ":planning_data",
        "//modules/common/proto:path_point_proto",
        "//modules/planning/math:double",
    ],
)

cc_library(
    name = "environment",
    srcs = [
        "environment.cc",
    ],
    hdrs = [
        "environment.h",
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
        "//modules/common:log",
        "//modules/planning/common:environment",
        "//modules/planning/common:planning_data",
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
    name = "data_center",
    srcs = [
        "data_center.cc",
    ],
    hdrs = [
        "data_center.h",
    ],
    deps = [
        ":environment",
        ":frame",
        ":planning_gflags",
        "//modules/common",
        "//modules/common/status",
        "//modules/map/hdmap",
        "//modules/planning/state_machine:master_state_machine",
    ],
)

cc_library(
    name = "planning_common",
    deps = [
        ":data_center",
        ":em_planning_data",
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
