load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "st_graph_point",
    srcs = [
        "st_graph_point.cc",
    ],
    hdrs = [
        "st_graph_point.h",
    ],
    deps = [
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/speed:st_point",
    ],
)

cc_library(
    name = "st_graph_boundary",
    srcs = [
        "st_graph_boundary.cc",
    ],
    hdrs = [
        "st_graph_boundary.h",
    ],
    deps = [
        ":st_graph_point",
        "//modules/common:log",
        "//modules/common/math",
        "//modules/planning/proto:planning_proto",
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
    deps = [
        ":st_graph_boundary",
        "//modules/planning/common:planning_gflags",
    ],
)

cc_library(
    name = "st_boundary_mapper",
    srcs = [
        "st_boundary_mapper.cc",
    ],
    hdrs = [
        "st_boundary_mapper.h",
    ],
    deps = [
        ":st_graph_boundary",
        "//modules/common/configs/proto:vehicle_config_proto",
        "//modules/common/proto:path_point_proto",
        "//modules/common/status",
        "//modules/map/hdmap",
        "//modules/map/proto:map_proto",
        "//modules/planning/common:decision_data",
        "//modules/planning/common:map_object",
        "//modules/planning/common:obstacle",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:planning_object",
        "//modules/planning/common:speed_limit",
        "//modules/planning/common/path:discretized_path",
        "//modules/planning/common/path:frenet_frame_path",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/common/trajectory:prediction_trajectory",
        "//modules/planning/math:double",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/reference_line",
        "//modules/planning/state_machine:master_state_machine",
    ],
)

cpplint()
