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
        "//modules/common/proto:path_point_proto",
        "//modules/planning/common:planning_gflags",
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
        "//modules/common:log",
        "//modules/common/math:math",
        "//modules/planning/proto:planning_proto",
        ":st_graph_point",
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
        "//modules/common/proto:path_point_proto",
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
        "//modules/map/proto:map_proto",
        "//modules/common/configs/proto:vehicle_config_proto",
        "//modules/map/hdmap:hdmap",
        "//modules/planning/reference_line:reference_line",
        "//modules/planning/common:data_center",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/path:discretized_path",
        "//modules/planning/common/path:frenet_frame_path",
        "//modules/planning/common:speed_limit",
        "//modules/planning/math:double",
        "//modules/planning/state_machine:master_state_machine",
        "//modules/common/proto:path_point_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/common:planning_gflags",
    ],
)


