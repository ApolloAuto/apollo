load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dp_st_boundary_mapper",
    srcs = [
        "dp_st_boundary_mapper.cc",
    ],
    hdrs = [
        "dp_st_boundary_mapper.h",
    ],
    deps = [
        "//modules/common/math:math",
        "//modules/common/proto:common_proto",
        "//modules/planning/common:data_center",
        "//modules/planning/optimizer/st_graph:st_boundary_mapper",
        "//modules/planning/proto:planning_proto",
    ],
)

cc_library(
    name = "dp_st_configuration",
    srcs = [
        "dp_st_configuration.cc",
    ],
    hdrs = [
        "dp_st_configuration.h",
    ],
)

cc_library(
    name = "dp_st_cost",
    srcs = [
        "dp_st_cost.cc",
    ],
    hdrs = [
        "dp_st_cost.h",
    ],
    deps = [
        ":dp_st_configuration",
        "//modules/common/proto:path_point_proto",
        "//modules/planning/common:data_center",
        "//modules/planning/math:double",
        "//modules/planning/optimizer/st_graph:st_graph_boundary",
    ],
)

cc_library(
    name = "dp_st_graph",
    srcs = [
        "dp_st_graph.cc",
    ],
    hdrs = [
        "dp_st_graph.h",
    ],
    deps = [
        ":dp_st_configuration",
        ":dp_st_cost",
        "//modules/common/configs/proto:vehicle_config_proto",
        "//modules/common/proto:common_proto",
        "//modules/planning/common:data_center",
        "//modules/planning/common:decision_data",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/optimizer/st_graph:st_graph_data",
        "//modules/planning/proto:planning_proto",
    ],
)

cc_library(
    name = "dp_st_speed_optimizer",
    srcs = [
        "dp_st_speed_optimizer.cc",
    ],
    hdrs = [
        "dp_st_speed_optimizer.h",
    ],
    deps = [
        ":dp_st_boundary_mapper",
        ":dp_st_configuration",
        ":dp_st_cost",
        ":dp_st_graph",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/planning/optimizer:speed_optimizer",
        "//modules/planning/optimizer/st_graph:st_graph_data",
    ],
)

cpplint()