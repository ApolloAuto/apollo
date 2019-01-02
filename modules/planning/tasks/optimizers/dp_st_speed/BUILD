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
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/speed:st_point",
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
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":st_graph_point",
        "//modules/common/proto:pnc_point_proto",
        "//modules/planning/common:frame",
        "//modules/planning/common:obstacle",
        "//modules/planning/common/speed:st_boundary",
        "//modules/planning/proto:dp_st_speed_config_proto",
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
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":dp_st_cost",
        ":st_graph_point",
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/configs/proto:vehicle_config_proto",
        "//modules/common/proto:geometry_proto",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/planning/common:obstacle",
        "//modules/planning/common:path_decision",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/proto:planning_config_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/tasks/optimizers/st_graph:st_graph_data",
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
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":dp_st_cost",
        ":dp_st_graph",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/localization/proto:localization_proto",
        "//modules/planning/proto:dp_st_speed_config_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/tasks/optimizers:speed_optimizer",
        "//modules/planning/tasks/optimizers/st_graph:st_boundary_mapper",
        "//modules/planning/tasks/optimizers/st_graph:st_graph_data",
    ],
)

cc_test(
    name = "dp_st_graph_test",
    size = "small",
    srcs = [
        "dp_st_graph_test.cc",
    ],
    data = [
        "//modules/planning:planning_conf",
    ],
    deps = [
        ":dp_st_graph",
        "//modules/planning/proto:planning_config_proto",
        "@gtest//:main",
    ],
)

cpplint()
