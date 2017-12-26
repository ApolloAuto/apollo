load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "speed_profile_cost",
    srcs = [
        "speed_profile_cost.cc",
    ],
    hdrs = [
        "speed_profile_cost.h",
    ],
    deps = [
        "//modules/planning/math/curve1d:quartic_polynomial_curve1d",
        "//modules/planning/proto:planning_config_proto",
        "//modules/planning/proto:poly_st_speed_config_proto",
        "//modules/planning/tasks:speed_optimizer",
        "//modules/planning/tasks/st_graph:st_boundary_mapper",
    ],
)

cc_library(
    name = "poly_st_graph",
    srcs = [
        "poly_st_graph.cc",
    ],
    hdrs = [
        "poly_st_graph.h",
    ],
    deps = [
        ":speed_profile_cost",
        "//modules/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/configs/proto:vehicle_config_proto",
        "//modules/common/proto:pnc_point_proto",
        "//modules/localization/proto:pose_proto",
        "//modules/map/hdmap",
        "//modules/planning/math/curve1d:quartic_polynomial_curve1d",
        "//modules/planning/proto:planning_config_proto",
        "//modules/planning/proto:poly_st_speed_config_proto",
        "//modules/planning/tasks:speed_optimizer",
        "//modules/planning/tasks/st_graph:st_boundary_mapper",
    ],
)

cc_library(
    name = "poly_st_speed_optimizer",
    srcs = [
        "poly_st_speed_optimizer.cc",
    ],
    hdrs = [
        "poly_st_speed_optimizer.h",
    ],
    deps = [
        ":poly_st_graph",
        "//modules/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/configs/proto:vehicle_config_proto",
        "//modules/common/proto:pnc_point_proto",
        "//modules/localization/proto:pose_proto",
        "//modules/map/hdmap",
        "//modules/planning/proto:planning_config_proto",
        "//modules/planning/proto:poly_st_speed_config_proto",
        "//modules/planning/tasks:speed_optimizer",
        "//modules/planning/tasks/st_graph:st_boundary_mapper",
    ],
)

cpplint()
