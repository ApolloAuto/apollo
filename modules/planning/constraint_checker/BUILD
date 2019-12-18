load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "constraint_checker",
    srcs = ["constraint_checker.cc"],
    hdrs = ["constraint_checker.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//cyber/common:log",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/trajectory:discretized_trajectory",
    ],
)

cc_library(
    name = "constraint_checker1d",
    srcs = ["constraint_checker1d.cc"],
    hdrs = ["constraint_checker1d.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//cyber/common:log",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/math/curve1d",
    ],
)

cc_library(
    name = "collision_checker",
    srcs = ["collision_checker.cc"],
    hdrs = ["collision_checker.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math:geometry",
        "//modules/common/math:path_matcher",
        "//modules/planning/common:frame",
        "//modules/planning/common:obstacle",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/lattice/behavior:path_time_graph",
        "//modules/prediction/proto:prediction_proto",
    ],
)

cpplint()
