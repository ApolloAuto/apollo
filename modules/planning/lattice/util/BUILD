load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "collision_checker",
    srcs = [
        "collision_checker.cc",
    ],
    hdrs = [
        "collision_checker.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/math:box2d",
        "//modules/planning/common:frame",
        "//modules/planning/common:obstacle",
        "//modules/planning/common/trajectory:discretized_trajectory",
        ":lattice_params",
        "//modules/prediction/proto:prediction_proto",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/planning/proto:lattice_structure_proto"
    ],
)

cc_library(
    name = "lattice_constraint_checker",
    srcs = [
        "lattice_constraint_checker.cc",
    ],
    hdrs = [
        "lattice_constraint_checker.h",
    ],
    deps = [
        "//modules/planning/math/curve1d:curve1d",
        "//modules/planning/common:planning_gflags",
        "//modules/common:log",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/common:frame",
    ],
)

cc_library(
    name = "reference_line_matcher",
    hdrs = [
        "reference_line_matcher.h",
    ],
    srcs = [
        "reference_line_matcher.cc",
    ],
    deps = [
        "//modules/common:common",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/math:hermite_spline",
        "//modules/common/proto:pnc_point_proto",
        "//modules/planning/common:planning_common",
        "@glog//:glog",
    ],
)

cc_library(
    name = "lattice_trajectory1d",
    hdrs = [
        "lattice_trajectory1d.h",
    ],
    srcs = [
        "lattice_trajectory1d.cc",
    ],
    deps = [
        "//modules/planning/math/curve1d:curve1d",
    ],
)

cc_library(
    name = "util",
    hdrs = [
        "lattice_util.h",
    ],
    srcs = [
        "lattice_util.cc",
    ],
    deps = [
        "//modules/common/proto:pnc_point_proto",
        "//modules/planning/reference_line:reference_line",
        "//modules/planning/math/frame_conversion:cartesian_frenet_conversion",
    ],
)

cc_library(
    name = "lattice_params",
    hdrs = ["lattice_params.h"],
    deps = [],
)

cc_library(
    name = "lattice_util",
    deps = [
        ":collision_checker", 
        ":lattice_constraint_checker", 
        ":reference_line_matcher",
        ":util",
        ":lattice_params",
    ],
)

cpplint()
