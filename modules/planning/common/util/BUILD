load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "util_lib",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
    ],
)

cc_library(
    name = "common_lib",
    srcs = ["common.cc"],
    hdrs = ["common.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/planning/common:frame",
        "//modules/planning/common:reference_line_info",
    ],
)

cc_library(
    name = "math_util_lib",
    srcs = ["math_util.cc"],
    hdrs = ["math_util.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
        "//modules/common/math",
    ],
)

cpplint()
