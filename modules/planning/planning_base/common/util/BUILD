load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "util_lib",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":print_debug_info",
        "//cyber",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common:reference_line_info",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_command_cc_proto",
    ],
)

cc_library(
    name = "common_lib",
    srcs = ["libcommon_util.so"],
    hdrs = ["common.h"],
    copts = PLANNING_COPTS,
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/planning/planning_base/common:frame",
        "//modules/planning/planning_base/common:reference_line_info",
    ],
)

cc_binary(
    name = "libcommon_util.so",
    srcs = ["common.cc", "common.h"],
    copts = PLANNING_COPTS,
    linkshared = True,
    linkstatic = True,
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/planning/planning_base/common:frame",
        "//modules/planning/planning_base/common:reference_line_info",
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

cc_library(
    name = "print_debug_info",
    srcs = ["print_debug_info.cc"],
    hdrs = ["print_debug_info.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/planning/planning_base/common:planning_gflags",
    ],
)

apollo_package()
cpplint()
