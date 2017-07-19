load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

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
        "//modules/planning/common:planning_data",
        "//modules/planning/common:environment",
        "//modules/common:log",
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
        "//modules/planning/common/speed",
    ],
)

cc_library(
    name = "lib_planning_common",
    deps = [
        "@eigen//:eigen",
        ":planning_gflags",
        ":frame",
        ":speed_limit",
        "//modules/common:log",
        "//modules/localization/common:localization_common",
        "//modules/localization/proto:localization_proto",
    ],
)

cpplint()
