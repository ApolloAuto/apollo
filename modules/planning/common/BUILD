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
        "logic_node.cc",
        "logic_point.cc",
    ],
    hdrs = [
        "logic_node.h",
        "logic_point.h",
    ],
    deps = [
        "//modules/common/math",
        "//modules/planning/common:planning_error_code",
    ],
)

cc_library(
    name = "planning_error_code",
    hdrs = [
        "planning_error.h",
    ],
)

cc_library(
    name = "lib_planning_common",
    deps = [
        "@eigen//:eigen",
        ":planning_gflags",
        "//modules/common:log",
        "//modules/localization/common:localization_common",
        "//modules/localization/proto:localization_proto",
    ],
)

cpplint()
