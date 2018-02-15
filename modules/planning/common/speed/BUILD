load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "st_point",
    srcs = [
        "st_point.cc",
    ],
    hdrs = [
        "st_point.h",
    ],
    deps = [
        "//external:gflags",
        "//modules/common/math",
    ],
)

cc_library(
    name = "st_boundary",
    srcs = [
        "st_boundary.cc",
    ],
    hdrs = [
        "st_boundary.h",
    ],
    deps = [
        ":st_point",
        "//modules/common:log",
        "//modules/planning/proto:planning_proto",
        "@gtest",
    ],
)

cc_test(
    name = "st_boundary_test",
    size = "small",
    srcs = [
        "st_boundary_test.cc",
    ],
    deps = [
        ":st_boundary",
        "//modules/common:log",
        "//modules/common/time",
        "//modules/common/util",
        "@gtest//:main",
    ],
)

cc_library(
    name = "speed_data",
    srcs = [
        "speed_data.cc",
    ],
    hdrs = [
        "speed_data.h",
    ],
    deps = [
        ":st_point",
        "//external:gflags",
        "//modules/common/math",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:planning_util",
        "//modules/planning/proto:planning_proto",
    ],
)

cpplint()
