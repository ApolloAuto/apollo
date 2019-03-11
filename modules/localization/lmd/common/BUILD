load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_lmd_common",
    srcs = [
        "pose_list.cc",
    ],
    hdrs = [
        "pose_list.h",
        "tm_list.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/math",
        "//modules/localization/proto:localization_proto",
    ],
)

cc_test(
    name = "localization_lmd_common_test",
    size = "small",
    srcs = [
        "pose_list_test.cc",
        "tm_list_test.cc",
    ],
    deps = [
        ":localization_lmd_common",
        "@gtest//:main",
    ],
)

cpplint()
