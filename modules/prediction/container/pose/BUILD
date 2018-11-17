load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pose_container",
    srcs = ["pose_container.cc"],
    hdrs = ["pose_container.h"],
    deps = [
        "//modules/common:log",
        "//modules/common/math:quaternion",
        "//modules/localization/proto:localization_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/container",
    ],
)

cc_test(
    name = "pose_container_test",
    size = "small",
    srcs = ["pose_container_test.cc"],
    deps = [
        "//modules/prediction/container/pose:pose_container",
        "@gtest//:main",
    ],
)

cpplint()
