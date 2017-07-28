load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pose_container",
    srcs = ["pose_container.cc"],
    hdrs = ["pose_container.h"],
    deps = [
        "//modules/prediction/container",
        "//modules/localization/proto:localization_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/common:log",
    ],
)

cpplint()
