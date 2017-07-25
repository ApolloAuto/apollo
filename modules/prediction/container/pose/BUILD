load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pose_container",
    srcs = ["pose_container.cc"],
    hdrs = ["pose_container.h"],
    deps = [
        "//modules/prediction/container:container",
        "//modules/perception/proto:perception_proto",
        "//modules/localization/proto:localization_proto",
    ]
)

cpplint()