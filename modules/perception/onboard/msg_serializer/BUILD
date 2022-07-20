load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "msg_serializer",
    srcs = ["msg_serializer.cc"],
    hdrs = ["msg_serializer.h"],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/onboard/common_flags",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/common_msgs/prediction_msgs:feature_cc_proto",
        "@eigen",
    ],
)

cpplint()
