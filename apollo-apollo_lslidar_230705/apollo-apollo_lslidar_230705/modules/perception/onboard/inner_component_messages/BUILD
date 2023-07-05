load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "inner_component_messages",
    hdrs = ["inner_component_messages.h"],
)

cc_library(
    name = "lidar_inner_component_messages",
    hdrs = [
        "lidar_inner_component_messages.h",
    ],
    deps = [
        "//cyber",
        "//modules/perception/lidar/common",
        "//modules/perception/onboard/inner_component_messages",
    ],
)

cpplint()
