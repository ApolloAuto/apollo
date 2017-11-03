load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_lib",
    srcs = ["perception.cc"],
    hdrs = ["perception.h"],
    deps = [
        "//modules/common:apollo_app",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/perception/lib/base:base",
        "//modules/perception/common:perception_common",
        "//modules/perception/onboard:perception_onboard",
        "//modules/perception/obstacle/onboard:perception_obstacle_onboard",

    ],
)

cc_binary(
    name = "perception",
    srcs = ["main.cc"],
    data = [
        "//modules/perception/conf:perception_adapter_manager_config",
    ],
    deps = [
        ":perception_lib",
        "//external:gflags",
        "//modules/common:log",
        "//modules/perception/common:perception_common",
        "//modules/perception/proto:perception_proto",
        "@ros//:ros_common",
    ],
)

filegroup(
    name = "perception_data",
    srcs = glob([
        "data/**",
    ]),
)

filegroup(
    name = "perception_model",
    srcs = glob([
        "model/**",
    ]),
)

cpplint()
