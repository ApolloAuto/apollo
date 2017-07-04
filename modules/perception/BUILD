load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_lib",
    srcs = ["perception.cc"],
    hdrs = ["perception.h"],
    deps = [
        "//modules/common:apollo_app",
        "//modules/common/adapters:adapter_manager",
        "//modules/perception/common:perception_common",
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
        "//third_party/ros:ros_common",
    ],
)

cpplint()
