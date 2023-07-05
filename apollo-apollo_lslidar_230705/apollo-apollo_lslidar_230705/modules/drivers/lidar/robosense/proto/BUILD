load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "sensor_suteng_cc_proto",
    deps = [
        ":sensor_suteng_conf_proto_lib",
    ],
)

proto_library(
    name = "sensor_suteng_conf_proto_lib",
    srcs = [
        "lidars_filter_config.proto",
        "sensor_suteng_conf.proto",
    ],
    deps = [
        ":sensor_suteng_proto_lib",
    ],
)

proto_library(
    name = "sensor_suteng_proto_lib",
    srcs = [
        "sensor_suteng.proto",
    ],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

cpplint()
