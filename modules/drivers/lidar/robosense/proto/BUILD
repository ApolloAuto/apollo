load("//tools:apollo_package.bzl", "apollo_package")
load("//tools/proto:proto.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "sensor_suteng_conf_proto",
    srcs = [":sensor_suteng_conf.proto"],
    deps = [
        ":sensor_suteng_proto",
    ],
)

proto_library(
    name = "lidars_filter_conf_proto",
    srcs = [":lidars_filter_config.proto"],
)

proto_library(
    name = "sensor_suteng_proto",
    srcs = [":sensor_suteng.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

apollo_package()
