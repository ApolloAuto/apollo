load("//tools:apollo_package.bzl", "apollo_package")
load("//tools/proto:proto.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "seyond_config_proto",
    srcs = ["seyond_config.proto"],
    deps = [
        ":seyond_proto",
    ],
)

proto_library(
    name = "seyond_proto",
    srcs = ["seyond.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

apollo_package()
