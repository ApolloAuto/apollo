load("//tools:apollo_package.bzl", "apollo_package")
load("//tools/proto:proto.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "livox_proto",
    srcs = ["livox.proto"],
    deps= [
      "//modules/drivers/lidar/common/proto:lidar_config_base_proto",
    ]
)

apollo_package()
