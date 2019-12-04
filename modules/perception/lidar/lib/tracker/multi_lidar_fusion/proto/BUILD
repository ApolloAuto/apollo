load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "multi_lidar_fusion_config_proto",
    deps = [
        ":multi_lidar_fusion_config_lib",
    ],
)

proto_library(
    name = "multi_lidar_fusion_config_lib",
    srcs = ["multi_lidar_fusion_config.proto"],
)

cpplint()
