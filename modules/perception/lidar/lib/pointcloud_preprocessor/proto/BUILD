load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "pointcloud_preprocessor_config_proto",
    deps = [
        ":pointcloud_preprocessor_config_proto_lib",
    ],
)

proto_library(
    name = "pointcloud_preprocessor_config_proto_lib",
    srcs = ["pointcloud_preprocessor_config.proto"],
)

cpplint()
