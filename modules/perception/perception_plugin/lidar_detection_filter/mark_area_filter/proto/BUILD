load("//tools/proto:proto.bzl", "proto_library")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "mark_area_filter_proto",
    srcs = ["mark_area_filter.proto"],
    deps = [
        "//modules/common_msgs/map_msgs:map_geometry_proto",
        "//modules/common_msgs/basic_msgs:geometry_proto",
    ],
)

apollo_package()
