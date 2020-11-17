load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "velodyne_proto",
    deps = [
        ":sensor_velodyne_proto",
    ],
)

cc_proto_library(
    name = "sensor_velodyne_proto",
    deps = [
        ":velodyne_proto_lib",
    ],
)

proto_library(
    name = "velodyne_proto_lib",
    srcs = [
        "config.proto",
        "velodyne.proto",
    ],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)

cpplint()
