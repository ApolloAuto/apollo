load("//tools:py_proto.bzl", "py_proto")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "robosense_proto",
    deps = [
        ":sensor_robosense_proto",
    ],
)

cc_proto_library(
    name = "sensor_robosense_proto",
    deps = [
        ":robosense_proto_lib",
    ],
)

proto_library(
    name = "robosense_proto_lib",
    srcs = [
        "config.proto",
        "robosense.proto",
    ],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)

cpplint()