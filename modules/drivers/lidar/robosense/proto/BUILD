load("//tools:cpplint.bzl", "cpplint")
load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library")

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
        "//modules/common/proto:header_proto",
    ],
)

cpplint()
