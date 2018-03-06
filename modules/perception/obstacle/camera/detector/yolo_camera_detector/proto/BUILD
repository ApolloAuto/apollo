syntax = "proto2"

load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "yolo_camera_proto",
    deps = [
        ":yolo_camera_proto_lib",
    ],
)

proto_library(
    name = "yolo_camera_proto_lib",
    srcs = [
        "yolo.proto",
    ],
)

cpplint()
