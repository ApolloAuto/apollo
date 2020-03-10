load("//tools:cpplint.bzl", "cpplint")
load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "multicue_proto",
    deps = [
        ":multicue_proto_lib",
    ],
)

proto_library(
    name = "multicue_proto_lib",
    srcs = ["multicue.proto"],
)

cc_library(
    name = "obj_mapper",
    srcs = ["obj_mapper.cc"],
    hdrs = ["obj_mapper.h"],
    deps = [
        "//cyber",
        "//modules/perception/camera/common",
    ],
)

cc_library(
    name = "multicue_obstacle_transformer",
    srcs = ["multicue_obstacle_transformer.cc"],
    hdrs = ["multicue_obstacle_transformer.h"],
    deps = [
        ":multicue_proto",
        ":obj_mapper",
        "//cyber",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
    ],
)

py_proto(
    name = "multicue_pb2",
    src = "multicue.proto",
)

cpplint()
