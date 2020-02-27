load("//tools:cpplint.bzl", "cpplint")
load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "location_refiner_proto",
    deps = [
        ":location_refiner_proto_lib",
    ],
)

proto_library(
    name = "location_refiner_proto_lib",
    srcs = ["location_refiner.proto"],
)

cc_library(
    name = "location_refiner_obstacle_postprocessor",
    srcs = ["location_refiner_obstacle_postprocessor.cc"],
    hdrs = ["location_refiner_obstacle_postprocessor.h"],
    deps = [
        ":location_refiner_proto",
        ":obj_postprocessor",
        "//cyber",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
    ],
)

cc_library(
    name = "obj_postprocessor",
    srcs = ["obj_postprocessor.cc"],
    hdrs = ["obj_postprocessor.h"],
    deps = [
        "//modules/perception/camera/common",
    ],
)

py_proto(
    name = "location_refiner_pb2",
    src = "location_refiner.proto",
)

cpplint()
