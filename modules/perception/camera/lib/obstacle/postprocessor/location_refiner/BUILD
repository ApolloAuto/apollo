load("//tools:cpplint.bzl", "cpplint")
load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library")
load("@com_github_grpc_grpc//bazel:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "location_refiner_cc_proto",
    deps = [
        ":location_refiner_proto",
    ],
)

proto_library(
    name = "location_refiner_proto",
    srcs = ["location_refiner.proto"],
)

cc_library(
    name = "location_refiner_obstacle_postprocessor",
    srcs = ["location_refiner_obstacle_postprocessor.cc"],
    hdrs = ["location_refiner_obstacle_postprocessor.h"],
    deps = [
        ":location_refiner_cc_proto",
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

py_proto_library(
    name = "location_refiner_py_pb2",
    deps = [
        ":location_refiner_proto",
    ],
)

cpplint()
