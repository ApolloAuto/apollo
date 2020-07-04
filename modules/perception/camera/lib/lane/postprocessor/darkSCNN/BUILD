load("//tools:cpplint.bzl", "cpplint")
load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "darkSCNN_postprocessor_proto",
    deps = [
        ":darkSCNN_postprocessor_proto_lib",
    ],
)

proto_library(
    name = "darkSCNN_postprocessor_proto_lib",
    srcs = ["darkSCNN_postprocessor.proto"],
)

cc_library(
    name = "darkSCNN_lane_postprocessor",
    srcs = ["darkSCNN_lane_postprocessor.cc"],
    hdrs = ["darkSCNN_lane_postprocessor.h"],
    deps = [
        ":darkSCNN_postprocessor_proto",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/lane/common:common_functions",
        "//modules/perception/camera/lib/lane/common:darkSCNN_proto",
        "//modules/perception/lib/registerer",
        "//modules/perception/lib/utils",
    ],
)

py_proto(
    name = "darkSCNN_postprocessor_pb2",
    src = "darkSCNN_postprocessor.proto",
)

cpplint()
