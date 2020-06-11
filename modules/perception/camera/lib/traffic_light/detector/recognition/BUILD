load("//tools:cpplint.bzl", "cpplint")
load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "trafficlight_detector_recognition_proto",
    deps = [
        ":trafficlight_detector_recognition_lib",
    ],
)

proto_library(
    name = "trafficlight_detector_recognition_lib",
    srcs = ["recognition.proto"],
)

cc_library(
    name = "classify",
    srcs = ["classify.cc"],
    hdrs = ["classify.h"],
    deps = [
        ":trafficlight_detector_recognition_proto",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/inference:inference_factory_lib",
        "//modules/perception/inference:inference_lib",
        "//modules/perception/inference/utils:inference_resize_lib",
        "//modules/perception/inference/utils:inference_util_lib",
    ],
)

cc_library(
    name = "recognition",
    srcs = ["recognition.cc"],
    hdrs = ["recognition.h"],
    deps = [
        ":classify",
        ":trafficlight_detector_recognition_proto",
        "//cyber/common:file",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/inference:inference_lib",
    ],
    alwayslink = True,
)

py_proto(
    name = "recognition_pb2",
    src = "recognition.proto",
)

cpplint()
