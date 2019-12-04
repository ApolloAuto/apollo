load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "trafficlight_detector_detection_proto",
    deps = [
        ":trafficlight_detector_detection_lib",
    ],
)

proto_library(
    name = "trafficlight_detector_detection_lib",
    srcs = ["detection.proto"],
)

cc_library(
    name = "detection",
    srcs = ["detection.cc"],
    hdrs = ["detection.h"],
    deps = [
        ":cropbox",
        ":select",
        ":trafficlight_detector_detection_proto",
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/inference:inference_factory_lib",
        "//modules/perception/inference:inference_lib",
        "//modules/perception/inference/utils:inference_resize_lib",
        "//modules/perception/inference/utils:inference_util_lib",
    ],
    alwayslink = True,
)

cc_library(
    name = "select",
    srcs = ["select.cc"],
    hdrs = ["select.h"],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/common/graph:hungarian_optimizer",
        "//modules/perception/common/graph:secure_matrix",
    ],
)

cc_library(
    name = "cropbox",
    srcs = ["cropbox.cc"],
    hdrs = ["cropbox.h"],
    deps = [
        "//modules/perception/base",
        "//modules/perception/camera/common",
    ],
)

cpplint()
