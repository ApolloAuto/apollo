load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "trafficlight_detector_recognition_proto",
    deps = [ 
        ":trafficlight_detector_recognition_lib",
    ],  
)

proto_library(
    name = "trafficlight_detector_recognition_lib",
    srcs = [ 
        "recognition.proto",
    ],  
)

cc_library(
    name = "classify",
    srcs = [
        "classify.cc",
    ],
    hdrs = [ 
        "classify.h",
    ],
    deps = [
        ":trafficlight_detector_recognition_proto",
        "//modules/perception/base:base",
        "//modules/perception/camera/common:common",
        "//modules/perception/camera/lib/interface:interface",
        "//modules/perception/inference:inference_factory_lib",
        "//modules/perception/inference:inference_lib",
        "//modules/perception/inference/utils:inference_resize_lib",
        "//modules/perception/inference/utils:inference_util_lib",
        "//modules/perception/lib/io:file_util",
    ],
)

cc_library(
    name = "recognition",
    srcs = [
        "recognition.cc",
    ],
    hdrs = [ 
        "recognition.h",
    ],
    deps = [
        ":classify",
        ":trafficlight_detector_recognition_proto",
        "//modules/perception/inference:inference_lib",
        "//modules/perception/camera/lib/interface:interface",
        "//modules/perception/lib/io:file_util",
        "//modules/common/util:file_util",
    ],
)

cpplint()
