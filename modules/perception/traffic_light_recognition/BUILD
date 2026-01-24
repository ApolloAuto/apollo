load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"trafficlight\\"']

filegroup(
    name = "traffic_light_recognition_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
        "launch/**",
    ]),
)

apollo_cc_library(
    name = "apollo_perception_traffic_light_recognition",
    srcs = [
        "recognition/caffe_recognizer/classify.cc",
        "recognition/caffe_recognizer/recognition.cc",
        "recognition/efficient_net/efficient_net.cc",
    ],
    hdrs = [
        "interface/base_traffic_light_recognitor.h",
        "recognition/caffe_recognizer/classify.h",
        "recognition/caffe_recognizer/recognition.h",
        "recognition/efficient_net/efficient_net.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        "//cyber",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/common/inference:apollo_perception_common_inference",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/traffic_light_recognition/proto:trafficlights_recognition_component_cc_proto",
        "//modules/perception/traffic_light_recognition/recognition/caffe_recognizer/proto:model_param_cc_proto",
        "//modules/perception/traffic_light_recognition/recognition/efficient_net/proto:efficient_net_param_cc_proto",
    ],
)

apollo_component(
    name = "libcomponent_traffic_light_recognition.so",
    srcs = [
        "traffic_light_recognition_component.cc",
    ],
    hdrs = [
        "traffic_light_recognition_component.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        "//cyber",
        ":apollo_perception_traffic_light_recognition",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/traffic_light_recognition/proto:trafficlights_recognition_component_cc_proto",
    ],
)

apollo_package(enable_source=False)

cpplint()
