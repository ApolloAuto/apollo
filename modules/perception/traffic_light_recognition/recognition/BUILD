load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "classify",
    srcs = ["classify.cc"],
    hdrs = ["classify.h"],
    deps = [
        "//modules/perception/common/base",
        "//modules/perception/common/camera/common",
        "//modules/perception/common/inference:inference_factory",
        "//modules/perception/common/inference:inference_lib",
        "//modules/perception/common/inference:model_util",
        "//modules/perception/common/inference/utils:inference_resize_lib",
        "//modules/perception/common/inference/utils:inference_util_lib",
        "//modules/perception/traffic_light_recognition/interface:base_traffic_light_recognitor",
        "//modules/perception/traffic_light_recognition/recognition/proto:model_param_cc_proto",
    ],
)

cc_library(
    name = "recognition",
    srcs = ["recognition.cc"],
    hdrs = ["recognition.h"],
    deps = [
        ":classify",
        "//cyber",
        "//modules/perception/common:util",
        "//modules/perception/common/inference:inference_lib",
        "//modules/perception/traffic_light_recognition/interface:base_traffic_light_recognitor",
        "//modules/perception/traffic_light_recognition/recognition/proto:model_param_cc_proto",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
