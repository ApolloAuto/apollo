load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

apollo_cc_library(
    name = "base_obstacle_detector",
    srcs = [
        "base_obstacle_detector.cc",
    ],
    hdrs = [
        "base_obstacle_detector.h",
    ],
    deps = [
        "//cyber",
        "//modules/perception/common/inference:apollo_perception_common_inference",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/common/proto:model_info_cc_proto",
        "@eigen",
        "@com_google_protobuf//:protobuf",
    ]
)

apollo_cc_library(
    name = "base_down_sample",
    hdrs = [
        "base_down_sample.h",
    ],
    deps = [
        "//cyber",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/lib:apollo_perception_common_lib",
    ]
)

apollo_package()
cpplint()
