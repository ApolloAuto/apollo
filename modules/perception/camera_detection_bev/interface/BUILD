load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

cc_library(
    name = "base_obstacle_detector",
    srcs = [
        "base_obstacle_detector.cc",
    ],
    hdrs = [
        "base_obstacle_detector.h",
    ],
    deps = [
        "//cyber",
        "//modules/perception/camera_detection_bev:camera_frame",
        "//modules/perception/common/inference:inference_factory",
        "//modules/perception/common/inference:inference_lib",
        "//modules/perception/common/inference:model_util",
        "//modules/perception/common/lib/interface:base_init_options",
        "//modules/perception/common/lib/registerer",
        "//modules/perception/common/proto:model_info_cc_proto",
        "@eigen",
        "@com_google_protobuf//:protobuf",
    ]
)

apollo_package()
cpplint()
