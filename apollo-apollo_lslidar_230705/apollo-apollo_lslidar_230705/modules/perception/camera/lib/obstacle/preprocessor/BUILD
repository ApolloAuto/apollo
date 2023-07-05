load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "camera_detection_preprocessor",
    srcs = ["camera_detection_preprocessor.cc"],
    hdrs = ["camera_detection_preprocessor.h"],
    deps = [
        "//modules/perception/camera/lib/obstacle/preprocessor/get_image_data",
        "//modules/perception/camera/lib/obstacle/preprocessor/resize_and_normalize",
        "//modules/perception/pipeline:stage",
        "//modules/perception/pipeline:plugin_factory",
        "//modules/perception/pipeline/proto/stage:camera_detection_preprocessor_config_cc_proto",
    ],
)

cpplint()
