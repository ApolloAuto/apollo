load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "camera_detection_postprocessor",
    srcs = ["camera_detection_postprocessor.cc"],
    hdrs = ["camera_detection_postprocessor.h"],
    deps = [
        "//modules/perception/camera/lib/obstacle/camera_detection_postprocessor/camera_get_object",
        "//modules/perception/camera/lib/obstacle/camera_detection_postprocessor/filter_bbox",
        "//modules/perception/camera/lib/obstacle/camera_detection_postprocessor/recover_bbox",
        "//modules/perception/pipeline:stage",
        "//modules/perception/pipeline/proto/stage:camera_detection_postprocessor_config_cc_proto",
    ],
)

cpplint()
