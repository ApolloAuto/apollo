load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_common",
    srcs = [
        "lane_common.cc",
    ],
    hdrs = [
        "lane_common.h",
    ],
    deps = [
        "//cybertron",
        "//modules/perception/base:base",
        "//modules/perception/camera/lib/lane/common:common_functions",
    ]
)

cc_binary(
    name = "lane_denseline_eval",
    srcs = [
        "lane_denseline_eval.cc",
    ],
    linkopts = [
        "-lopencv_core",
        "-lopencv_imgproc",
        "-lopencv_highgui",
    ],
    deps = [
        ":lane_common",
        "//external:gflags",
        "//cybertron",
        "//modules/perception/camera/lib/interface:interface",
        "//modules/perception/camera/lib/calibration_service/online_calibration_service:online_calibration_service",
        "//modules/perception/camera/lib/lane/detector/denseline:denseline_lane_detector",
        "//modules/perception/camera/lib/lane/postprocessor/denseline:denseline_lane_postprocessor",
        "//modules/perception/base:base",
        "//modules/perception/camera/common:common",
        "//modules/perception/common/io:io_util",
        "//modules/perception/lib/io:file_util",
        "//modules/perception/lib/io:protobuf_util",
    ]
)

cpplint()
