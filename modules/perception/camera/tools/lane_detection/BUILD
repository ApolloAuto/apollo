load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_common",
    srcs = ["lane_common.cc"],
    hdrs = ["lane_common.h"],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/lib/lane/common:common_functions",
    ],
)

cc_binary(
    name = "lane_denseline_eval",
    srcs = ["lane_denseline_eval.cc"],
    linkopts = [
        "-lopencv_core",
        "-lopencv_imgproc",
        "-lopencv_highgui",
    ],
    linkstatic = False,
    deps = [
        ":lane_common",
        "//cyber",
        "//modules/common/util:string_util",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/calibration_service/online_calibration_service",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/lane/detector/denseline:denseline_lane_detector",
        "//modules/perception/camera/lib/lane/postprocessor/denseline:denseline_lane_postprocessor",
        "//modules/perception/common/io:io_util",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
