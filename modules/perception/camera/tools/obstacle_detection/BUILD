load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "obstacle_detection",
    srcs = [
        "obstacle_detection.cc",
    ],
    linkopts = [
        "-lboost_system",
        "-lopencv_core",
        "-lopencv_imgproc",
        "-lopencv_highgui",
    ],
    linkstatic = False,
    deps = [
        "//modules/perception/base",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/common/io:io_util",
        "@gtest",
    ],
)

cpplint()
