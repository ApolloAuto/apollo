load("//tools:cpplint.bzl", "cpplint")
package(default_visibility = ["//visibility:public"])


cc_binary(
    name = "obstacle_detection",
    srcs = [
        "obstacle_detection.cc",
    ],
    linkopts = [
        "-lopencv_core",
        "-lopencv_imgproc",
        "-lopencv_highgui",
    ],
    deps = [
        "@gtest",
        "//modules/perception/camera/lib/interface:interface",
        "//modules/perception/base:base",
        "//modules/perception/common/io:io_util",
    ]
)

cpplint()
