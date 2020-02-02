load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "obstacle_detection",
    srcs = ["obstacle_detection.cc"],
    deps = [
        "//modules/perception/base",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/common/io:io_util",
        "//third_party:boost",
        "@com_google_googletest//:gtest",
        "@opencv",
    ],
)

cpplint()
