load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "util",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    deps = [
        "@caffe//:lib",
        "@eigen//:eigen",
        "@opencv2//:core",
        "@undistort//:undistort",
        "//modules/common:log",
        "@opencv2//:imgproc",
        "@cuda",
    ]
)
cpplint()
