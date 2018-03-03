load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "util",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    deps = [
        "@caffe//:lib",
        "@eigen//:eigen",
        "//modules/common:log",
        "//modules/perception/obstacle/base:perception_obstacle_base"
    ]
)

cpplint()
