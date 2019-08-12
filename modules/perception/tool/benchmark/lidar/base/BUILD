load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "benchmark_base",
    srcs = [
        "frame.cc",
        "point_cloud_frame.cc",
    ],
    hdrs = [
        "frame.h",
        "point_cloud_frame.h",
        "sequence_maintainer.h",
    ],
    deps = [
        "//cyber",
        "//modules/perception/tool/benchmark/lidar/util:benchmark_util",
    ],
)

cpplint()
