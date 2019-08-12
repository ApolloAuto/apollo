load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "benchmark_eval",
    srcs = [
        "position_metric.cc",
        "meta_statistics.cc",
        "sequence_self_statistics.cc",
        "frame_statistics.cc",
        "lidar_option.cc",
    ],
    hdrs = [
        "position_metric.h",
        "meta_statistics.h",
        "sequence_self_statistics.h",
        "frame_statistics.h",
        "lidar_option.h",
    ],
    deps = [
        "//cyber",
        "//modules/perception/tool/benchmark/lidar/util:benchmark_util",
        "//modules/perception/tool/benchmark/lidar/base:benchmark_base",
    ],
)

cpplint()
