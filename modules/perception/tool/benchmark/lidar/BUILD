load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "benchmark_evaluation",
    srcs = ["detection_evaluation.cc"],
    hdrs = ["detection_evaluation.h"],
    deps = [
        "//modules/perception/tool/benchmark/lidar/eval:benchmark_eval",
        "//modules/perception/tool/benchmark/lidar/loader:benchmark_loader",
    ],
)

cc_binary(
    name = "lidar_benchmark",
    srcs = ["lidar_detection_benchmark.cc"],
    linkopts = [
        "-llz4",
    ],
    deps = [
        "//modules/perception/tool/benchmark/lidar:benchmark_evaluation",
        "//third_party:boost",
    ],
)

cpplint()
