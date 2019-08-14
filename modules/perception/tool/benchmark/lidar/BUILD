load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "benchmark_evaluation",
    hdrs = [
        "detection_evaluation.h",
    ],
    srcs = [
        "detection_evaluation.cc",
    ],
    deps = [
        "//modules/perception/tool/benchmark/lidar/eval:benchmark_eval",
        "//modules/perception/tool/benchmark/lidar/loader:benchmark_loader",
    ],
)

cc_binary(
    name = "lidar_benchmark",
    srcs = ["lidar_detection_benchmark.cc"],
    deps = [
        "//modules/perception/tool/benchmark/lidar:benchmark_evaluation",
        "@boost",
    ],
    linkopts = [
        "-llz4",
    ],
)

cpplint()
