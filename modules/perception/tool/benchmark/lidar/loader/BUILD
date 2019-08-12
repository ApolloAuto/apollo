load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "benchmark_loader",
    hdrs = [
        "async_sequence_data_loader.h",
        "sequence_data_loader.h",
        "string_compare.h",
    ],
    deps = [
        "//modules/perception/tool/benchmark/lidar/ctpl:benchmark_ctpl",
    ],
)

cpplint()
