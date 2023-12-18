load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "benchmark",
    includes = [
        "include",
    ],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-lbenchmark",
        "-pthread",
    ],
    visibility = ["//visibility:public"],
    strip_include_prefix = "include",
)

cc_library(
    name = "benchmark_main",
    linkopts = [
        "-lbenchmark_main",
    ],
    visibility = ["//visibility:public"],
    deps = [":benchmark"],
)
