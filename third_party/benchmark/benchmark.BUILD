load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "benchmark",
    includes = [
        ".",
    ],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-L/opt/apollo/sysroot/lib",
        "-lbenchmark",
        "-pthread",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "benchmark_main",
    linkopts = [
        "-L/opt/apollo/sysroot/lib",
        "-lbenchmark_main",
    ],
    visibility = ["//visibility:public"],
    deps = [":benchmark"],
)
