load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "fftw3",
    includes = ["include"],
    hdrs = glob(["include/fftw3*"]),
    linkopts = [
        "-lfftw3",
        "-lm",
    ],
    strip_include_prefix = "include",
)

cc_library(
    name = "fftw3_omp",
    includes = ["include"],
    linkopts = [
        "-lgomp",
        "-pthread",
        "-ldl",
    ],
    deps = [
        ":fftw3",
    ],
    strip_include_prefix = "include",
)

cc_library(
    name = "fftw3_threads",
    includes = ["include"],
    linkopts = [
        "-pthread",
    ],
    deps = [
        ":fftw3",
    ],
    strip_include_prefix = "include",
)
