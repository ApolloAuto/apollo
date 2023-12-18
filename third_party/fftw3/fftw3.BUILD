load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "fftw3",
    includes = ["."],
    hdrs = glob(["**/*"]),
    linkopts = [
        "-lfftw3",
        "-lm",
    ],
)

cc_library(
    name = "fftw3_omp",
    includes = ["."],
    hdrs = glob(["**/*"]),
    linkopts = [
        "-lgomp",
        "-pthread",
        "-ldl",
    ],
    deps = [
        ":fftw3",
    ],
)

cc_library(
    name = "fftw3_threads",
    includes = ["."],
    hdrs = glob(["**/*"]),
    linkopts = [
        "-pthread",
    ],
    deps = [
        ":fftw3",
    ],
)

# cc_library(
#     name = "fftw3l",
#     includes = ["."],
#     linkopts = [
#         "-lfftw3l",
#         "-lm",
#     ],
# )
#
# cc_library(
#     name = "fftw3l_omp",
#     includes = ["."],
#     linkopts = [
#         "-lgomp",
#         "-pthread",
#         "-ldl",
#     ],
#     deps = [
#         ":fftw3l",
#     ],
# )
#
# cc_library(
#     name = "fftw3l_threads",
#     includes = ["."],
#     linkopts = [
#         "-pthread",
#     ],
#     deps = [
#         ":fftw3l",
#     ],
# )
#
# cc_library(
#     name = "fftw3f",
#     includes = ["."],
#     linkopts = [
#         "-lfftw3f",
#         "-lm",
#     ],
# )
#
# cc_library(
#     name = "fftw3f_omp",
#     includes = ["."],
#     linkopts = [
#         "-lgomp",
#         "-pthread",
#         "-ldl",
#     ],
#     deps = [
#         ":fftw3f",
#     ],
# )
#
# cc_library(
#     name = "fftw3f_threads",
#     includes = ["."],
#     linkopts = [
#         "-pthread",
#     ],
#     deps = [
#         ":fftw3f",
#     ],
# )
#
# cc_library(
#     name = "fftw3q",
#     includes = ["."],
#     linkopts = [
#         "-lfftw3q",
#         "-lm",
#     ],
# )
#
# cc_library(
#     name = "fftw3q_omp",
#     includes = ["."],
#     linkopts = [
#         "-lgomp",
#         "-pthread",
#         "-ldl",
#     ],
#     deps = [
#         ":fftw3q",
#     ],
# )
#
# cc_library(
#     name = "fftw3q_threads",
#     includes = ["."],
#     linkopts = [
#         "-pthread",
#     ],
#     deps = [
#         ":fftw3q",
#     ],
# )
