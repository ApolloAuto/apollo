load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "avcodec",
    includes = ["."],
    linkopts = [
        "-L/opt/apollo/sysroot/lib",
        "-lavcodec",
    ],
)

cc_library(
    name = "avformat",
    includes = ["."],
    linkopts = [
        "-L/opt/apollo/sysroot/lib",
        "-lavformat",
    ],
)

cc_library(
    name = "swscale",
    includes = ["."],
    linkopts = [
        "-L/opt/apollo/sysroot/lib",
        "-lswscale",
    ],
)

cc_library(
    name = "avutil",
    includes = ["."],
    linkopts = [
        "-L/opt/apollo/sysroot/lib",
        "-lavutil",
    ],
)
