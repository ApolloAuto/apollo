load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "avcodec",
    includes = ["."],
    hdrs = glob(["libavcodec/*.h"]),
    linkopts = [
        "-L/opt/apollo/sysroot/lib",
        "-lavcodec",
    ],
)

cc_library(
    name = "avformat",
    includes = ["."],
    hdrs = glob(["libavformat/*.h"]),
    linkopts = [
        "-L/opt/apollo/sysroot/lib",
        "-lavformat",
    ],
)

cc_library(
    name = "swscale",
    includes = ["."],
    hdrs = glob(["libswscale/*.h"]),
    linkopts = [
        "-L/opt/apollo/sysroot/lib",
        "-lswscale",
    ],
)

cc_library(
    name = "avutil",
    includes = ["."],
    hdrs = glob(["libavutil/*.h"]),
    linkopts = [
        "-L/opt/apollo/sysroot/lib",
        "-lavutil",
    ],
)
