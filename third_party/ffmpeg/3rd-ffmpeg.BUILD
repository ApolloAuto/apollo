load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "avcodec",
    hdrs = glob(["include/libavcodec/*.h"]),
    strip_include_prefix = "include",
    linkopts = [
        "-L/opt/apollo/sysroot/lib",
        "-lavcodec",
    ],
)

cc_library(
    name = "avformat",
    hdrs = glob(["include/libavformat/*.h"]),
    strip_include_prefix = "include",
    linkopts = [
        "-L/opt/apollo/sysroot/lib",
        "-lavformat",
    ],
)

cc_library(
    name = "swscale",
    hdrs = glob(["include/libswscale/*.h"]),
    strip_include_prefix = "include",
    linkopts = [
        "-L/opt/apollo/sysroot/lib",
        "-lswscale",
    ],
)

cc_library(
    name = "avutil",
    hdrs = glob(["include/libavutil/*.h"]),
    strip_include_prefix = "include",
    linkopts = [
        "-L/opt/apollo/sysroot/lib",
        "-lavutil",
    ],
)
