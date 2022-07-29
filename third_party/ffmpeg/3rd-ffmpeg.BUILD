load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "avcodec",
    includes = ["include"],
    linkopts = [
        "-lavcodec",
    ],
    strip_include_prefix = "include",

)

cc_library(
    name = "avformat",
    includes = ["include"],
    linkopts = [
        "-lavformat",
    ],
    strip_include_prefix = "include",

)

cc_library(
    name = "swscale",
    includes = ["include"],
    linkopts = [
        "-lswscale",
    ],
    strip_include_prefix = "include",

)

cc_library(
    name = "avutil",
    includes = ["include"],
    linkopts = [
        "-lavutil",
    ],
    strip_include_prefix = "include",
)
