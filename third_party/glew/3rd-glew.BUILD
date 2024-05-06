load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "glew",
    includes = ["include"],
    linkopts = [
        "-lGLEW",
    ],
    strip_include_prefix = "include",
)
