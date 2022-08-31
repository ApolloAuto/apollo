load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "portaudio",
    hdrs = glob(["include/**/*"]),
    strip_include_prefix = "include",
    linkopts = [
        "-lrt",
        "-lasound",
        "-ljack",
        "-lpthread",
        "-lportaudio",
    ],
)
