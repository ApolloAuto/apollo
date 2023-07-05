load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "ncurses5",
    includes = [
        "include",
    ],
    linkopts = [
        "-lncurses",
    ],
    linkstatic = False,
    strip_include_prefix = "include",
)
