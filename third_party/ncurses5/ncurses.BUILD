load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "ncurses5",
    includes = [
        ".",
    ],
    linkopts = [
        "-lncurses",
    ],
    hdrs = glob(["**/*"]),
    linkstatic = False,
)
