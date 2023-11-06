load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "opengl",
    includes = [
        ".",
    ],
    hdrs = glob(["**/*"]),
    linkopts = [
        "-lGL",
        # "-lGLU",
        # "-lSM",
        # "-lICE",
        # "-lXext",
    ],
    linkstatic = False,
)
