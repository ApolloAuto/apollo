load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "glfw",
    includes = ["."],
    linkopts = [
        "-lglfw",
        "-lglapi",
        "-lglib-2.0",
    ],
)
