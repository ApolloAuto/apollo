load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "tf2",
    includes = [
        ".",
    ],
    linkopts = [
        "-L/opt/apollo/pkgs/tf2/lib",
        "-ltf2",
    ],
)
