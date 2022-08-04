load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "tinyxml2",
    includes = [
        "include",
    ],
    linkopts = [
        "-ltinyxml2",
    ],
    visibility = ["//visibility:public"],
    strip_include_prefix = "include",
)
