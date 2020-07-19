load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "fastrtps",
    includes = [
        ".",
    ],
    linkopts = [
        "-L/usr/local/fast-rtps/lib",
        "-lfastrtps",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@fastcdr",
    ],
)
