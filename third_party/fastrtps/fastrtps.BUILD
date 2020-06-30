load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "fastrtps",
    includes = [
        "include",
    ],
    linkopts = [
        "-L/usr/local/fast-rtps/lib",
        "-lfastcdr",
        "-lfastrtps",
    ],
    visibility = ["//visibility:public"],
)
