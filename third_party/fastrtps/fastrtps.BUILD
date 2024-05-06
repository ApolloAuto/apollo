load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "fastrtps",
    includes = [
        ".",
    ],
    hdrs = glob(["**/*"]),
    linkopts = [
        "-L/usr/local/fast-rtps/lib",
        "-lfastrtps",
        "-lfastcdr",
    ],
    visibility = ["//visibility:public"],
)
