load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "fastdds",
    includes = [
        "include",
    ],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-lfastrtps",
        "-lfastcdr",
    ],
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)

