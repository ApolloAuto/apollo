load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "tf2",
    hdrs = glob([
        "include/geometry_msgs/**",
        "include/tf2_msgs/**",
        "include/tf2/**",
    ]),
    srcs = glob(["lib/*.so"]),
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)