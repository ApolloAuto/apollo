load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "calibration",
    includes = ["include"],
    hdrs = glob(["include/**/*"]),
    srcs = glob(["lib/**/*.so*"]),
    include_prefix = "modules/calibration",
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)