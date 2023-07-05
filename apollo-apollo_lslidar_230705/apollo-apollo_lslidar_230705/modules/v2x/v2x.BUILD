load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "v2x",
    includes = ["include"],
    hdrs = glob(["include/**/*.h"]),
    srcs = glob(["lib/**/*.so*"]),
    include_prefix = "modules/v2x",
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)