load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "audio",
    includes = ["include"],
    hdrs = glob(["include/**/*.h"]),
    srcs = glob(["lib/**/*.so*"]),
    include_prefix = "modules/audio",
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)