load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "control",
    includes = ["include"],
    hdrs = glob(["include/**/*.h"]),
    srcs = glob(["lib/**/lib*.so*"]),
    include_prefix = "modules/control",
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)
