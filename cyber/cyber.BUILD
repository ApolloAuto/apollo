load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "cyber",
    includes = ["include"],
    hdrs = glob(["include/**/*"]),
    srcs = glob(["lib/**/lib*.so*"]),
    include_prefix = "cyber",
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)