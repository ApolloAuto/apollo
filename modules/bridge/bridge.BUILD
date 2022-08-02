load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "bridge",
    includes = ["include"],
    hdrs = glob(["include/**/*"]),
    srcs = glob(["lib/**/*.so*"]),
    include_prefix = "modules/bridge",
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)