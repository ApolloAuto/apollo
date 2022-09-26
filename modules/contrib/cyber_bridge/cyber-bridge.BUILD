load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "cyber_bridge",
    includes = ["include"],
    hdrs = glob(["include/**/*"]),
    srcs = glob(["lib/**/*.so*"]),
    include_prefix = "modules/cyber_bridge",
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)