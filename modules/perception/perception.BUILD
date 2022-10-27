load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "perception",
    includes = ["include"],
    hdrs = glob(["include/**/*.h"]),
    srcs = glob(["lib/*.so*"], exclude=["lib/libperception_component_lidar.so"]),
    include_prefix = "modules/perception",
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)
