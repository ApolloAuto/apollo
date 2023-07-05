load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "drivers",
    includes = ["include"],
    hdrs = glob(["include/**/*.h"]),
    srcs = glob(["lib/*.so*"], exclude=["lib/libsmartereye_component.so"]),
    include_prefix = "modules/drivers",
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)