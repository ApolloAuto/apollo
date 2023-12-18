load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "drivers-smartereye",
    includes = ["include"],
    hdrs = glob(["include/**/*.h"]),
    srcs = glob(["lib/*.so*"]),
    include_prefix = "modules/drivers/smartereye",
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)