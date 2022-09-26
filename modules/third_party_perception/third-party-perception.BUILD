load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "third_party_perception",
    includes = ["include"],
    hdrs = glob(["include/**/*.h"]),
    srcs = glob(["lib/**/*.so*"]),
    include_prefix = "modules/third_party_perception",
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)