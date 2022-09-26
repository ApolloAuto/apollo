load("@rules_cc//cc:defs.bzl", "cc_library")
  
cc_library(
    name = "common-msgs",
    includes = ["include"],
    hdrs = glob(
        [
            "include/**/*.h",
        ],
    ),
    include_prefix = "modules/common_msgs",
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)