load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "task-manager",
    includes = ["include"],
    hdrs = glob(["include/**/*.h"]),
    srcs = glob(["lib/**/*.so*"]),
    include_prefix = "modules/task_manager",
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)