load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "control_task_base",
    includes = ["include"],
    hdrs = glob(["include/**/*.h"]),
    srcs = glob(["lib/**/*.so*"]),
    include_prefix = "modules/control/controllers/control_task_base",
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)