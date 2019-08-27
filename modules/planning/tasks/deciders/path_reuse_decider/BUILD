load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "path_reuse_decider",
    srcs = [
        "path_reuse_decider.cc",
    ],
    hdrs = [
        "path_reuse_decider.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/tasks/deciders:decider_base",
    ],
)

cpplint()
