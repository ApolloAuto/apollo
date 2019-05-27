load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "path_decider_obstacle_utils",
    srcs = [
        "path_decider_obstacle_utils.cc",
    ],
    hdrs = [
        "path_decider_obstacle_utils.h",
    ],
    deps = [
        "//modules/planning/common:frame",
    ],
)

cc_library(
    name = "decider_base",
    srcs = [
        "decider.cc",
    ],
    hdrs = [
        "decider.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/tasks:task",
    ],
)

cpplint()
