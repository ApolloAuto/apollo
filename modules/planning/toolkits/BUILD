load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "task",
    srcs = [
        "task.cc",
    ],
    hdrs = [
        "task.h",
    ],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/common/trajectory:publishable_trajectory",
        "//modules/planning/proto:planning_config_proto",
    ],
)

cpplint()
