load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "creep_decider",
    srcs = [
        "creep_decider.cc",
    ],
    hdrs = [
        "creep_decider.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:planning_context",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/scenarios/util:scenario_util_lib",
        "//modules/planning/tasks/deciders:decider_base",
    ],
)

cpplint()
