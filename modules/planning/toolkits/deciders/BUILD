load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "decider",
    srcs = [
        "decider.cc",
        "decider_creep.cc",
        "decider_stop_sign_monitor.cc",
    ],
    hdrs = [
        "decider.h",
        "decider_creep.h",
        "decider_stop_sign_monitor.h",
    ],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/toolkits:task",
    ],
)

cpplint()
