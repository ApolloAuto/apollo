load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "decider",
    srcs = [
        "decider.cc",
    ],
    hdrs = [
        "decider.h",
    ],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/toolkits:task",
    ],
)

cc_library(
    name = "creep",
    srcs = [
        "creep.cc",
    ],
    hdrs = [
        "creep.h",
    ],
    deps = [
        "//modules/common/util:map_util",
        "//modules/planning/common:frame",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/toolkits/deciders:decider",

    ],
)

cpplint()
