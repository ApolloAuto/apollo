load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "scenario_util_lib",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//cyber/common:log",
        "//modules/common/util:map_util",
        "//modules/planning/common:planning_common",
    ],
)

cpplint()
