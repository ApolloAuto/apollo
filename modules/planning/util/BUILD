load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "planning_util",
    srcs = [
        "planning_util.cc",
    ],
    hdrs = [
        "planning_util.h",
    ],
    deps = [
        "//modules/common/proto:path_point_proto",
        "//modules/planning/proto:planning_proto",
    ],
)

cpplint()
