load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

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
    ],
)

cpplint()
