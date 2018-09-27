load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "radar_util",
    srcs = [
        "radar_util.cc",
    ],
    hdrs = [
        "radar_util.h",
    ],
    deps = [
        "@eigen",
        "//modules/perception/base:base",
    ],
)

cc_library(
    name = "types",
    hdrs = [
        "types.h",
    ],
    deps = [
    ],
)

cpplint()
