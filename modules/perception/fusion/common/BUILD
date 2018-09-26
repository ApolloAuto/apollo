load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_filter",
    hdrs = [
        "base_filter.h",
    ],
)

cc_library(
    name = "camera_util",
    srcs = [
        "camera_util.cc",
    ],
    hdrs = [
        "camera_util.h",
    ],
    deps = [
        "//modules/perception/base:camera",
        "//modules/perception/base:object",
        "//modules/perception/common/geometry:common",
        "//modules/perception/fusion/base:sensor",
    ],
)

cpplint()
