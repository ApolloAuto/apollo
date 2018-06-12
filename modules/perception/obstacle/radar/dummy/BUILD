load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dummy",
    srcs = [
        "dummy_algorithms.cc",
    ],
    hdrs = [
        "dummy_algorithms.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/perception/obstacle/common",
        "//modules/perception/obstacle/radar/interface",
        "@eigen",
        "@pcl",
    ],
)

cpplint()
