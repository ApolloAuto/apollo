load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "async_fusion",
    srcs = [
        "async_fusion.cc",
    ],
    hdrs = [
        "async_fusion.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/common",
        "//modules/perception/obstacle/fusion/interface",
        "//modules/perception/obstacle/fusion/probabilistic_fusion",
        "@eigen",
    ],
)

cpplint()
