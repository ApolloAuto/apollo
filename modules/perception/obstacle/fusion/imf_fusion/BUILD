load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "imf_fusion",
    srcs = [
        "imf_fusion.cc",
        "async_track.cc",
        "async_fusion.cc",
    ],
    hdrs = [
        "imf_fusion.h",
        "async_track.h",
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
