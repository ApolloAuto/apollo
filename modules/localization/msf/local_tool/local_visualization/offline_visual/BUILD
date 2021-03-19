load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "offline_local_visualizer_lib",
    srcs = ["offline_local_visualizer.cc"],
    hdrs = ["offline_local_visualizer.h"],
    deps = [
        "//cyber/common:log",
        "//modules/common/util",
        "//modules/localization/msf/common/io:common_io",
        "//modules/localization/msf/local_pyramid_map/base_map",
        "//modules/localization/msf/local_tool/local_visualization/engine",
        "@boost",
        "@com_google_absl//absl/strings",
    ],
)

cc_binary(
    name = "offline_local_visualizer",
    srcs = ["offline_local_visualizer_main.cc"],
    linkstatic = 0,
    deps = [
        ":offline_local_visualizer_lib",
        "@boost",
    ],
)

cpplint()
