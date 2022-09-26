load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    runtime_dest = "localization/bin",
    targets = [
        ":offline_local_visualizer",
    ],
)

cc_library(
    name = "offline_local_visualizer_lib",
    srcs = ["offline_local_visualizer.cc"],
    hdrs = ["offline_local_visualizer.h"],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/localization/msf/common/io:common_io",
        "//modules/localization/msf/local_pyramid_map/base_map",
        "//modules/localization/msf/local_tool/local_visualization/engine",
        "@boost",
        "@com_google_absl//:absl",
    ],
)

cc_binary(
    name = "offline_local_visualizer",
    srcs = ["offline_local_visualizer_main.cc"],
    linkstatic = True,
    deps = [
        ":offline_local_visualizer_lib",
        "@boost",
    ],
)

cpplint()
