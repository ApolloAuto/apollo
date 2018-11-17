load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "offline_local_visualizer_lib",
    srcs = [
        "offline_local_visualizer.cc",
    ],
    hdrs = [
        "offline_local_visualizer.h",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
        "-lboost_program_options",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/util",
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "//modules/localization/msf/local_tool/local_visualization/engine",
        "@eigen//:eigen",
    ],
)

cc_binary(
    name = "offline_local_visualizer",
    srcs = [
        "offline_local_visualizer_main.cc",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
        "-lboost_program_options",
    ],
    linkstatic = 0,
    deps = [
        ":offline_local_visualizer_lib",
        "//modules/common:log",
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "@eigen//:eigen",
    ],
)

cpplint()
