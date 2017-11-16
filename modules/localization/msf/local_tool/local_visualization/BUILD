load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "local_visualizer",
    srcs = [
        "engine/visualization_engine.cc",
        "offline_local_visualizer.cc"
    ],
    hdrs = [
        "engine/visualization_engine.h",
        "offline_local_visualizer.h"
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
        "-lboost_program_options"
    ],
    deps = [
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "@eigen//:eigen",
        "@glog//:glog"
    ],
)

cc_binary(
    name = "offline_local_visualizer",
    srcs = [
        "offline_local_visualizer_main.cc"
    ],
    linkstatic = 0,
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
        "-lboost_program_options"
    ],
    deps = [
        ":local_visualizer",
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "@eigen//:eigen",
        "@glog//:glog"
    ],
)

cpplint()
