load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "offline_local_visualizer_lib",
    srcs = [
        "engine/visualization_engine.cc",
        "engine/visualization_manager.cc",
        "offline_visual/offline_local_visualizer.cc"
    ],
    hdrs = [
        "engine/visualization_engine.h",
        "engine/visualization_manager.h",
        "offline_visual/offline_local_visualizer.h"
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
        "//modules/common:log",
    ],
)

cc_binary(
    name = "offline_local_visualizer",
    srcs = [
        "offline_visual/offline_local_visualizer_main.cc"
    ],
    linkstatic = 0,
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
        "-lboost_program_options"
    ],
    deps = [
        ":offline_local_visualizer_lib",
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "@eigen//:eigen",
        "//modules/common:log",
    ],
)

cc_library(
    name = "online_local_visualizer_lib",
    srcs = [
        "engine/visualization_engine.cc",
        "engine/visualization_manager.cc",
        "online_visual/online_local_visualizer.cc"
    ],
    hdrs = [
        "engine/visualization_engine.h",
        "engine/visualization_manager.h",
        "online_visual/online_local_visualizer.h"
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
        "-lboost_program_options"
    ],
    deps = [
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor",
        "//modules/common:apollo_app",
        "//modules/common/proto:common_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common:log",
        "//modules/localization/common:localization_common",
        "//modules/localization/proto:localization_config_proto",
        "//modules/localization/proto:localization_proto",
        "@eigen//:eigen",
        "@ros//:ros_common",
        "@pcl//:pcl"
    ],
)

cc_binary(
    name = "online_local_visualizer",
    srcs = [
        "online_visual/online_local_visualizer_main.cc"
    ],
    data = [
        "//modules/localization/conf:localization_adapter_manager_config",
    ],
    deps = [
        ":online_local_visualizer_lib"
    ],
)

cpplint()
