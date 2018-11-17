load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "online_local_visualizer_lib",
    srcs = [
        "online_local_visualizer.cc",
    ],
    hdrs = [
        "online_local_visualizer.h",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
        "-lboost_program_options",
    ],
    deps = [
        "//modules/common:apollo_app",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor_log",
        "//modules/common/proto:common_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/localization/common:localization_common",
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "//modules/localization/msf/local_tool/local_visualization/engine",
        "//modules/localization/proto:localization_config_proto",
        "//modules/localization/proto:localization_proto",
        "@eigen//:eigen",
        "@pcl//:pcl",
        "@ros//:ros_common",
    ],
)

cc_binary(
    name = "online_local_visualizer",
    srcs = [
        "online_local_visualizer_main.cc",
    ],
    data = [
        "//modules/localization/conf:localization_adapter_manager_config",
    ],
    deps = [
        ":online_local_visualizer_lib",
    ],
)

cpplint()
