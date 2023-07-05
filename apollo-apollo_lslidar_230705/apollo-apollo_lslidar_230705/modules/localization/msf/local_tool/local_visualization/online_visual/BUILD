load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "online_visualizer_component_lib",
    srcs = ["online_visualizer_component.cc"],
    hdrs = ["online_visualizer_component.h"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/math",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/localization/common:localization_gflags",
        "//modules/localization/msf/common/io:common_io",
        "//modules/localization/msf/local_pyramid_map/base_map",
        "//modules/localization/msf/local_tool/local_visualization/engine",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/localization/proto:localization_config_cc_proto",
        "@boost",
        "@com_google_googletest//:gtest_main",
        "@local_config_pcl//:pcl",
    ],
    alwayslink = True,
)

cc_binary(
    name = "online_visualizer_compenont.so",
    linkshared = True,
    linkstatic = True,
    deps = [":online_visualizer_component_lib"],
)

install(
    name = "install",
    library_dest = "localization/lib/msf/local_tool/local_visualization/online_visual",
    targets = [
        ":online_visualizer_compenont.so",
    ],
)

cpplint()
