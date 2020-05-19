load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "online_visualizer_compenont_lib",
    srcs = ["online_visualizer_component.cc"],
    hdrs = ["online_visualizer_component.h"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/proto:geometry_cc_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/math",
        "//modules/drivers/proto:pointcloud_cc_proto",
        "//modules/localization/common:localization_common",
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map",
        "//modules/localization/msf/local_tool/local_visualization/engine",
        "//modules/localization/proto:localization_config_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//third_party:boost",
        "@eigen",
        "@pcl",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "online_visualizer_compenont.so",
    deps = [":online_visualizer_compenont_lib"],
)

cpplint()
