load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_cc_binary", "apollo_cc_test")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "online_visualizer_component_lib",
    srcs = ["online_visualizer_component.cc"],
    hdrs = ["online_visualizer_component.h"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/math",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common/status",
        "//modules/common/util:common_util",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/localization/common:localization_gflags",
        "//modules/localization/msf:apollo_localization_msf",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/localization/proto:localization_config_cc_proto",
        "@boost",
        "@com_google_googletest//:gtest_main",
        "@local_config_pcl//:pcl",
    ],
)

apollo_cc_binary(
    name = "online_visualizer_compenont.so",
    linkshared = True,
    linkstatic = True,
    deps = [":online_visualizer_component_lib"],
)

apollo_package()
cpplint()
