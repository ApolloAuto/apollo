load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ndt_localization_component_lib",
    srcs = [
        "localization_pose_buffer.cc",
        "ndt_localization.cc",
        "ndt_localization_component.cc",
    ],
    hdrs = [
        "localization_pose_buffer.h",
        "ndt_localization.h",
        "ndt_localization_component.h",
    ],
    copts = ["-DMODULE_NAME=\\\"NDTlocalization\\\""],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/math",
        "//modules/common/monitor_log",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_cc_proto",
        "//modules/common_msgs/sensor_msgs:ins_cc_proto",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/localization/common:localization_gflags",
        "//modules/localization/msf/common/util",
        "//modules/localization/ndt/ndt_locator:ndt_lidar_locator",
        "//modules/common_msgs/localization_msgs:gps_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/transform:buffer",
        "//modules/transform:transform_broadcaster",
        "@boost",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_github_google_glog//:glog",
        "@eigen",
    ],
)

cc_binary(
    name = "libndt_localization_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":ndt_localization_component_lib"],
)

cc_test(
    name = "ndt_localization_test",
    size = "small",
    timeout = "short",
    srcs = ["ndt_localization_test.cc"],
    deps = [
        ":ndt_localization_component_lib",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_test(
    name = "ndt_localization_pose_buffer_test",
    size = "small",
    timeout = "short",
    srcs = ["localization_pose_buffer_test.cc"],
    deps = [
        ":ndt_localization_component_lib",
        "//cyber",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

install(
    name = "install",
    library_dest = "localization/lib/ndt",
    targets = [
        ":libndt_localization_component.so",
    ],
)

cpplint()
