load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
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
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/math",
        "//modules/common/monitor_log",
        "//modules/common/proto:geometry_cc_proto",
        "//modules/common/util:message_util",
        "//modules/common/util:perf_util",
        "//modules/drivers/gnss/proto:gnss_best_pose_cc_proto",
        "//modules/drivers/gnss/proto:gnss_cc_proto",
        "//modules/drivers/gnss/proto:ins_cc_proto",
        "//modules/drivers/proto:pointcloud_cc_proto",
        "//modules/localization/common:localization_common",
        "//modules/localization/msf/common/util",
        "//modules/localization/ndt/ndt_locator:ndt_lidar_locator",
        "//modules/localization/proto:gps_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
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
    linkstatic = False,
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

cpplint()
