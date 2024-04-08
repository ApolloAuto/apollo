load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_component", "apollo_cc_test")

package(default_visibility = ["//visibility:public"])

apollo_component(
    name = "libndt_localization_component.so",
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
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_cc_proto",
        "//modules/common_msgs/sensor_msgs:ins_cc_proto",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/localization/common:localization_gflags",
        "//modules/localization/msf:apollo_localization_msf",
        "//modules/localization/ndt/ndt_locator:ndt_lidar_locator",
        "//modules/common_msgs/localization_msgs:gps_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/transform:apollo_transform",
        "@boost",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_github_google_glog//:glog",
        "@eigen",
    ],
)

apollo_cc_test(
    name = "ndt_localization_test",
    size = "small",
    timeout = "short",
    srcs = ["ndt_localization_test.cc"],
    deps = [
        ":DO_NOT_IMPORT_ndt_localization_component",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "ndt_localization_pose_buffer_test",
    size = "small",
    timeout = "short",
    srcs = ["localization_pose_buffer_test.cc"],
    deps = [
        ":DO_NOT_IMPORT_ndt_localization_component",
        "//cyber",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

apollo_package()
cpplint()
