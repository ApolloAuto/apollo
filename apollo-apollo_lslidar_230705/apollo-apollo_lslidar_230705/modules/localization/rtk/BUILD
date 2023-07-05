load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rtk_localization_component_lib",
    srcs = [
        "rtk_localization.cc",
        "rtk_localization_component.cc",
    ],
    hdrs = [
        "rtk_localization.h",
        "rtk_localization_component.h",
    ],
    copts = ["-DMODULE_NAME=\\\"localization\\\""],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/common/monitor_log",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_cc_proto",
        "//modules/common_msgs/sensor_msgs:ins_cc_proto",
        "//modules/common_msgs/localization_msgs:gps_cc_proto",
        "//modules/common_msgs/localization_msgs:imu_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/localization/proto:rtk_config_cc_proto",
        "//modules/transform:transform_broadcaster",
        "@com_google_googletest//:gtest",
        "@eigen",
    ],
)

cc_binary(
    name = "librtk_localization_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":rtk_localization_component_lib"],
)

cc_test(
    name = "rtk_localization_test",
    size = "small",
    srcs = ["rtk_localization_test.cc"],
    data = ["//modules/localization:localization_testdata"],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/localization/rtk:rtk_localization_component_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

install(
    name = "install",
    library_dest = "localization/lib/rtk",
    targets = [
        ":librtk_localization_component.so",
    ],
)

cpplint()
