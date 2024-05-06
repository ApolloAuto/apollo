load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_component", "apollo_cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_component(
    name = "librtk_localization_component.so",
    srcs = [
        "rtk_localization.cc",
        "rtk_localization_component.cc",
    ],
    hdrs = [
        "rtk_localization.h",
        "rtk_localization_component.h",
    ],
    copts = ["-DMODULE_NAME=\\\"localization\\\""],
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/common/monitor_log",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/transform_msgs:transform_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_cc_proto",
        "//modules/common_msgs/sensor_msgs:ins_cc_proto",
        "//modules/common_msgs/localization_msgs:gps_cc_proto",
        "//modules/common_msgs/localization_msgs:imu_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/localization/proto:rtk_config_cc_proto",
        "//modules/transform:apollo_transform",
        "@com_google_googletest//:gtest",
        "@eigen",
    ],
)

apollo_cc_test(
    name = "rtk_localization_test",
    size = "small",
    srcs = ["rtk_localization_test.cc"],
    data = ["//modules/localization:localization_testdata"],
    deps = [
        "//cyber",
        "//modules/common/util:common_util",
        "//modules/localization/rtk:DO_NOT_IMPORT_rtk_localization_component",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()
cpplint()
