load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
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
    deps = [
        "//cyber",
        "//cyber/time:clock",
        "//modules/common/math",
        "//modules/common/monitor_log",
        "//modules/common/proto:geometry_cc_proto",
        "//modules/common/util:message_util",
        "//modules/common/util:string_util",
        "//modules/drivers/gnss/proto:gnss_best_pose_cc_proto",
        "//modules/drivers/gnss/proto:gnss_cc_proto",
        "//modules/drivers/gnss/proto:ins_cc_proto",
        "//modules/localization/proto:gps_cc_proto",
        "//modules/localization/proto:imu_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/localization/proto:rtk_config_cc_proto",
        "//modules/transform:transform_broadcaster",
        "@com_google_googletest//:gtest",
        "@eigen",
    ],
)

cc_binary(
    name = "librtk_localization_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":rtk_localization_component_lib"],
)

cc_test(
    name = "rtk_localization_test",
    size = "small",
    srcs = ["rtk_localization_test.cc"],
    data = ["//modules/localization:localization_testdata"],
    deps = [
        "//cyber",
        "//cyber/time:clock",
        "//modules/common/util",
        "//modules/localization/rtk:rtk_localization_component_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
