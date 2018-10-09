load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rtk_localization_component_lib",
    srcs = [
        "rtk_localization_component.cc",
        "rtk_localization.cc",
    ],
    hdrs = [
        "rtk_localization_component.h",
        "rtk_localization.h",
    ],
    copts = ["-DMODULE_NAME=\\\"localization\\\""],
    deps = [
        "//modules/common/monitor_log",
        "//modules/common/time",
        "//modules/common/math",
        "//modules/common/util:message_util",
        "//modules/common/proto:geometry_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/localization/proto:rtk_config_proto",
        "//modules/localization/proto:gps_proto",
        "//modules/localization/proto:imu_proto",
        "//modules/transform:transform_broadcaster_lib",
        "//framework:cybertron",
        "@eigen",
        "@gtest",
    ],
)

cc_binary(
    name = "librtk_localization_component.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":rtk_localization_component_lib"],
)

cc_test(
    name = "rtk_localization_test",
    size = "small",
    srcs = [
        "rtk_localization_test.cc",
    ],
    data = ["//modules/localization:localization_testdata"],
    deps = [
        "//framework:cybertron",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/localization/rtk:rtk_localization_component_lib",
        "@gtest//:main",
    ],
)

cpplint()
