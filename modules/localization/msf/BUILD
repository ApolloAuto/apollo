load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "msf_localization_component_lib",
    srcs = [
        "msf_localization.cc",
        "msf_localization_component.cc",
    ],
    hdrs = [
        "msf_localization.h",
        "msf_localization_component.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"localization\\\"",
    ],
    deps = [
        "//cyber",
        "//cyber/time:clock",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/math",
        "//modules/common/monitor_log",
        "//modules/common/proto:geometry_cc_proto",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/drivers/gnss/proto:config_cc_proto",
        "//modules/drivers/gnss/proto:gnss_best_pose_cc_proto",
        "//modules/drivers/gnss/proto:gnss_cc_proto",
        "//modules/drivers/gnss/proto:heading_cc_proto",
        "//modules/drivers/gnss/proto:imu_cc_proto",
        "//modules/localization/common:localization_common",
        "//modules/localization/msf/common/util",
        "//modules/localization/msf/local_integ",
        "//modules/localization/msf/local_pyramid_map/base_map",
        "//modules/localization/msf/local_pyramid_map/pyramid_map",
        "//modules/localization/proto:gnss_pnt_result_cc_proto",
        "//modules/localization/proto:gps_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/localization/proto:localization_config_cc_proto",
        "//modules/localization/proto:localization_status_cc_proto",
        "//modules/localization/proto:measure_cc_proto",
        "//modules/localization/proto:sins_pva_cc_proto",
        "//modules/transform:transform_broadcaster",
        "@boost",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
    ],
)

cc_binary(
    name = "libmsf_localization_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":msf_localization_component_lib"],
)

cc_test(
    name = "msf_localization_test",
    size = "enormous",
    srcs = ["msf_localization_test.cc"],
    copts = [
        "-fno-access-control",
    ],
    data = ["//modules/localization:localization_testdata"],
    deps = [
        "//cyber/time:clock",
        "//modules/common/util",
        "//modules/localization/msf:msf_localization_component_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
