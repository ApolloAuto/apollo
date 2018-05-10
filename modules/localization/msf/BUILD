load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "msf_localization",
    srcs = [
        "msf_localization.cc",
    ],
    hdrs = [
        "msf_localization.h",
    ],
    linkopts = [
        "-lboost_system",
    ],
    deps = [
        "//modules/common",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor_log",
        "//modules/common/proto:common_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/drivers/gnss/proto:config_proto",
        "//modules/drivers/gnss/proto:gnss_best_pose_proto",
        "//modules/drivers/gnss/proto:imu_proto",
        "//modules/localization:localization_base",
        "//modules/localization/common:localization_common",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "//modules/localization/msf/local_map/lossy_map:localization_msf_lossy_map",
        "//modules/localization/msf/local_integ:localization_msf_local_integ",
        "//modules/localization/proto:gnss_pnt_result_proto",
        "//modules/localization/proto:localization_config_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/localization/proto:measure_proto",
        "//modules/localization/proto:sins_pva_proto",
        "@glog",
        "@gtest",
        "@ros//:ros_common",
        "@yaml_cpp//:yaml",
    ],
)

cc_test(
    name = "msf_localization_test",
    size = "small",
    srcs = [
        "msf_localization_test.cc",
    ],
    data = ["//modules/localization:localization_testdata"],
    deps = [
        ":msf_localization",
        "//modules/common:log",
        "//modules/common/time",
        "//modules/common/util",
        "@gtest//:main",
    ],
)

cpplint()
