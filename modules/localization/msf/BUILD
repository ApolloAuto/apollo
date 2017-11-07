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
        "//modules/common/monitor",
        "//modules/common/proto:common_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/localization:localization_base",
        "//modules/localization/common:localization_common",
        "//modules/localization/proto:localization_config_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/localization/proto:measure_proto",
        "//modules/localization/proto:sins_pva_proto",
        "//modules/drivers/gnss/proto:gnss_best_pose_proto",
        "//modules/localization/msf/local_integ:localization_msf_local_integ",
        "@gtest//:gtest",
        "@glog//:glog",
        "@ros//:ros_common",
    ],
)

cpplint()
