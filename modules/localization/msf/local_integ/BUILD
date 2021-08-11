load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "local_integ",
    srcs = glob(["*.cc"]),
    hdrs = glob(["*.h"]),
    deps = [
        "//modules/common/math",
        "//modules/common/status",
        "//modules/common/util:perf_util",
        "//modules/common/util:time_util",
        "//modules/drivers/gnss/proto:gnss_best_pose_cc_proto",
        "//modules/drivers/gnss/proto:gnss_cc_proto",
        "//modules/drivers/gnss/proto:gnss_raw_observation_cc_proto",
        "//modules/drivers/gnss/proto:heading_cc_proto",
        "//modules/drivers/gnss/proto:imu_cc_proto",
        "//modules/drivers/proto:pointcloud_cc_proto",
        "//modules/localization/common:localization_common",
        "//modules/localization/msf/common/util",
        "//modules/localization/msf/common/util:frame_transform",
        "//modules/localization/msf/local_pyramid_map/base_map",
        "//modules/localization/msf/local_pyramid_map/pyramid_map",
        "//modules/localization/msf/local_pyramid_map/pyramid_map:pyramid_map_pool",
        "//modules/localization/proto:gnss_pnt_result_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/localization/proto:localization_status_cc_proto",
        "//modules/localization/proto:measure_cc_proto",
        "//modules/localization/proto:pose_cc_proto",
        "//modules/localization/proto:sins_pva_cc_proto",
        "//third_party/localization_msf",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_github_google_glog//:glog",
        "@eigen",
    ],
)

cpplint()
