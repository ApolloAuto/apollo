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
        "//modules/common/util:util_tool",
        "//modules/common_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_raw_observation_cc_proto",
        "//modules/common_msgs/sensor_msgs:heading_cc_proto",
        "//modules/common_msgs/sensor_msgs:imu_cc_proto",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/localization/common:localization_gflags",
        "//modules/localization/msf/common/util",
        "//modules/localization/msf/common/util:frame_transform",
        "//modules/localization/msf/local_pyramid_map/base_map",
        "//modules/localization/msf/local_pyramid_map/pyramid_map",
        "//modules/localization/msf/local_pyramid_map/pyramid_map:pyramid_map_pool",
        "//modules/localization/proto:gnss_pnt_result_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_status_cc_proto",
        "//modules/localization/proto:measure_cc_proto",
        "//modules/common_msgs/localization_msgs:pose_cc_proto",
        "//modules/localization/proto:sins_pva_cc_proto",
        "//third_party/localization_msf",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_github_google_glog//:glog",
        "@eigen",
    ],
)

cpplint()
