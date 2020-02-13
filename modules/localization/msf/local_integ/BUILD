load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_msf_local_integ",
    srcs = glob(["*.cc"]),
    hdrs = glob(["*.h"]),
    copts = [
        "-O2",
        "-DMODULE_NAME=\\\"localization\\\"",
    ],
    linkopts = [
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgproc",
    ],
    deps = [
        "//modules/common/status",
        "//modules/common/time",
        "//modules/drivers/gnss/proto:gnss_best_pose_proto",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//modules/drivers/gnss/proto:gnss_raw_observation_proto",
        "//modules/drivers/gnss/proto:imu_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/localization/common:localization_common",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/common/util:localization_msf_common_util_frame_transform",
        "//modules/localization/msf/local_pyramid_map/base_map:localization_pyramid_map_base_map",
        "//modules/localization/msf/local_pyramid_map/pyramid_map:localization_pyramid_map_pyramid_map",
        "//modules/localization/msf/local_pyramid_map/pyramid_map:localization_pyramid_map_pyramid_map_pool",
        "//modules/localization/proto:gnss_pnt_result_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/localization/proto:localization_status_proto",
        "//modules/localization/proto:measure_proto",
        "//modules/localization/proto:pose_proto",
        "//modules/localization/proto:sins_pva_proto",
        "//third_party:boost",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_google_glog//:glog",
        "@eigen",
        "@local_integ",
    ],
)

cpplint()
