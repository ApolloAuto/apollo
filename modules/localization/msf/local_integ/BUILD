load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_msf_local_integ",
    srcs = ["liblocalization_msf_local_integ.so"],
    hdrs = ["localization_integ.h"],
    defines = ["VIS_USE_OPENCV"],
    copts = ["-fopenmp"],
    linkopts = [
        "-lboost_system",
        "-lboost_thread",
        "-lboost_program_options",
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgproc",
        "-lgomp",
    ],
    deps = [
        "//external:gflags",
        "//modules/localization/common:localization_common",
        "//modules/localization/proto:imu_proto",
        "//modules/localization/proto:pose_proto",
        "//modules/localization/proto:sins_pva_proto",
        "//modules/localization/proto:measure_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/localization/proto:gnss_pnt_result_proto",
        "//modules/drivers/gnss/proto:gnss_raw_observation_proto",
        "//modules/drivers/gnss/proto:gnss_best_pose_proto",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/common/threadpool:localization_msf_common_threadpool",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "//modules/localization/msf/local_map/lossy_map:localization_msf_lossy_map",
        "@glog//:glog",
        "@eigen//:eigen",
        "@yaml_cpp//:yaml",
        "@ros//:ros_common",
    ],
)

cpplint()