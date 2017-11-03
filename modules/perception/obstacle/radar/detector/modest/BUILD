load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_radar_detector_modest_detector",
    srcs = [
        "modest_radar_detector.cc",
        "radar_track.cc",
        "radar_track_manager.cc",
        "object_builder.cc",
    ],
    hdrs = [
        "modest_radar_detector.h",
        "conti_radar_util.h",
        "radar_define.h",
        "radar_util.h",
        "radar_track.h",
        "radar_track_manager.h",
        "conti_radar_id_expansion.h",
        "object_builder.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/common:perception_obstacle_common",
        "//modules/perception/obstacle/radar/interface:perception_obstacle_radar_interface",
        "//modules/perception/obstacle/radar/filter/akf:perception_obstacle_radar_filter_adaptive_kalman_filter",
        "@eigen//:eigen",
        "@pcl//:pcl",
    ],
)

cpplint()
