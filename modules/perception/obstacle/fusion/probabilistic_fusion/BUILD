load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_fusion_probabilistic_fusion",
    srcs = [
        "pbf_background_track.cc",
        "pbf_base_track_object_matcher.cc",
        "pbf_hm_track_object_matcher.cc",
        "pbf_kalman_motion_fusion.cc",
        "pbf_sensor.cc",
        "pbf_sensor_manager.cc",
        "pbf_sensor_monitor.cc",
        "pbf_sensor_object.cc",
        "pbf_track.cc",
        "pbf_track_manager.cc",
        "pbf_track_object_distance.cc",
        "probabilistic_fusion.cc",
    ],
    hdrs = [
        "pbf_background_track.h",
        "pbf_base_motion_fusion.h",
        "pbf_base_track_object_matcher.h",
        "pbf_hm_track_object_matcher.h",
        "pbf_kalman_motion_fusion.h",
        "pbf_sensor.h",
        "pbf_sensor_manager.h",
        "pbf_sensor_monitor.h",
        "pbf_sensor_object.h",
        "pbf_track.h",
        "pbf_track_manager.h",
        "pbf_track_object_distance.h",
        "probabilistic_fusion.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "//modules/perception/obstacle/common:perception_obstacle_common",
        "//modules/perception/obstacle/fusion/interface:perception_obstacle_fusion_interface",
        "//modules/perception/obstacle/onboard:perception_obstacle_hdmapinput",
        "@eigen//:eigen",
        "@pcl//:pcl",
    ],
)