load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hm_tracker",
    srcs = [
        "feature_descriptor.cc",
        "hm_tracker.cc",
        "hungarian_matcher.cc",
        "kalman_filter.cc",
        "object_track.cc",
        "track_object_distance.cc",
        "tracked_object.cc",
    ],
    hdrs = [
        "base_filter.h",
        "base_matcher.h",
        "feature_descriptor.h",
        "hm_tracker.h",
        "hungarian_matcher.h",
        "kalman_filter.h",
        "object_track.h",
        "track_object_distance.h",
        "tracked_object.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/perception/common",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/common",
        "//modules/perception/obstacle/lidar/interface",
        "//modules/perception/obstacle/onboard:hdmapinput",
        "@eigen",
        "@pcl",
    ],
)

cc_test(
    name = "hm_tracker_test",
    size = "small",
    srcs = [
        "hm_tracker_test.cc",
    ],
    data = [
        "//modules/perception:perception_data",
        "//modules/perception:perception_model",
        "//modules/perception/conf:perception_config",
    ],
    linkopts = [
        "-lqhull",
    ],
    deps = [
        ":hm_tracker",
        "//modules/common:log",
        "//modules/common/util",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/lidar/object_builder/min_box",
        "@gtest//:main",
        "@pcl",
    ],
)

cpplint()
