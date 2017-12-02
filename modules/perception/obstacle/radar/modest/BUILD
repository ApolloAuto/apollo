load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_radar_modest_modest_detector",
    srcs = [
        "modest_radar_detector.cc",
        "conti_radar_util.cc",
        "radar_util.cc",
        "radar_track.cc",
        "radar_track_manager.cc",
       "conti_radar_id_expansion.cc",
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
        "@eigen//:eigen",
        "@pcl//:pcl",
    ],
)

cc_test(
    name = "conti_radar_id_expansion_test",
    size = "small",
    srcs = [
        "conti_radar_id_expansion_test.cc",
    ],
    deps = [
        "//external:gflags",
        "//modules/common:log",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/radar/modest:perception_obstacle_radar_modest_modest_detector",
        "@gtest//:gtest",
        "@gtest//:main",
    ],
)

cc_test(
    name = "modest_radar_detector_test",
    size = "small",
    srcs = [
        "modest_radar_detector_test.cc",
    ],
    data = [
        "//modules/perception:perception_data",
        "//modules/perception:perception_model",
        "//modules/perception/conf:perception_config",
    ],
    deps = [
        "//external:gflags",
        "//modules/common:log",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/radar/modest:perception_obstacle_radar_modest_modest_detector",
        "@gtest//:gtest",
        "@gtest//:main",
    ],
)

cc_test(
    name = "conti_radar_util_test",
    size = "small",
    srcs = [
        "conti_radar_util_test.cc",
    ],
    data = [
        "//modules/perception:perception_data",
        "//modules/perception:perception_model",
        "//modules/perception/conf:perception_config",
    ],
    deps = [
        "//external:gflags",
        "//modules/common:log",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/radar/modest:perception_obstacle_radar_modest_modest_detector",
        "@gtest//:gtest",
        "@gtest//:main",
    ],
)

cpplint()
