load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "base_matcher_test",
    size = "small",
    srcs = ["base_matcher_test.cc"],
    deps = [
        ":base_matcher",
        "//modules/perception/radar_detection/lib/tracker/common:radar_track_manager",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "base_matcher",
    srcs = ["base_matcher.cc"],
    hdrs = ["base_matcher.h"],
    deps = [
        "//modules/perception/common/base",
        "//modules/perception/common/lib/interface:base_init_options",
        "//modules/perception/common/lib/registerer",
        "//modules/perception/radar_detection/lib/tracker/common:radar_track",
        "//modules/perception/radar_detection/proto:radar_obstacle_perception_cc_proto",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

cc_library(
    name = "base_filter",
    hdrs = ["base_filter.h"],
    deps = [
        "@eigen",
    ],
)

cc_library(
    name = "base_detector",
    hdrs = ["base_detector.h"],
    deps = [
        "//modules/common_msgs/sensor_msgs:conti_radar_cc_proto",
        "//modules/perception/common/base:base_type",
        "//modules/perception/common/lib/interface:base_init_options",
        "@eigen",
    ],
)

cc_library(
    name = "base_preprocessor",
    hdrs = ["base_preprocessor.h"],
    deps = [
        "//modules/common_msgs/sensor_msgs:conti_radar_cc_proto",
        "//modules/perception/common/lib/interface:base_init_options",
    ],
)

cc_library(
    name = "base_radar_obstacle_perception",
    hdrs = ["base_radar_obstacle_perception.h"],
    deps = [
        "base_detector",
        "base_roi_filter",
        "base_tracker",
        "//modules/perception/common/lib/interface:base_init_options",
    ]
)

cc_library(
    name = "base_roi_filter",
    hdrs = ["base_roi_filter.h"],
    deps = [
        "//modules/perception/common/algorithm/geometry:roi_filter",
        "//modules/perception/common/lib/interface:base_init_options",
    ]
)

cc_library(
    name = "base_tracker",
    hdrs = ["base_tracker.h"],
    deps = [
        "//modules/perception/common/lib/interface:base_init_options",
    ]
)

apollo_package()
cpplint()
