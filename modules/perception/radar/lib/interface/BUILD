load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "base_matcher_test",
    size = "small",
    srcs = ["base_matcher_test.cc"],
    deps = [
        ":base_matcher",
        "//modules/perception/radar/lib/tracker/common:radar_track_manager",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "base_matcher",
    srcs = ["base_matcher.cc"],
    hdrs = ["base_matcher.h"],
    deps = [
        "//modules/perception/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/proto:perception_config_schema_cc_proto",
        "//modules/perception/radar/common:types",
        "//modules/perception/radar/lib/tracker/common:radar_track",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

cc_library(
    name = "base_filter",
    hdrs = ["base_filter.h"],
)

cc_library(
    name = "base_detector",
    hdrs = ["base_detector.h"],
    deps = [
        "//modules/drivers/canbus/proto:can_card_parameter_cc_proto",
        "//modules/drivers/proto:conti_radar_cc_proto",
        "//modules/drivers/radar/conti_radar/proto:conti_radar_conf_cc_proto",
    ],
)

cc_library(
    name = "base_preprocessor",
    hdrs = ["base_preprocessor.h"],
    deps = [
        "//modules/drivers/canbus/proto:can_card_parameter_cc_proto",
        "//modules/drivers/proto:conti_radar_cc_proto",
        "//modules/drivers/radar/conti_radar/proto:conti_radar_conf_cc_proto",
    ],
)

cc_library(
    name = "base_radar_obstacle_perception",
    hdrs = ["base_radar_obstacle_perception.h"],
)

cc_library(
    name = "base_roi_filter",
    hdrs = ["base_roi_filter.h"],
)

cc_library(
    name = "base_tracker",
    hdrs = ["base_tracker.h"],
)

cpplint()
