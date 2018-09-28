load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "radar_conti_ars_detector_test",
    size = "small",
    srcs = [
        "radar_conti_ars_detector_test.cc",
    ],
    deps = [
        "//modules/perception/radar/lib/detector/conti_ars_detector:conti_ars_detector",
        "@gtest//:main",
    ],
)

cc_test(
    name = "radar_conti_ars_preprocessor_test",
    srcs = [
        "radar_conti_ars_preprocessor_test.cc",
    ],
    deps = [
        "//modules/perception/radar/lib/preprocessor/conti_ars_preprocessor:conti_ars_preprocessor",
        "@gtest//:main",
    ],
)

cc_test(
    name = "radar_dummy_algorithms_test",
    srcs = [
        "radar_dummy_algorithms_test.cc",
    ],
    deps = [
        "//modules/perception/radar/lib/dummy:dummy_algorithms",
        "@gtest//:main",
    ],
)

cc_test(
    name = "radar_filter_adaptive_kalman_filter_test",
    srcs = [
        "radar_filter_adaptive_kalman_filter_test.cc",
    ],
    deps = [
        "//modules/perception/radar/lib/tracker/filter:adaptive_kalman_filter",
        "@gtest//:main",
    ],
)

cc_test(
    name = "radar_hdmap_radar_roi_filter_test",
    srcs = [
        "radar_hdmap_radar_roi_filter_test.cc",
    ],
    deps = [
        "//modules/perception/radar/lib/roi_filter/hdmap_radar_roi_filter:hdmap_radar_roi_filter",
        "@gtest//:main",
    ],
)

cc_test(
    name = "radar_interface_base_matcher_test",
    srcs = [
        "radar_interface_base_matcher_test.cc",
    ],
    deps = [
        "//modules/perception/radar/lib/tracker/common:radar_track_manager",
        "//modules/perception/radar/lib/interface:base_matcher",
        "@gtest//:main",
    ],
)

cc_test(
    name = "radar_tracker_conti_ars_tracker_empty_test",
    srcs = [
        "radar_tracker_conti_ars_tracker_empty_test.cc",
    ],
    deps = [
        "//modules/perception/radar/lib/tracker/conti_ars_tracker:conti_ars_tracker",
        "@gtest//:main",
    ],
)

cc_test(
    name = "radar_tracker_conti_ars_tracker_test",
    srcs = [
        "radar_tracker_conti_ars_tracker_test.cc",
    ],
    deps = [
        "//modules/perception/radar/lib/tracker/conti_ars_tracker:conti_ars_tracker",
        "@gtest//:main",
    ],
)

cc_test(
    name = "radar_tracker_hm_matcher_test",
    srcs = [
        "radar_tracker_hm_matcher_test.cc",
    ],
    deps = [
        "//modules/perception/radar/lib/tracker/common:radar_track_manager",
        "//modules/perception/radar/lib/tracker/matcher:hm_matcher",
        "@gtest//:main",
    ],
)

cc_test(
    name = "radar_tracker_radar_track_manager_test",
    srcs = [
        "radar_tracker_radar_track_manager_test.cc",
    ],
    deps = [
        "//modules/perception/radar/lib/tracker/common:radar_track_manager",
        "@gtest//:main",
    ],
)

cc_test(
    name = "radar_tracker_radar_track_test",
    srcs = [
        "radar_tracker_radar_track_test.cc",
    ],
    deps = [
        "//modules/perception/radar/lib/tracker/common:radar_track",
        "@gtest//:main",
    ],
)

cpplint()
