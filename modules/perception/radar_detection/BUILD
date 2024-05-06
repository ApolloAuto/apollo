load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "radar_detection_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
        "launch/**",
    ]),
)

apollo_cc_library(
    name = "apollo_perception_radar_detection",
    srcs = [
        "app/radar_obstacle_perception.cc",
        "common/radar_util.cc",
        "interface/base_matcher.cc",
        "lib/detector/conti_ars_detector/conti_ars_detector.cc",
        "lib/dummy/dummy_algorithms.cc",
        "lib/preprocessor/conti_ars_preprocessor/conti_ars_preprocessor.cc",
        "lib/roi_filter/hdmap_radar_roi_filter/hdmap_radar_roi_filter.cc",
        "lib/tracker/common/radar_track.cc",
        "lib/tracker/common/radar_track_manager.cc",
        "lib/tracker/conti_ars_tracker/conti_ars_tracker.cc",
        "lib/tracker/filter/adaptive_kalman_filter.cc",
        "lib/tracker/matcher/hm_matcher.cc",
    ],
    hdrs = [
        "app/radar_obstacle_perception.h",
        "common/radar_util.h",
        "common/types.h",
        "interface/base_detector.h",
        "interface/base_filter.h",
        "interface/base_matcher.h",
        "interface/base_preprocessor.h",
        "interface/base_radar_obstacle_perception.h",
        "interface/base_roi_filter.h",
        "interface/base_tracker.h",
        "lib/detector/conti_ars_detector/conti_ars_detector.h",
        "lib/dummy/dummy_algorithms.h",
        "lib/preprocessor/conti_ars_preprocessor/conti_ars_preprocessor.h",
        "lib/roi_filter/hdmap_radar_roi_filter/hdmap_radar_roi_filter.h",
        "lib/tracker/common/radar_track.h",
        "lib/tracker/common/radar_track_manager.h",
        "lib/tracker/conti_ars_tracker/conti_ars_tracker.h",
        "lib/tracker/filter/adaptive_kalman_filter.h",
        "lib/tracker/matcher/hm_matcher.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/sensor_msgs:conti_radar_cc_proto",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/hdmap:apollo_perception_common_hdmap",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/radar_detection/proto:matcher_config_cc_proto",
        "//modules/perception/radar_detection/proto:preprocessor_cc_proto",
        "//modules/perception/radar_detection/proto:radar_component_config_cc_proto",
        "//modules/perception/radar_detection/proto:radar_obstacle_perception_cc_proto",
        "//modules/perception/radar_detection/proto:tracker_config_cc_proto",
        "@com_google_absl//:absl",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

apollo_component(
    name = "libradar_detection.so",
    srcs = [
        "radar_detection_component.cc",
    ],
    hdrs = [
        "radar_detection_component.h",
    ],
    deps = [
        "//cyber",
        ":apollo_perception_radar_detection",
        "//modules/common/util:util_tool",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/hdmap:apollo_perception_common_hdmap",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/radar_detection/proto:radar_component_config_cc_proto",
    ],
)

apollo_cc_test(
    name = "base_matcher_test",
    size = "small",
    srcs = ["interface/base_matcher_test.cc"],
    deps = [
        ":apollo_perception_radar_detection",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "conti_ars_detector_test",
    size = "small",
    srcs = ["lib/detector/conti_ars_detector/conti_ars_detector_test.cc"],
    deps = [
        ":apollo_perception_radar_detection",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "dummy_algorithms_test",
    size = "small",
    srcs = ["lib/dummy/dummy_algorithms_test.cc"],
    deps = [
        ":apollo_perception_radar_detection",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "conti_ars_preprocessor_test",
    size = "small",
    srcs = ["lib/preprocessor/conti_ars_preprocessor/conti_ars_preprocessor_test.cc"],
    deps = [
        ":apollo_perception_radar_detection",
        "//modules/perception/common:perception_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "hdmap_radar_roi_filter_test",
    size = "small",
    srcs = ["lib/roi_filter/hdmap_radar_roi_filter/hdmap_radar_roi_filter_test.cc"],
    deps = [
        ":apollo_perception_radar_detection",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "radar_track_manager_test",
    size = "small",
    srcs = ["lib/tracker/common/radar_track_manager_test.cc"],
    deps = [
        ":apollo_perception_radar_detection",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "radar_track_test",
    size = "small",
    srcs = ["lib/tracker/common/radar_track_test.cc"],
    deps = [
        ":apollo_perception_radar_detection",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "conti_ars_tracker_test",
    size = "small",
    srcs = ["lib/tracker/conti_ars_tracker/conti_ars_tracker_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":apollo_perception_radar_detection",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "conti_ars_tracker_empty_test",
    size = "small",
    srcs = ["lib/tracker/conti_ars_tracker/conti_ars_tracker_empty_test.cc"],
    deps = [
        ":apollo_perception_radar_detection",
        "//modules/perception/common:perception_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "adaptive_kalman_filter_test",
    size = "small",
    srcs = ["lib/tracker/filter/adaptive_kalman_filter_test.cc"],
    deps = [
        ":apollo_perception_radar_detection",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "hm_matcher_test",
    size = "small",
    srcs = ["lib/tracker/matcher/hm_matcher_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":apollo_perception_radar_detection",
        "//modules/perception/common:perception_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

cpplint()
