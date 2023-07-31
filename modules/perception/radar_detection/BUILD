load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "radar_detection_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "launch/**",
    ])
)

cc_binary(
    name = "libradar_detection.so",
    linkshared = True,
    linkstatic = True,
    deps = [":radar_detection_component"],
)

cc_library(
    name = "radar_detection_component",
    srcs = [
        "radar_detection_component.cc",
    ],
    hdrs = [
        "radar_detection_component.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/perception/common/algorithm/sensor_manager",
        "//modules/perception/common/base",
        "//modules/perception/common/hdmap:hdmap_input",
        "//modules/perception/common/lib/registerer",
        "//modules/perception/common/onboard/common_flags",
        "//modules/perception/common/onboard/inner_component_messages",
        "//modules/perception/common/onboard/msg_buffer",
        "//modules/perception/common/onboard/transform_wrapper",
        "//modules/perception/radar_detection/app:radar_obstacle_perception",
        "//modules/perception/radar_detection/common:radar_util",
        "//modules/perception/radar_detection/interface:base_preprocessor",
        "//modules/perception/radar_detection/interface:base_radar_obstacle_perception",
        "//modules/perception/radar_detection/lib/detector/conti_ars_detector",
        "//modules/perception/radar_detection/lib/dummy:dummy_algorithms",
        "//modules/perception/radar_detection/lib/preprocessor/conti_ars_preprocessor",
        "//modules/perception/radar_detection/lib/roi_filter/hdmap_radar_roi_filter",
        "//modules/perception/radar_detection/lib/tracker/common:radar_track_manager",
        "//modules/perception/radar_detection/lib/tracker/common:radar_track",
        "//modules/perception/radar_detection/lib/tracker/conti_ars_tracker",
        "//modules/perception/radar_detection/lib/tracker/filter:adaptive_kalman_filter",
        "//modules/perception/radar_detection/lib/tracker/matcher:hm_matcher",
        "//modules/perception/radar_detection/proto:radar_component_config_cc_proto",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
