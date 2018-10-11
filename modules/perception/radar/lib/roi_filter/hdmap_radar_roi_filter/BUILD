load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "hdmap_radar_roi_filter_test",
    size = "small",
    srcs = [
        "hdmap_radar_roi_filter_test.cc",
    ],
    deps = [
        ":hdmap_radar_roi_filter",
        "@gtest//:main",
    ],
)

cc_library(
    name = "hdmap_radar_roi_filter",
    srcs = [
        "hdmap_radar_roi_filter.cc",
    ],
    hdrs = [
        "hdmap_radar_roi_filter.h",
    ],
    deps = [
        "//modules/perception/base:base",
        "//modules/perception/lib/singleton:singleton",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/lib/registerer:registerer",
        "//modules/perception/common/geometry:roi_filter",
        "//modules/perception/radar/lib/interface:base_roi_filter",
        "//modules/perception/proto:perception_config_schema_proto",
        "@eigen",
    ],
)

cpplint()
