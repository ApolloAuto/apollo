load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "hdmap_radar_roi_filter_test",
    size = "small",
    srcs = ["hdmap_radar_roi_filter_test.cc"],
    deps = [
        ":hdmap_radar_roi_filter",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "hdmap_radar_roi_filter",
    srcs = ["hdmap_radar_roi_filter.cc"],
    hdrs = ["hdmap_radar_roi_filter.h"],
    deps = [
        "//modules/perception/base",
        "//modules/perception/common/geometry:roi_filter",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/proto:perception_config_schema_proto",
        "//modules/perception/radar/lib/interface:base_roi_filter",
        "@eigen",
    ],
    alwayslink = True,
)

cpplint()
