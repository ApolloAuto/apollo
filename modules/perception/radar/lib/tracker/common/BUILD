load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "radar_track_manager_test",
    size = "small",
    srcs = ["radar_track_manager_test.cc"],
    deps = [
        ":radar_track_manager",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "radar_track_test",
    size = "small",
    srcs = ["radar_track_test.cc"],
    deps = [
        ":radar_track",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "radar_track_manager",
    srcs = ["radar_track_manager.cc"],
    hdrs = ["radar_track_manager.h"],
    deps = [
        ":radar_track",
        "@eigen",
    ],
    alwayslink = True,
)

cc_library(
    name = "radar_track",
    srcs = ["radar_track.cc"],
    hdrs = ["radar_track.h"],
    deps = [
        "//modules/perception/base",
        "//modules/perception/radar/common:types",
        "//modules/perception/radar/lib/interface:base_filter",
        "//modules/perception/radar/lib/tracker/filter:adaptive_kalman_filter",
        "@eigen",
    ],
    alwayslink = True,
)

cpplint()
