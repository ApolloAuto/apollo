load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
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
)

cc_library(
    name = "radar_track",
    srcs = ["radar_track.cc"],
    hdrs = ["radar_track.h"],
    deps = [
        "//modules/perception/common/base",
        "//modules/perception/radar_detection/common:types",
        "//modules/perception/radar_detection/interface:base_filter",
        "//modules/perception/radar_detection/lib/tracker/filter:adaptive_kalman_filter",
        "@eigen",
    ],
)

apollo_package()
cpplint()
