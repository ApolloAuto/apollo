load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "radar_track_manager",
    srcs = [
        "radar_track_manager.cc",
    ],
    hdrs = [
        "radar_track_manager.h",
    ],
    deps = [
        ":radar_track",
        "@eigen",
    ],
)

cc_library(
    name = "radar_track",
    srcs = [
        "radar_track.cc",
    ],
    hdrs = [
        "radar_track.h",
    ],
    deps = [
        "//modules/perception/base:base",
        "//modules/perception/radar/common:types",
        "//modules/perception/radar/lib/interface:radar_lib_interface",
        "//modules/perception/radar/lib/tracker/filter:adaptive_kalman_filter",
        "@eigen",
    ],
)

cpplint()
