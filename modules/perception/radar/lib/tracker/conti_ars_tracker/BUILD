load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "conti_ars_tracker_test",
    size = "small",
    srcs = ["conti_ars_tracker_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":conti_ars_tracker",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "conti_ars_tracker_empty_test",
    size = "small",
    srcs = ["conti_ars_tracker_empty_test.cc"],
    deps = [
        ":conti_ars_tracker",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "conti_ars_tracker",
    srcs = ["conti_ars_tracker.cc"],
    hdrs = ["conti_ars_tracker.h"],
    deps = [
        "//modules/perception/base",
        "//modules/perception/common/graph",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/proto:perception_config_schema_cc_proto",
        "//modules/perception/radar/common:types",
        "//modules/perception/radar/lib/interface:base_matcher",
        "//modules/perception/radar/lib/interface:base_tracker",
        "//modules/perception/radar/lib/tracker/common:radar_track_manager",
        "//modules/perception/radar/lib/tracker/matcher:hm_matcher",
        "@eigen",
    ],
    alwayslink = True,
)

cpplint()
