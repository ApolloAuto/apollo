load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
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
        "//modules/perception/common:perception_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "conti_ars_tracker",
    srcs = ["conti_ars_tracker.cc"],
    hdrs = ["conti_ars_tracker.h"],
    deps = [
        "//cyber",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common:util",
        "//modules/perception/common/base",
        "//modules/perception/common/lib/registerer",
        "//modules/perception/radar_detection/common:types",
        "//modules/perception/radar_detection/interface:base_matcher",
        "//modules/perception/radar_detection/interface:base_tracker",
        "//modules/perception/radar_detection/lib/tracker/common:radar_track_manager",
        "//modules/perception/radar_detection/lib/tracker/matcher:hm_matcher",
        "//modules/perception/radar_detection/proto:tracker_config_cc_proto",
        "@eigen",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
