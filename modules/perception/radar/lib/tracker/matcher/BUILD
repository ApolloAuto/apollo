load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "hm_matcher_test",
    size = "small",
    srcs = ["hm_matcher_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":hm_matcher",
        "//modules/perception/radar/lib/tracker/common:radar_track_manager",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "hm_matcher",
    srcs = ["hm_matcher.cc"],
    hdrs = ["hm_matcher.h"],
    deps = [
        "//cyber/common:file",
        "//modules/perception/base",
        "//modules/perception/common/graph",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/proto:perception_config_schema_cc_proto",
        "//modules/perception/proto:tracker_config_cc_proto",
        "//modules/perception/radar/common:types",
        "//modules/perception/radar/lib/interface:base_matcher",
        "//modules/perception/radar/lib/tracker/common:radar_track",
        "@com_google_absl//absl/strings",
        "@eigen",
    ],
)

cpplint()
