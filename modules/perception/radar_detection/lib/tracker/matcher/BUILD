load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "hm_matcher_test",
    size = "small",
    srcs = ["hm_matcher_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":hm_matcher",
        "//modules/perception/radar_detection/lib/tracker/common:radar_track_manager",
        "//modules/perception/common:perception_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "hm_matcher",
    srcs = ["hm_matcher.cc"],
    hdrs = ["hm_matcher.h"],
    deps = [
        "//cyber",
        "//modules/perception/common:util",
        "//modules/perception/common/base",
        "//modules/perception/common/lib/registerer",
        "//modules/perception/radar_detection/common:types",
        "//modules/perception/radar_detection/interface:base_matcher",
        "//modules/perception/radar_detection/lib/tracker/common:radar_track",
        "//modules/perception/radar_detection/proto:matcher_config_cc_proto",
        "//modules/perception/common/algorithm/graph",
        "@com_google_absl//:absl",
        "@eigen",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
