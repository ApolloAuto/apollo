load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "hm_matcher_test",
    size = "small",
    srcs = [
        "hm_matcher_test.cc",
    ],
    deps = [
        "//modules/perception/radar/lib/tracker/common:radar_track_manager",
        ":hm_matcher",
        "@gtest//:main",
    ],
)

cc_library(
    name = "hm_matcher",
    srcs = [
        "hm_matcher.cc",
    ],
    hdrs = [
        "hm_matcher.h",
    ],
    deps = [
        "//modules/common/util:file_util",
        "//modules/common/util:string_util",
        "//modules/perception/base:base",
        "//modules/perception/lib/singleton:singleton",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/lib/registerer:registerer",
        "//modules/perception/common/graph:graph",
        "//modules/perception/radar/common:types",
        "//modules/perception/radar/lib/tracker/common:radar_track",
        "//modules/perception/radar/lib/interface:base_matcher",
        "//modules/perception/proto:perception_config_schema_proto",
        "//modules/perception/proto:tracker_config_proto",
        "@eigen",
    ],
)

cpplint()
