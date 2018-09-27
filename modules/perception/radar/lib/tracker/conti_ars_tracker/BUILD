load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "conti_ars_tracker",
    srcs = [
        "conti_ars_tracker.cc",
    ],
    hdrs = [
        "conti_ars_tracker.h",
    ],
    deps = [
        "//modules/perception/base:base",
        "//modules/perception/lib/singleton:singleton",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/lib/registerer:registerer",
        "//modules/perception/common/graph:graph",
        "//modules/perception/radar/common:types",
        "//modules/perception/radar/lib/interface:radar_lib_interface",
        "//modules/perception/radar/lib/tracker/common:radar_track_manager",
        "//modules/perception/radar/lib/tracker/matcher:hm_matcher",
        "//modules/perception/proto:perception_config_schema_proto",
        "@eigen",
    ],
)

cpplint()
