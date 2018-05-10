load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "onboard",
    srcs = [
        "hdmap_input.cc",
        "tl_preprocessor_subnode.cc",
        "tl_proc_subnode.cc",
    ],
    hdrs = [
        "hdmap_input.h",
        "tl_preprocessor_subnode.h",
        "tl_proc_subnode.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/configs:config_gflags",
        "//modules/map/hdmap",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/proto:map_proto",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/onboard",
        "//modules/perception/onboard/proto:dag_proto",
        "//modules/perception/traffic_light/base",
        "//modules/perception/traffic_light/interface",
        "//modules/perception/traffic_light/preprocessor:perception_traffic_light_preprocessor",
        "//modules/perception/traffic_light/projection:perception_traffic_light_projection",
        "//modules/perception/traffic_light/recognizer:perception_traffic_light_recognizer",
        "//modules/perception/traffic_light/rectify:perception_traffic_light_rectify",
        "//modules/perception/traffic_light/reviser:perception_traffic_light_reviser",
    ],
)

cpplint()
