load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_traffic_light_onboard",
    srcs = [
        "hdmap_input.cc",
        "preprocessor_subnode.cc",
        "proc_subnode.cc",
    ],
    hdrs = [
        "hdmap_input.h",
        "preprocessor_subnode.h",
        "proc_subnode.h",
        "tl_shared_data.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/configs:config_gflags",
        "//modules/map/hdmap",
        "//modules/map/hdmap:hdmap_util",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/onboard:perception_onboard",
        "//modules/perception/onboard/proto:dag_proto",
        "//modules/perception/traffic_light/base:perception_traffic_light_base",
        "//modules/perception/traffic_light/interface:perception_traffic_light_interface",
        "//modules/perception/traffic_light/preprocessor:perception_traffic_light_preprocessor",
        "//modules/perception/traffic_light/projection:perception_traffic_light_projection",
        "//modules/perception/traffic_light/recognizer:perception_traffic_light_recognizer",
        "//modules/perception/traffic_light/rectify:perception_traffic_light_rectify",
        "//modules/perception/traffic_light/reviser:perception_traffic_light_reviser",
    ],
)

cpplint()
