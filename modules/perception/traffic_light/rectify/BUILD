load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_traffic_light_rectify",
    srcs = [
        "cropbox.cc",
        "select.cc",
        "unity_rectify.cc",
        "detection.cc",
    ],
    hdrs = [
        "cropbox.h",
        "select.h",
        "unity_rectify.h",
        "detection.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/proto:error_code_proto",
        "//modules/common/proto:header_proto",
        "//modules/map/proto:map_proto",
        "//modules/perception/lib/config_manager",
        "//modules/perception/proto:perception_proto",
        "//modules/perception/obstacle/common:perception_obstacle_common",
        "//modules/perception/traffic_light/base:perception_traffic_light_base",
        "//modules/perception/traffic_light/interface:perception_traffic_light_interface",
    ],
)

cpplint()
