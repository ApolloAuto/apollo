load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "debug_info",
    srcs = ["debug_info.cc"],
    hdrs = ["debug_info.h"],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
    ],
)

cc_library(
    name = "traffic_light_camera_perception",
    srcs = ["traffic_light_camera_perception.cc"],
    hdrs = ["traffic_light_camera_perception.h"],
    deps = [
        "//cyber",
        "//modules/common/util:perf_util",
        "//modules/perception/camera/app/proto:perception_cc_proto",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/traffic_light/detector/detection",
        "//modules/perception/camera/lib/traffic_light/detector/recognition",
        "//modules/perception/camera/lib/traffic_light/proto:detection_cc_proto",
        "//modules/perception/camera/lib/traffic_light/tracker:semantic_decision",
        "//modules/perception/camera/lib/traffic_light/tracker/proto:semantic_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "cipv_camera",
    srcs = ["cipv_camera.cc"],
    hdrs = ["cipv_camera.h"],
    deps = [
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math:geometry",
        "//modules/perception/base",
        "//modules/perception/camera/common:lane_object",
        "//modules/perception/camera/lib/interface",
    ],
)

cpplint()
