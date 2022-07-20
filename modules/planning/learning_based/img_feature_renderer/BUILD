load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "birdview_img_feature_renderer",
    srcs = ["birdview_img_feature_renderer.cc"],
    hdrs = ["birdview_img_feature_renderer.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/util",
        "//modules/map/hdmap:hdmap_util",
        "//modules/common_msgs/perception_msgs:traffic_light_detection_cc_proto",
        "//modules/planning/proto:learning_data_cc_proto",
        "//modules/planning/proto:planning_semantic_map_config_cc_proto",
        "@opencv//:imgcodecs",
    ],
)

cpplint()
