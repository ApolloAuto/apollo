load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "birdview_img_feature_renderer",
    hdrs = ["birdview_img_feature_renderer.h"],
    srcs = ["birdview_img_feature_renderer.cc"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/util",
        "//modules/map/hdmap:hdmap_util",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/proto:learning_data_proto",
        "//modules/planning/proto:planning_semantic_map_config_proto",
        "@opencv",
    ],
)

cpplint()
