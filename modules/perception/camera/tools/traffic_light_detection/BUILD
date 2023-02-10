load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "traffic_light_detection",
    srcs = ["traffic_light_detection.cc"],
    deps = [
        "//cyber",
        "//modules/perception/camera/app:traffic_light_camera_perception",
        "//modules/perception/camera/tools/common",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/pipeline/proto:pipeline_config_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
