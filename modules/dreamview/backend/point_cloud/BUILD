load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "point_cloud_updater",
    srcs = ["point_cloud_updater.cc"],
    hdrs = ["point_cloud_updater.h"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/math",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/dreamview/backend/simulation_world:simulation_world_updater",
        "//modules/dreamview/proto:point_cloud_cc_proto",
        "//modules/drivers/proto:pointcloud_cc_proto",
        "//modules/drivers/proto:sensor_image_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "@boost",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_github_nlohmann_json//:json",
        "@com_google_protobuf//:protobuf",
        "@local_config_pcl//:pcl",
    ],
)

cpplint()
