load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_camera_updater",
    srcs = ["perception_camera_updater.cc"],
    hdrs = ["perception_camera_updater.h"],
    copts = ['-DMODULE_NAME=\\"dreamview\\"'],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/dreamview/proto:camera_update_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/transform:buffer",
        "//modules/common_msgs/transform_msgs:transform_cc_proto",
        "@eigen",
        "@opencv//:imgcodecs",
    ],
)

cpplint()
