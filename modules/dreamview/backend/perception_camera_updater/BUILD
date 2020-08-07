load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_camera_updater",
    srcs = ["perception_camera_updater.cc"],
    hdrs = ["perception_camera_updater.h"],
    copts = ['-DMODULE_NAME=\\"dreamview\\"'],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/proto:geometry_cc_proto",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/dreamview/proto:camera_update_cc_proto",
        "//modules/drivers/proto:sensor_image_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/transform:buffer",
        "//modules/transform/proto:transform_cc_proto",
        "@eigen",
        "@opencv//:imgcodecs",
    ],
)

cpplint()
