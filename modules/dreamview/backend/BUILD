load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools/platform:build_defs.bzl", "copts_if_teleop", "if_teleop")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

DREAMVIEW_COPTS = ['-DMODULE_NAME=\\"dreamview\\"']

cc_library(
    name = "backend",
    srcs = ["dreamview.cc"],
    hdrs = ["dreamview.h"],
    copts = DREAMVIEW_COPTS + copts_if_teleop(),
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/status",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/data_collection_monitor",
        "//modules/dreamview/backend/handlers:image_handler",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/dreamview/backend/hmi",
        "//modules/dreamview/backend/map:map_service",
        "//modules/dreamview/backend/perception_camera_updater",
        "//modules/dreamview/backend/point_cloud:point_cloud_updater",
        "//modules/dreamview/backend/sim_control",
        "//modules/dreamview/backend/simulation_world:simulation_world_updater",
        "//modules/dreamview/proto:simulation_world_cc_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/pnc_map",
        "//modules/planning/proto:pad_msg_cc_proto",
        "@civetweb//:civetweb++",
    ] + if_teleop(["//modules/dreamview/backend/teleop"]),
)

cpplint()
