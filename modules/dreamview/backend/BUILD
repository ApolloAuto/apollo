load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "backend",
    srcs = [
        "dreamview.cc",
        "main.cc",
    ],
    hdrs = [
        "dreamview.h",
    ],
    copts = ['-DMODULE_NAME=\\"dreamview\\"'],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/status",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/data_collection_monitor",
        "//modules/dreamview/backend/handlers:image_handler",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/dreamview/backend/hmi",
        "//modules/dreamview/backend/map:map_service",
        "//modules/dreamview/backend/point_cloud:point_cloud_updater",
        "//modules/dreamview/backend/sim_control",
        "//modules/dreamview/backend/simulation_world:simulation_world_updater",
        "//modules/dreamview/proto:simulation_world_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/pnc_map",
        "@civetweb//:civetweb++",
    ],
)

cpplint()
