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
    deps = [
        "//modules/common/configs/proto:vehicle_config_proto",
        "//modules/common/status",
        "//modules/dreamview/backend/handlers:websocket_handler",
        # "//modules/dreamview/backend/hmi",
        "//modules/dreamview/backend/map:map_service",
        # "//modules/dreamview/backend/point_cloud:point_cloud_updater",
        # "//modules/dreamview/backend/sim_control",
        # "//modules/dreamview/backend/simulation_world:simulation_world_updater",
        "//modules/dreamview/proto:simulation_world_proto",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/pnc_map",
        "@civetweb//:civetweb++",
    ],
)

cpplint()
