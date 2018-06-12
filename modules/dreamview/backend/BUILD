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
        "//modules/common:apollo_app",
        "//modules/common/adapters:adapter_manager",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/dreamview/backend/hmi",
        "//modules/dreamview/backend/point_cloud:point_cloud_updater",
        "//modules/dreamview/backend/sim_control",
        "//modules/dreamview/backend/simulation_world:simulation_world_updater",
        "//modules/map/hdmap:hdmap_util",
        "@civetweb//:civetweb++",
    ],
)

cpplint()
