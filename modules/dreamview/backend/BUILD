load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("//tools/platform:build_defs.bzl", "copts_if_teleop", "if_teleop")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

DREAMVIEW_COPTS = ['-DMODULE_NAME=\\"dreamview\\"']

cc_library(
    name = "backend",
    srcs = ["libbackend.so"],
    hdrs = ["dreamview.h"],
    copts = DREAMVIEW_COPTS + copts_if_teleop(),
    alwayslink = True,
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/status",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/handlers:image_handler",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/dreamview/backend/hmi",
        "//modules/dreamview/backend/map:map_service",
        "//modules/dreamview/backend/perception_camera_updater",
        "//modules/dreamview/backend/plugins:plugin_manager",
        "//modules/dreamview/backend/point_cloud:point_cloud_updater",
        "//modules/dreamview/backend/sim_control_manager",
        "//modules/dreamview/backend/sim_control_manager/core:sim_control_base",
        "//modules/dreamview/backend/sim_control_manager/core:sim_control_with_model_base",
        "//modules/dreamview/backend/sim_control_manager/common:interpolation_2d",
        "//modules/dreamview/backend/simulation_world:simulation_world_updater",
        "//modules/dreamview/proto:simulation_world_cc_proto",
        "//modules/dreamview/backend/sim_control_manager/proto:sim_control_internal_cc_proto",
        "//modules/dreamview/backend/sim_control_manager/proto:dynamic_model_conf_cc_proto",
        #"//modules/dreamview/backend/sim_control_manager/proto:fnn_model_cc_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/pnc_map",
        "//modules/common_msgs/planning_msgs:pad_msg_cc_proto",
        "@civetweb//:civetweb++",
    ] + if_teleop(["//modules/dreamview/backend/teleop"]),
)

cc_binary(
    name = "libbackend.so",
    srcs = ["dreamview.cc", "dreamview.h"],
    copts = DREAMVIEW_COPTS + copts_if_teleop(),
    linkstatic = True,
    linkshared = True,
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/status",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/handlers:image_handler",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/dreamview/backend/hmi",
        "//modules/dreamview/backend/map:map_service",
        "//modules/dreamview/backend/perception_camera_updater",
        "//modules/dreamview/backend/plugins:plugin_manager",
        "//modules/dreamview/backend/point_cloud:point_cloud_updater",
        "//modules/dreamview/backend/sim_control_manager",
        "//modules/dreamview/backend/sim_control_manager/core:sim_control_base",
        "//modules/dreamview/backend/sim_control_manager/core:sim_control_with_model_base",
        "//modules/dreamview/backend/sim_control_manager/common:interpolation_2d",
        "//modules/dreamview/backend/simulation_world:simulation_world_updater",
        "//modules/dreamview/proto:simulation_world_cc_proto",
        "//modules/dreamview/backend/sim_control_manager/proto:sim_control_internal_cc_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/pnc_map",
        "//modules/common_msgs/planning_msgs:pad_msg_cc_proto",
        "@civetweb//:civetweb++",
    ] + if_teleop(["//modules/dreamview/backend/teleop"]),
)

install(
    name = "install",
    library_dest = "dreamview/lib",
    data_dest = "dreamview/addition_data/backend",
    data = ["//modules/dreamview/backend/testdata:testdata"],
    targets = [":libbackend.so"]
)

cpplint()
