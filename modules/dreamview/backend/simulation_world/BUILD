load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

DREAMVIEW_COPTS = ['-DMODULE_NAME=\\"dreamview\\"']

cc_library(
    name = "simulation_world_service",
    srcs = ["simulation_world_service.cc"],
    hdrs = ["simulation_world_service.h"],
    copts = DREAMVIEW_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/audio_msgs:audio_cc_proto",
        "//modules/common_msgs/audio_msgs:audio_event_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/monitor_log",
        "//modules/common_msgs/basic_msgs:drive_event_cc_proto",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common/util",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/map:map_service",
        "//modules/dreamview/proto:simulation_world_cc_proto",
        "//modules/common_msgs/localization_msgs:gps_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_cc_proto",
        "//modules/common_msgs/storytelling_msgs:story_cc_proto",
        "//modules/common_msgs/task_manager_msgs:task_manager_cc_proto",
        "@com_github_nlohmann_json//:json",
        "@com_google_absl//:absl",
    ],
)

cc_test(
    name = "simulation_world_service_test",
    size = "small",
    srcs = ["simulation_world_service_test.cc"],
    data = [
        "//modules/dreamview/backend/testdata",
    ],
    deps = [
        ":simulation_world_service",
        "//modules/common/math",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_library(
    name = "simulation_world_updater",
    srcs = ["simulation_world_updater.cc"],
    hdrs = ["simulation_world_updater.h"],
    copts = DREAMVIEW_COPTS,
    alwayslink = True,
    deps = [
        ":simulation_world_service",
        "//modules/common/util:util_tool",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/fuel_monitor:fuel_monitor_manager",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/dreamview/backend/map:map_service",
        "//modules/dreamview/backend/perception_camera_updater",
        "//modules/common_msgs/routing_msgs:poi_cc_proto",
        "//modules/common_msgs/task_manager_msgs:task_manager_cc_proto",
        "//modules/dreamview/backend/plugins:plugin_manager",
        "//modules/dreamview/backend/sim_control_manager:sim_control_manager",
        "//modules/dreamview/backend/sim_control",
        "@boost",
    ],
)

cpplint()
