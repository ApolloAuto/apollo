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
        "//modules/audio/proto:audio_event_cc_proto",
        "//modules/audio/proto:audio_cc_proto",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math:quaternion",
        "//modules/common/monitor_log",
        "//modules/common/proto:drive_event_cc_proto",
        "//modules/common/proto:geometry_cc_proto",
        "//modules/common/util",
        "//modules/common/util:map_util",
        "//modules/common/util:points_downsampler",
        "//modules/control/proto:control_cmd_cc_proto",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/map:map_service",
        "//modules/dreamview/proto:simulation_world_cc_proto",
        "//modules/localization/proto:gps_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/perception/proto:perception_obstacle_cc_proto",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/storytelling/proto:story_cc_proto",
        "//modules/task_manager/proto:task_manager_cc_proto",
        "@com_github_nlohmann_json//:json",
        "@com_google_absl//absl/strings",
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
        "//modules/common/math:quaternion",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "simulation_world_updater",
    srcs = ["simulation_world_updater.cc"],
    hdrs = ["simulation_world_updater.h"],
    copts = DREAMVIEW_COPTS,
    deps = [
        ":simulation_world_service",
        "//modules/common/util:map_util",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/data_collection_monitor",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/dreamview/backend/map:map_service",
        "//modules/dreamview/backend/perception_camera_updater",
        "//modules/dreamview/backend/sim_control",
        "//modules/routing/proto:default_routing_cc_proto",
        "//modules/routing/proto:poi_cc_proto",
        "//modules/task_manager/proto:task_manager_cc_proto",
        "@boost",
    ],
)

cpplint()
