load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "simulation_world_service",
    srcs = [
        "simulation_world_service.cc",
    ],
    hdrs = [
        "simulation_world_service.h",
    ],
    copts = ['-DMODULE_NAME=\\"dreamview\\"'],
    deps = [
        "//cyber",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math:quaternion",
        "//modules/common/monitor_log",
        "//modules/common/proto:drive_event_proto",
        "//modules/common/proto:geometry_proto",
        "//modules/common/util",
        "//modules/common/util:map_util",
        "//modules/common/util:points_downsampler",
        "//modules/control/proto:control_proto",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/map:map_service",
        "//modules/dreamview/proto:simulation_world_proto",
        "//modules/localization/proto:gps_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/prediction/proto:prediction_proto",
        "//third_party/json",
    ],
)

cc_test(
    name = "simulation_world_service_test",
    size = "small",
    srcs = [
        "simulation_world_service_test.cc",
    ],
    data = [
        "//modules/dreamview/backend/testdata",
    ],
    deps = [
        ":simulation_world_service",
        "//modules/common/math:quaternion",
        "@gtest//:main",
    ],
)

cc_library(
    name = "simulation_world_updater",
    srcs = [
        "simulation_world_updater.cc",
    ],
    hdrs = [
        "simulation_world_updater.h",
    ],
    copts = ['-DMODULE_NAME=\\"dreamview\\"'],
    linkopts = [
        "-lboost_thread",
    ],
    deps = [
        ":simulation_world_service",
        "//modules/common/util:map_util",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/data_collection_monitor",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/dreamview/backend/map:map_service",
        "//modules/dreamview/backend/sim_control",
    ],
)

cpplint()
