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
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math:quaternion",
        "//modules/common/monitor_log",
        "//modules/common/proto:common_proto",
        "//modules/common/util",
        "//modules/common/util:map_util",
        "//modules/common/util:points_downsampler",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/map:map_service",
        "//modules/dreamview/backend/util:trajectory_point_collector",
        "//modules/dreamview/proto:simulation_world_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/planning/proto:planning_proto",
        "//third_party/json",
        "@com_google_protobuf//:protobuf",
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
        "//modules/common/adapters:adapter_manager",
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
    linkopts = [
        "-lboost_thread",
    ],
    deps = [
        ":simulation_world_service",
        "//modules/common/util:map_util",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/handlers:image_handler",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/dreamview/backend/map:map_service",
        "//modules/dreamview/backend/sim_control",
        "@com_google_protobuf//:protobuf",
    ],
)

cpplint()
