load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "websocket",
    srcs = [
        "websocket.cc",
    ],
    hdrs = [
        "websocket.h",
    ],
    deps = [
        "//modules/common:log",
        "//third_party/json",
        "@websocketpp//:websocketpp",
    ],
)

cc_test(
    name = "websocket_test",
    size = "small",
    srcs = [
        "websocket_test.cc",
    ],
    deps = [
        ":websocket",
        "//modules/common:log",
        "@gtest//:main",
        "@websocketpp//:websocketpp",
    ],
)

cc_library(
    name = "trajectory_point_collector",
    srcs = [
        "trajectory_point_collector.cc",
    ],
    hdrs = [
        "trajectory_point_collector.h",
    ],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/planning/proto:planning_proto",
        "//modules/dreamview/proto:simulation_world_proto",
    ],
)

cc_test(
    name = "trajectory_point_collector_test",
    size = "small",
    srcs = [
        "trajectory_point_collector_test.cc",
    ],
    deps = [
        ":trajectory_point_collector",
        "@gtest//:main",
    ],
)

cc_library(
    name = "simulation_world_service",
    srcs = [
        "simulation_world_service.cc",
    ],
    hdrs = [
        "simulation_world_service.h",
    ],
    deps = [
        ":trajectory_point_collector",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/math:quaternion",
        "//modules/common/proto:common_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/dreamview/proto:simulation_world_proto",
        "//third_party/json",
        "@com_github_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "simulation_world_service_test",
    size = "small",
    srcs = [
        "simulation_world_service_test.cc",
    ],
    deps = [
        ":simulation_world_service",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/math:quaternion",
        "@gtest//:main",
    ],
)

cc_library(
    name = "backend",
    srcs = ["server.cc"],
    deps = [
        ":simulation_world_service",
        ":websocket",
        "@civetweb//:civetweb++",
        "@com_github_google_protobuf//:protobuf",
    ],
)

cpplint()
