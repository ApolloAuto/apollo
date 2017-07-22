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
        "@civetweb//:civetweb++",
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
        "//modules/dreamview/proto:simulation_world_proto",
        "//modules/planning/proto:planning_proto",
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
        ":map_service",
        ":trajectory_point_collector",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/math:quaternion",
        "//modules/common/proto:common_proto",
        "//modules/dreamview/proto:simulation_world_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/planning/proto:planning_proto",
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

filegroup(
    name = "testdata",
    srcs = glob(["testdata/*"]),
)

cc_library(
    name = "map_service",
    hdrs = [
        "map_service.h"
    ],
    srcs = [
        "map_service.cc"
    ],
    deps = [
        "//modules/map/hdmap:hdmap",
        "//third_party/json",
        "@glog//:glog",
    ],
)

cc_test(
    name = "map_service_test",
    size = "small",
    srcs = [
        "map_service_test.cc",
    ],
    data = [":testdata"],
    deps = [
        ":map_service",
        "@gtest//:main",
    ],
)

cc_library(
    name = "simulation_world_updater",
    hdrs = [
        "simulation_world_updater.h"
    ],
    srcs = [
        "simulation_world_updater.cc",
    ],
    deps = [
        ":map_service",
        ":simulation_world_service",
        ":websocket",
        "//external:gflags",
        "@com_github_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "backend",
    srcs = ["server.cc"],
    deps = [
        ":simulation_world_updater",
        ":websocket",
        "//modules/common/adapters:adapter_manager",
        "@civetweb//:civetweb++",
    ],
)

cpplint()
