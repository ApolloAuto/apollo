package(default_visibility = ["//visibility:public"])

cc_library(
    name = "adapter_gflags",
    srcs = [
        "adapter_gflags.cc",
    ],
    hdrs = [
        "adapter_gflags.h",
    ],
    deps = [
        "//external:gflags",
    ],
)

cc_library(
    name = "adapter_manager",
    srcs = [
        "adapter_manager.cc",
    ],
    hdrs = [
        "adapter_manager.h",
    ],
    deps = [
        ":adapter_gflags",
        ":message_adapters",
        "//modules/common",
        "//modules/common/adapters/proto:adapter_config_proto",
        "//modules/common/monitor/proto:monitor_proto",
        "//modules/common/util",
        "//third_party/ros:ros_common",
        "@glog//:glog",
    ],
)

cc_library(
    name = "adapter",
    hdrs = [
        "adapter.h",
    ],
    deps = [
        ":adapter_gflags",
        "//modules/common/proto:common_proto",
        "//modules/common/time",
        "//modules/common/util",
        "@com_github_google_protobuf//:protobuf",
        "@glog//:glog",
    ],
)

cc_test(
    name = "adapter_test",
    size = "small",
    srcs = [
        "adapter_test.cc",
    ],
    tags = [
        "external",
    ],
    deps = [
        ":adapter",
        ":adapter_gflags",
        "//modules/localization/proto:localization_proto",
        "@gtest//:main",
    ],
)

cc_library(
    name = "message_adapters",
    hdrs = [
        "message_adapters.h",
    ],
    deps = [
        ":adapter",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/monitor/proto:monitor_proto",
        "//modules/control/proto:control_proto",
        "//modules/decision/proto:decision_proto",
        "//modules/localization/proto:camera_proto",
        "//modules/localization/proto:gps_proto",
        "//modules/localization/proto:imu_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/prediction/proto:prediction_proto",
        "@glog//:glog",
    ],
)
