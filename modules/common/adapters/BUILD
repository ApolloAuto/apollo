load("//tools:cpplint.bzl", "cpplint")

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
        "@glog//:glog",
        "@ros//:ros_common",
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
        "@com_google_protobuf//:protobuf",
        "@glog//:glog",
        "@ros//:ros_common",
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
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "message_adapters",
    hdrs = [
        "message_adapters.h",
    ],
    deps = [
        ":adapter",
        "//modules/calibration/republish_msg/proto:relative_odometry_proto",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/monitor/proto:monitor_proto",
        "//modules/common/proto:gnss_status_proto",
        "//modules/control/proto:control_proto",
        "//modules/dreamview/proto:hmi_status_proto",
        "//modules/drivers/gnss/proto:ins_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/hmi/proto:hmi_message_proto",
        "//modules/localization/proto:gps_proto",
        "//modules/localization/proto:imu_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/map/proto:map_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/prediction/proto:prediction_proto",
        "//modules/routing/proto:routing_proto",
        "@glog//:glog",
    ],
)

cpplint()
