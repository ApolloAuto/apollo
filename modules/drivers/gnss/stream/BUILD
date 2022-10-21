load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gnss_stream",
    deps = [
        ":ntrip_stream",
        ":raw_stream",
        ":serial_stream",
        ":tcp_stream",
        ":udp_stream",
    ],
)

cc_library(
    name = "ntrip_stream",
    srcs = ["ntrip_stream.cc"],
    hdrs = ["tcp_stream.h"],
    deps = [
        ":stream",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/math",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/util",
    ],
)

cc_library(
    name = "raw_stream",
    srcs = ["raw_stream.cc"],
    hdrs = ["raw_stream.h"],
    deps = [
        ":ntrip_stream",
        ":serial_stream",
        ":stream",
        "//cyber",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common_msgs/basic_msgs:drive_state_cc_proto",
        "//modules/common_msgs/basic_msgs:vehicle_signal_cc_proto",
        "//modules/common/util:util_tool",
        "//modules/drivers/gnss/parser:gnss_parser",
        "//modules/drivers/gnss/proto:gnss_status_cc_proto",
        "//modules/common_msgs/sensor_msgs:heading_cc_proto",
        "//modules/common_msgs/localization_msgs:imu_cc_proto",
        "//modules/drivers/gnss/util:gnss_util",
        "@com_google_absl//:absl",
    ],
)

cc_library(
    name = "serial_stream",
    srcs = ["serial_stream.cc"],
    deps = [
        ":stream",
        "//cyber",
        "//modules/drivers/gnss/util:gnss_util",
    ],
)

cc_library(
    name = "tcp_stream",
    srcs = ["tcp_stream.cc"],
    hdrs = ["tcp_stream.h"],
    deps = [
        ":stream",
        "//cyber",
    ],
)

cc_library(
    name = "udp_stream",
    srcs = ["udp_stream.cc"],
    deps = [
        ":stream",
        "//cyber",
        "//modules/drivers/gnss/util:gnss_util",
    ],
)

cc_library(
    name = "stream",
    hdrs = ["stream.h"],
    deps = [
        "//cyber",
        "//modules/drivers/gnss/util:gnss_util",
    ],
)

cpplint()
