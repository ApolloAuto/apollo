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
        "//modules/common/proto:pnc_point_cc_proto",
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
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs/proto:vehicle_config_cc_proto",
        "//modules/common/proto:drive_state_cc_proto",
        "//modules/common/proto:vehicle_signal_cc_proto",
        "//modules/common/util:message_util",
        "//modules/drivers/gnss/parser:gnss_parser",
        "//modules/drivers/gnss/proto:gnss_status_cc_proto",
        "//modules/drivers/gnss/proto:heading_cc_proto",
        "//modules/drivers/gnss/proto:imu_cc_proto",
        "//modules/drivers/gnss/util:gnss_util",
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
