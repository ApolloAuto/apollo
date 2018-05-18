load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gnss_stream",
    deps = [
        ":ntrip_stream",
        ":serial_stream",
        ":raw_stream",
        ":tcp_stream",
        ":udp_stream"
    ],
)

cc_library(
    name = "ntrip_stream",
    srcs = ["ntrip_stream.cc"],
    hdrs = ["tcp_stream.h"],
    deps = [
        ":stream",
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/monitor_log",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "raw_stream",
    srcs = ["raw_stream.cc"],
    hdrs = ["raw_stream.h"],
    deps = [
        ":stream",
        ":serial_stream",
        ":ntrip_stream",
        "//modules/drivers/gnss/util:gnss_util",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//modules/drivers/gnss:gnss_gflags",
        "//modules/drivers/gnss/parser:data_parser",
        "//modules/drivers/gnss/parser:rtcm_parser",
        "//modules/drivers/gnss/parser:rtcm3_parser",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "@ros//:ros_common", 
    ],
)

cc_library(
    name = "serial_stream",
    srcs = ["serial_stream.cc"],
    deps = [
        ":stream",
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/monitor_log",
        "//modules/drivers/gnss/util:gnss_util",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "tcp_stream",
    srcs = ["tcp_stream.cc"],
    hdrs = ["tcp_stream.h"],
    deps = [
        ":stream",
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/monitor_log",
        "@ros//:ros_common"
    ],
)

cc_library(
    name = "udp_stream",
    srcs = ["udp_stream.cc"],
    deps = [
        ":stream",
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/monitor_log",
        "//modules/drivers/gnss/util:gnss_util",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "stream",
    hdrs = ["stream.h"],
    deps = [
        "//modules/common:log",
        "//modules/drivers/gnss/util:gnss_util",
    ],
)

cpplint()
