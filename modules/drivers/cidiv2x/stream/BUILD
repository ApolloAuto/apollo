load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cidiv2x_stream",
    deps = [
        ":serial_stream",
        ":raw_stream",
        ":tcp_stream",
        ":udp_stream"
    ],
)

cc_library(
    name = "raw_stream",
    srcs = ["raw_stream.cc"],
    hdrs = ["raw_stream.h"],
    deps = [
        ":stream",
        ":serial_stream",
        "//cyber",
        "//modules/drivers/cidiv2x/stream/parser:cidiv2x_parser",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
    ],
)

cc_library(
    name = "serial_stream",
    srcs = ["serial_stream.cc"],
    deps = [
        ":stream",
        "//external:gflags",
        "//modules/common/monitor_log",
        "//modules/drivers/cidiv2x/stream/util:stream_util",
        "//cyber",
    ],
)

cc_library(
    name = "tcp_stream",
    srcs = ["tcp_stream.cc"],
    hdrs = ["tcp_stream.h"],
    deps = [
        ":stream",
        "//external:gflags",
        "//modules/common/monitor_log",
        "//cyber"
    ],
)

cc_library(
    name = "udp_stream",
    srcs = ["udp_stream.cc"],
    deps = [
        ":stream",
        "//external:gflags",
        "//modules/common/monitor_log",
        "//cyber",
    ],
)

cc_library(
    name = "stream",
    hdrs = ["stream.h"],
    deps = [
        "//cyber",
        "//modules/drivers/cidiv2x/stream/util:stream_util",
    ],
)

cpplint()
