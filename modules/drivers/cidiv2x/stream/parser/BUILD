load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cidiv2x_parser",
    srcs = ["cidiv2x_parser.cc"],
    hdrs = [
        "cidiv2x_parser.h",
        ],
    deps = [
        "//external:gflags",
        "//modules/common/monitor_log",
        "//modules/drivers/cidiv2x/stream/proto:stream_proto",
        "//modules/drivers/cidiv2x/proto:cidiv2x_proto",
        "//modules/drivers/cidiv2x/stream/util:stream_util",
        "//modules/common/adapters:adapter_gflags",
        "//cyber"
    ],
)

cpplint()
