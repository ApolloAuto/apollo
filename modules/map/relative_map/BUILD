load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "relative_map_conf",
    srcs = glob(["conf/**"]),
)

cc_library(
    name = "relative_map_lib",
    srcs = [
        "relative_map.cc",
    ],
    hdrs = [
        "relative_map.h",
        "relative_map_interface.h",
    ],
    data = [
        ":relative_map_conf",
    ],
    deps = [
        "//modules/common",
        "//modules/common:apollo_app",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/map/proto:map_proto",
        "//modules/map/relative_map/common:relative_map_gflags",
    ],
)

cc_binary(
    name = "relative_map",
    srcs = ["main.cc"],
    deps = [
        ":relative_map_lib",
        "//external:gflags",
    ],
)

cpplint()
