load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gnss_lib",
    srcs = [
        "gnss.cc",
    ],
    hdrs = [
        "gnss.h",
    ],
    deps = [
        "//modules/drivers/gnss/stream:gnss_stream",
        "//modules/common",
        "//modules/common:apollo_app",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor_log"
    ],
)

cc_binary(
    name = "gnss",
    srcs = ["main.cc"],
    deps = [
        ":gnss_lib",
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/monitor_log",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "gnss_gflags",
    srcs = [
        "gnss_gflags.cc",
    ],
    hdrs = [
        "gnss_gflags.h",
    ],
    deps = [
        "//external:gflags",
    ],
)

cpplint()
