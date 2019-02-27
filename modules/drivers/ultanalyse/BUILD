load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ultanalyse_lib",
    srcs = ["ultanalyse.cc"],
    hdrs = ["ultanalyse.h"],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/adapters:adapter_manager",        
        "//modules/common:apollo_app",
        "//modules/common:log",
        "//modules/control/proto:control_proto",        
        "//modules/drivers/ultanalyse/common:ultanalyse_gflags",
        "//modules/drivers/ultanalyse/proto:ultanalyse_proto",
        "//modules/monitor/proto:system_status_proto",
    ],
)

cc_binary(
    name = "ultanalyse",
    srcs = ["main.cc"],
    deps = [
        ":ultanalyse_lib",
        "//external:gflags",
        "//modules/common:log",
        "@ros//:ros_common",
    ],
)

cpplint()
