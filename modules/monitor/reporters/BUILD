load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "static_info_reporter",
    srcs = ["static_info_reporter.cc"],
    hdrs = ["static_info_reporter.h"],
    deps = [
        "//modules/common/adapters:adapter_manager",
        "//modules/data/util:info_collector",
        "//modules/monitor/common:recurrent_runner",
    ],
)

cc_library(
    name = "vehicle_state_reporter",
    srcs = ["vehicle_state_reporter.cc"],
    hdrs = ["vehicle_state_reporter.h"],
    deps = [
        "//modules/common/util:http_client",
        "//modules/common/util:json_util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/data/util:info_collector",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
        "//modules/monitor/proto:online_report_proto",
    ],
)

cpplint()
