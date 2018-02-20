load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "monitor_conf_proto",
    deps = [
        ":monitor_conf_proto_lib",
    ],
)

proto_library(
    name = "monitor_conf_proto_lib",
    srcs = ["monitor_conf.proto"],
    deps = [
        "//modules/common/adapters/proto:adapter_config_proto_lib",
    ],
)

cc_proto_library(
    name = "system_status_proto",
    deps = [
        ":system_status_proto_lib",
    ],
)

proto_library(
    name = "system_status_proto_lib",
    srcs = ["system_status.proto"],
    deps = [
        ":monitor_conf_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "online_report_proto",
    deps = [
        ":online_report_proto_lib",
    ],
)

proto_library(
    name = "online_report_proto_lib",
    srcs = ["online_report.proto"],
    deps = [
        "//modules/common/vehicle_state/proto:vehicle_state_proto_lib",
        "//modules/data/proto:static_info_proto_lib",
    ],
)

cpplint()
