#load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "cyber_config_cc_proto",
    deps = [
        ":cyber_config_proto",
    ],
)

proto_library(
    name = "cyber_config_proto",
    srcs = [
        "cyber_config.proto",
    ],
    deps = [
        ":croutine_conf_proto",
        ":scheduler_conf_proto",
        ":transport_conf_proto",
        ":log_conf_proto",
        ":perf_conf_proto",
        ":run_mode_conf_proto",
        ":record_conf_proto",
        ":topology_conf_proto",
    ],
)

cc_proto_library(
    name = "croutine_conf_cc_proto",
    deps = [
        ":croutine_conf_proto",
    ],
)

proto_library(
    name = "croutine_conf_proto",
    srcs = [
        "croutine_conf.proto",
    ],
)

cc_proto_library(
    name = "scheduler_conf_cc_proto",
    deps = [
        ":scheduler_conf_proto",
    ],
)

proto_library(
    name = "scheduler_conf_proto",
    srcs = [
        "scheduler_conf.proto",
    ],
)

cc_proto_library(
    name = "transport_conf_cc_proto",
    deps = [
        ":transport_conf_proto",
    ],
)

proto_library(
    name = "transport_conf_proto",
    srcs = [
        "transport_conf.proto",
    ],
)

cc_proto_library(
    name = "log_conf_cc_proto",
    deps = [
        ":log_conf_proto",
    ],
)

proto_library(
    name = "log_conf_proto",
    srcs = [
        "log_conf.proto",
    ],
)

cc_proto_library(
    name = "perf_conf_cc_proto",
    deps = [
        ":perf_conf_proto",
    ],
)

proto_library(
    name = "perf_conf_proto",
    srcs = [
        "perf_conf.proto",
    ],
)

cc_proto_library(
    name = "run_mode_conf_cc_proto",
    deps = [
        ":run_mode_conf_proto",
    ],
)

proto_library(
    name = "run_mode_conf_proto",
    srcs = [
        "run_mode_conf.proto",
    ],
)

cc_proto_library(
    name = "record_conf_cc_proto",
    deps = [
        ":record_conf_proto",
    ],
)

proto_library(
    name = "record_conf_proto",
    srcs = [
        "record_conf.proto",
    ],
)

cc_proto_library(
    name = "proto_desc_cc_proto",
    deps = [
        ":proto_desc_proto",
    ],
)

proto_library(
    name = "proto_desc_proto",
    srcs = [
        "proto_desc.proto",
    ],
)

cc_proto_library(
    name = "record_cc_proto",
    deps = [
        ":record_proto",
    ],
)

proto_library(
    name = "record_proto",
    srcs = [
        "record.proto",
    ],
)

cc_proto_library(
    name = "unit_test_cc_proto",
    deps = [
        ":unit_test_proto",
    ],
)

proto_library(
    name = "unit_test_proto",
    srcs = [
        "unit_test.proto",
    ],
)

cc_proto_library(
    name = "perception_cc_proto",
    deps = [
        ":perception_proto",
    ],
)

proto_library(
    name = "perception_proto",
    srcs = [
        "perception.proto",
    ],
)

cc_proto_library(
    name = "driver_cc_proto",
    deps = [
        ":driver_proto",
    ],
)

proto_library(
    name = "driver_proto",
    srcs = [
        "driver.proto",
    ],
)

cc_proto_library(
    name = "chatter_cc_proto",
    deps = [
        ":chatter_proto",
    ],
)

proto_library(
    name = "chatter_proto",
    srcs = [
        "chatter.proto",
    ],
)

cc_proto_library(
    name = "parameter_cc_proto",
    deps = [
        ":parameter_proto",
    ],
)

proto_library(
    name = "parameter_proto",
    srcs = [
        "parameter.proto",
    ],
)

cc_proto_library(
    name = "qos_profile_cc_proto",
    deps = [
        ":qos_profile_proto",
    ],
)

proto_library(
    name = "qos_profile_proto",
    srcs = [
        "qos_profile.proto",
    ],
)

cc_proto_library(
    name = "component_config_cc_proto",
    deps = [
        ":component_config_proto",
    ],
)

proto_library(
    name = "component_config_proto",
    srcs = [
        "component_config.proto",
    ],
    deps = [
        ":qos_profile_proto",
    ],
)

cc_proto_library(
    name = "dag_config_cc_proto",
    deps = [
        ":dag_config_proto",
    ],
)

proto_library(
    name = "dag_config_proto",
    srcs = [
        "dag_config.proto",
    ],
    deps = [
        ":component_config_proto",
    ],
)

cc_proto_library(
    name = "role_attributes_cc_proto",
    deps = [
        ":role_attributes_proto",
    ],
)

proto_library(
    name = "role_attributes_proto",
    srcs = [
        "role_attributes.proto",
    ],
    deps = [
        ":qos_profile_proto",
    ],
)

cc_proto_library(
    name = "topology_change_cc_proto",
    deps = [
        ":topology_change_proto",
    ],
)

proto_library(
    name = "topology_change_proto",
    srcs = [
        "topology_change.proto",
    ],
    deps = [
        ":role_attributes_proto",
    ],
)

cc_proto_library(
    name = "topology_conf_cc_proto",
    deps = [
        ":topology_conf_proto",
    ],
)

proto_library(
    name = "topology_conf_proto",
    srcs = [
        "topology_conf.proto",
    ],
)

#cpplint()
