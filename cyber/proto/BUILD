load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "cyber_conf_cc_proto",
    deps = [
        ":cyber_conf_proto",
    ],
)

proto_library(
    name = "cyber_conf_proto",
    srcs = ["cyber_conf.proto"],
    deps = [
        ":choreography_conf_proto",
        ":perf_conf_proto",
        ":run_mode_conf_proto",
        ":scheduler_conf_proto",
        ":transport_conf_proto",
    ],
)

cc_proto_library(
    name = "choreography_conf_cc_proto",
    deps = [
        ":choreography_conf_proto",
    ],
)

proto_library(
    name = "choreography_conf_proto",
    srcs = ["choreography_conf.proto"],
)

cc_proto_library(
    name = "classic_conf_cc_proto",
    deps = [
        ":classic_conf_proto",
    ],
)

proto_library(
    name = "classic_conf_proto",
    srcs = ["classic_conf.proto"],
)

cc_proto_library(
    name = "perf_conf_cc_proto",
    deps = [
        ":perf_conf_proto",
    ],
)

proto_library(
    name = "perf_conf_proto",
    srcs = ["perf_conf.proto"],
)

cc_proto_library(
    name = "scheduler_conf_cc_proto",
    deps = [
        ":scheduler_conf_proto",
    ],
)

proto_library(
    name = "scheduler_conf_proto",
    srcs = ["scheduler_conf.proto"],
    deps = [
        ":choreography_conf_proto",
        ":classic_conf_proto",
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
    srcs = ["transport_conf.proto"],
)

cc_proto_library(
    name = "run_mode_conf_cc_proto",
    deps = [
        ":run_mode_conf_proto",
    ],
)

proto_library(
    name = "run_mode_conf_proto",
    srcs = ["run_mode_conf.proto"],
)

cc_proto_library(
    name = "proto_desc_cc_proto",
    deps = [
        ":proto_desc_proto",
    ],
)

proto_library(
    name = "proto_desc_proto",
    srcs = ["proto_desc.proto"],
)

cc_proto_library(
    name = "record_cc_proto",
    deps = [
        ":record_proto",
    ],
)

proto_library(
    name = "record_proto",
    srcs = ["record.proto"],
)

py_proto(
    name = "record_pb2",
    src = "record.proto",
)

cc_proto_library(
    name = "unit_test_cc_proto",
    deps = [
        ":unit_test_proto",
    ],
)

proto_library(
    name = "unit_test_proto",
    srcs = ["unit_test.proto"],
)

cc_proto_library(
    name = "perception_cc_proto",
    deps = [
        ":perception_proto",
    ],
)

proto_library(
    name = "perception_proto",
    srcs = ["perception.proto"],
)

cc_proto_library(
    name = "parameter_cc_proto",
    deps = [
        ":parameter_proto",
    ],
)

proto_library(
    name = "parameter_proto",
    srcs = ["parameter.proto"],
)

cc_proto_library(
    name = "qos_profile_cc_proto",
    deps = [
        ":qos_profile_proto",
    ],
)

proto_library(
    name = "qos_profile_proto",
    srcs = ["qos_profile.proto"],
)

cc_proto_library(
    name = "component_conf_cc_proto",
    deps = [
        ":component_conf_proto",
    ],
)

proto_library(
    name = "component_conf_proto",
    srcs = ["component_conf.proto"],
    deps = [
        ":qos_profile_proto",
    ],
)

cc_proto_library(
    name = "dag_conf_cc_proto",
    deps = [
        ":dag_conf_proto",
    ],
)

proto_library(
    name = "dag_conf_proto",
    srcs = ["dag_conf.proto"],
    deps = [
        ":component_conf_proto",
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
    srcs = ["role_attributes.proto"],
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
    srcs = ["topology_change.proto"],
    deps = [
        ":role_attributes_proto",
    ],
)

py_proto(
    name = "perf_conf_pb2",
    src = "perf_conf.proto",
)

py_proto(
    name = "run_mode_conf_pb2",
    src = "run_mode_conf.proto",
)

py_proto(
    name = "dag_conf_pb2",
    src = "dag_conf.proto",
    deps = [
        "//cyber/proto:component_conf_pb2",
    ],
)

py_proto(
    name = "unit_test_pb2",
    src = "unit_test.proto",
)

py_proto(
    name = "choreography_conf_pb2",
    src = "choreography_conf.proto",
)

py_proto(
    name = "role_attributes_pb2",
    src = "role_attributes.proto",
    deps = [
        "//cyber/proto:qos_profile_pb2",
    ],
)

py_proto(
    name = "proto_desc_pb2",
    src = "proto_desc.proto",
)

py_proto(
    name = "perception_pb2",
    src = "perception.proto",
)

py_proto(
    name = "scheduler_conf_pb2",
    src = "scheduler_conf.proto",
    deps = [
        "//cyber/proto:choreography_conf_pb2",
        "//cyber/proto:classic_conf_pb2",
    ],
)

py_proto(
    name = "parameter_pb2",
    src = "parameter.proto",
)

py_proto(
    name = "classic_conf_pb2",
    src = "classic_conf.proto",
)

py_proto(
    name = "component_conf_pb2",
    src = "component_conf.proto",
    deps = [
        "//cyber/proto:qos_profile_pb2",
    ],
)

py_proto(
    name = "transport_conf_pb2",
    src = "transport_conf.proto",
)

py_proto(
    name = "qos_profile_pb2",
    src = "qos_profile.proto",
)

py_proto(
    name = "cyber_conf_pb2",
    src = "cyber_conf.proto",
    deps = [
        "//cyber/proto:perf_conf_pb2",
        "//cyber/proto:run_mode_conf_pb2",
        "//cyber/proto:scheduler_conf_pb2",
        "//cyber/proto:transport_conf_pb2",
    ],
)

py_proto(
    name = "topology_change_pb2",
    src = "topology_change.proto",
    deps = [
        "//cyber/proto:role_attributes_pb2",
    ],
)
