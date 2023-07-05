load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

pb_deps = ["@com_google_protobuf//:protobuf_python"]

py_library(
    name = "topology_change_py_pb2",
    srcs = ["topology_change_py_pb2.py"],
    deps = [":role_attributes_py_pb2"]
)

py_library(
    name = "dag_conf_py_pb2",
    srcs = ["dag_conf_py_pb2.py"],
    deps = [":component_conf_py_pb2"]
)

py_library(
    name = "proto_desc_py_pb2",
    srcs = ["proto_desc_py_pb2.py"],
    deps = pb_deps
)

py_library(
    name = "choreography_conf_py_pb2",
    srcs = ["choreography_conf_py_pb2.py"],
    deps = pb_deps
)


py_library(
    name = "record_py_pb2",
    srcs = ["record_py_pb2.py"],
    deps = pb_deps
)


py_library(
    name = "component_conf_py_pb2",
    srcs = ["component_conf_py_pb2.py"],
    deps = [":qos_profile_py_pb2"]
)


py_library(
    name = "cyber_conf_py_pb2",
    srcs = ["cyber_conf_py_pb2.py"],
    deps = [
        ":scheduler_conf_py_pb2",
        ":transport_conf_py_pb2",
        ":run_mode_conf_py_pb2",
        ":perf_conf_py_pb2",
    ]
)

py_library(
    name = "perf_conf_py_pb2",
    srcs = ["perf_conf_py_pb2.py"],
    deps = pb_deps
)

py_library(
    name = "classic_conf_py_pb2",
    srcs = ["classic_conf_py_pb2.py"],
    deps = pb_deps
)

py_library(
    name = "parameter_py_pb2",
    srcs = ["parameter_py_pb2.py"],
    deps = pb_deps
)

py_library(
    name = "unit_test_py_pb2",
    srcs = ["unit_test_py_pb2.py"],
    deps = pb_deps
)

py_library(
    name = "scheduler_conf_py_pb2",
    srcs = ["scheduler_conf_py_pb2.py"],
    deps = [
        ":classic_conf_py_pb2",
        ":choreography_conf_py_pb2",
    ]
)

py_library(
    name = "transport_conf_py_pb2",
    srcs = ["transport_conf_py_pb2.py"],
    deps = pb_deps
)

py_library(
    name = "qos_profile_py_pb2",
    srcs = ["qos_profile_py_pb2.py"],
    deps = pb_deps
)

py_library(
    name = "run_mode_conf_py_pb2",
    srcs = ["run_mode_conf_py_pb2.py"],
    deps = pb_deps
)

py_library(
    name = "role_attributes_py_pb2",
    srcs = ["role_attributes_py_pb2.py"],
    deps = [
        ":qos_profile_py_pb2",
    ]
)

py_library(
    name = "clock_py_pb2",
    srcs = ["clock_py_pb2.py"],
    deps = pb_deps
)