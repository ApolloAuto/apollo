load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("@com_github_grpc_grpc//bazel:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "daemon_cmd_cc_proto",
    deps = [
        ":daemon_cmd_proto",
    ],
)

proto_library(
    name = "daemon_cmd_proto",
    srcs = ["daemon_cmd.proto"],
    deps = [
        "//modules/common/proto:header_proto",
    ],
)

py_proto_library(
    name = "daemon_cmd_py_pb2",
    deps = [
        ":daemon_cmd_proto",
    ],
)

cc_proto_library(
    name = "daemon_rpt_cc_proto",
    deps = [
        ":daemon_rpt_proto",
    ],
)

proto_library(
    name = "daemon_rpt_proto",
    srcs = ["daemon_rpt.proto"],
    deps = [
        "//modules/common/proto:header_proto",
    ],
)

py_proto_library(
    name = "daemon_rpt_py_pb2",
    deps = [
        ":daemon_rpt_proto",
    ],
)

cc_proto_library(
    name = "modem_info_cc_proto",
    deps = [
        ":modem_info_proto",
    ],
)

proto_library(
    name = "modem_info_proto",
    srcs = ["modem_info.proto"],
    deps = [
        "//modules/common/proto:header_proto",
    ],
)

py_proto_library(
    name = "modem_info_py_pb2",
    deps = [
        ":modem_info_proto",
    ],
)
