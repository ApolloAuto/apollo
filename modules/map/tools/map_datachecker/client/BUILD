load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "map_datachecker_client",
    srcs = ["main.cc"],
    deps = [
        ":client",
        ":client_gflags",
        "@boost",
        "@com_github_grpc_grpc//:grpc++",
    ],
)

cc_library(
    name = "client",
    srcs = ["client.cc"],
    hdrs = [
        "client.h",
        "client_alignment.h",
    ],
    deps = [
        ":client_channel_checker",
        ":client_common",
        ":client_gflags",
        ":client_loops_check",
        "//cyber",
        "@boost",
        "@com_github_grpc_grpc//:grpc++",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
    ],
)

cc_library(
    name = "client_channel_checker",
    srcs = ["client_channel_checker.cc"],
    hdrs = ["client_channel_checker.h"],
    deps = [
        ":client_common",
        ":client_gflags",
        ":exception_handler",
        "//cyber",
        "//modules/map/tools/map_datachecker/proto:collection_service_cc_grpc",
        "@boost",
        "@com_github_grpc_grpc//:grpc++",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
    ],
)

cc_library(
    name = "client_common",
    srcs = ["client_common.cc"],
    hdrs = ["client_common.h"],
    deps = [
        "//cyber",
        "@boost",
        "@com_github_grpc_grpc//:grpc++",
    ],
)

cc_library(
    name = "client_gflags",
    srcs = ["client_gflags.cc"],
    hdrs = ["client_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "client_loops_check",
    srcs = ["client_loops_check.cc"],
    hdrs = ["client_loops_check.h"],
    deps = [
        ":client_common",
        ":client_gflags",
        ":exception_handler",
        "//cyber",
        "//modules/map/tools/map_datachecker/proto:collection_service_cc_grpc",
        "@boost",
        "@com_github_grpc_grpc//:grpc++",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
    ],
)

cc_library(
    name = "exception_handler",
    srcs = ["exception_handler.cc"],
    hdrs = ["exception_handler.h"],
    deps = [
        ":client_common",
        "//cyber",
        "//modules/map/tools/map_datachecker/proto:collection_error_code_cc_proto",
    ],
)

cpplint()
