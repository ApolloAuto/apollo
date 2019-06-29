load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "map_datachecker_client",
    deps = [
        ":map_datachecker_client_lib",
    ],
)

cc_library(
    name = "map_datachecker_client_lib",
    srcs = [
        "main.cc",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lgrpc++",
    ],
    deps = [
        ":client",
        ":client_gflags",
    ],
)

cc_library(
    name = "client",
    srcs = [
        "client.cc",
    ],
    hdrs = [
        "client.h",
        "client_alignment.h",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lgrpc++",
    ],
    deps = [
        ":client_channel_checker",
        ":client_common",
        ":client_gflags",
        ":client_loops_check",
        "//cyber",
        "@yaml_cpp//:yaml",
    ],
)

cc_library(
    name = "client_channel_checker",
    srcs = [
        "client_channel_checker.cc",
    ],
    hdrs = [
        "client_channel_checker.h",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lgrpc++",
    ],
    deps = [
        ":client_common",
        ":client_gflags",
        ":exception_handler",
        "//cyber",
        "//modules/map/tools/map_datachecker/proto:map_datachecker_grpc",
        "@yaml_cpp//:yaml",
    ],
)

cc_library(
    name = "client_common",
    srcs = [
        "client_common.cc",
    ],
    hdrs = [
        "client_common.h",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lgrpc++",
    ],
    deps = [
        "//cyber",
    ],
)

cc_library(
    name = "client_gflags",
    srcs = [
        "client_gflags.cc",
    ],
    hdrs = [
        "client_gflags.h",
    ],
    deps = [
        "//external:gflags",
    ],
)

cc_library(
    name = "client_loops_check",
    srcs = [
        "client_loops_check.cc",
    ],
    hdrs = [
        "client_loops_check.h",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lgrpc++",
    ],
    deps = [
        ":client_common",
        ":client_gflags",
        ":exception_handler",
        "//cyber",
        "//modules/map/tools/map_datachecker/proto:map_datachecker_grpc",
        "@yaml_cpp//:yaml",
    ],
)

cc_library(
    name = "exception_handler",
    srcs = [
        "exception_handler.cc",
    ],
    hdrs = [
        "exception_handler.h",
    ],
    deps = [
        ":client_common",
        "//cyber",
        "//modules/map/tools/map_datachecker/proto:map_datachecker_proto",
    ],
)

cpplint()
