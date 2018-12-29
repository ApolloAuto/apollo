load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "grpc_client",
    srcs = [
        "grpc_client.cc",
    ],
    hdrs = [
        "grpc_client.h",
    ],
    linkopts = [
        "-lgrpc++",
    ],
    deps = [
        "//cyber",
        "//modules/v2x/common:v2x_proxy_gflags",
        "//modules/v2x/proto:v2x_service_car_to_obu_grpc",
    ],
)

cc_library(
    name = "grpc_server",
    srcs = [
        "grpc_server.cc",
    ],
    hdrs = [
        "grpc_server.h",
    ],
    linkopts = [
        "-lgrpc++",
    ],
    deps = [
        "//cyber",
        "//modules/v2x/proto:v2x_service_obu_to_car_grpc",
    ],
)

cc_test(
    name = "grpc_client_test",
    size = "small",
    srcs = [
        "grpc_client_test.cc",
    ],
    deps = [
        ":grpc_client",
        "@gtest//:main",
    ],
)

cc_test(
    name = "grpc_server_test",
    size = "small",
    srcs = [
        "grpc_server_test.cc",
    ],
    deps = [
        ":grpc_server",
        "@gtest//:main",
    ],
)

cpplint()
