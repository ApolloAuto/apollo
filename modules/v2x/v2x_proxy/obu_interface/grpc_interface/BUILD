load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "grpc_client",
    srcs = ["grpc_client.cc"],
    hdrs = ["grpc_client.h"],
    deps = [
        "//cyber",
        "//modules/v2x/common:v2x_proxy_gflags",
        "//modules/v2x/proto:v2x_service_car_to_obu_cc_grpc",
        "//modules/v2x/proto:v2x_service_obu_to_car_cc_grpc",
        "@com_github_grpc_grpc//:grpc++",
    ],
)

cc_library(
    name = "grpc_server",
    srcs = ["grpc_server.cc"],
    hdrs = ["grpc_server.h"],
    deps = [
        "//cyber",
        "//modules/perception/proto:perception_obstacle_cc_proto",
        "//modules/perception/proto:traffic_light_detection_cc_proto",
        "//modules/v2x/common:v2x_proxy_gflags",
        "//modules/v2x/proto:v2x_obu_rsi_cc_proto",
        "//modules/v2x/proto:v2x_service_car_to_obu_cc_grpc",
        "//modules/v2x/proto:v2x_service_obu_to_car_cc_grpc",
        "//modules/v2x/proto:v2x_traffic_light_cc_proto",
        "@com_github_grpc_grpc//:grpc++",
    ],
)

cc_test(
    name = "grpc_client_test",
    size = "small",
    srcs = ["grpc_client_test.cc"],
    deps = [
        ":grpc_client",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "grpc_server_test",
    size = "small",
    srcs = ["grpc_server_test.cc"],
    deps = [
        ":grpc_server",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
