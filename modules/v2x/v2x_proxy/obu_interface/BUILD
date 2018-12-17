load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "obu_interface_grpc_impl",
    srcs = [
        "obu_interface_grpc_impl.cc",
    ],
    hdrs = [
        "obu_interface_abstract_class.h",
        "obu_interface_grpc_impl.h",
    ],
    linkopts = [
        "-lgrpc++",
    ],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/localization/proto:localization_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/v2x/common:v2x_proxy_gflags",
        "//modules/v2x/proto:v2x_carstatus_proto",
        "//modules/v2x/proto:v2x_traffic_light_proto",
        "//modules/v2x/v2x_proxy/obu_interface/grpc_interface:grpc_client",
        "//modules/v2x/v2x_proxy/obu_interface/grpc_interface:grpc_server",
    ],
)

cc_test(
    name = "obu_interface_grpc_impl_test",
    size = "small",
    srcs = [
        "obu_interface_grpc_impl_test.cc",
    ],
    deps = [
        ":obu_interface_grpc_impl",
        "@gtest//:main",
    ],
)

cpplint()
