load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "obu_interface_grpc_impl",
    srcs = ["obu_interface_grpc_impl.cc"],
    hdrs = [
        "obu_interface_abstract_class.h",
        "obu_interface_grpc_impl.h",
    ],
    deps = [
        "//modules/v2x/v2x_proxy/obu_interface/grpc_interface:grpc_client",
        "//modules/v2x/v2x_proxy/obu_interface/grpc_interface:grpc_server",
        "@com_github_grpc_grpc//:grpc++",
    ],
)

cc_test(
    name = "obu_interface_grpc_impl_test",
    size = "small",
    srcs = ["obu_interface_grpc_impl_test.cc"],
    deps = [
        ":obu_interface_grpc_impl",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
